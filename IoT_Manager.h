/* Projeto Láb. Iot, Conexão Esp32 ao servidor Thingsboard, UFPR - Autor: João André A. da Silva */
#ifndef THINGSBOARD_MANAGER
#define THINGSBOARD_MANAGER

#include "WifiManager.h"
#include <string>
#include <time.h>
#include <Server_Side_RPC.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <HTTPClient.h>
#include <cmath>

#ifndef MAX_RPC_CALLBACKS
  #define MAX_RPC_CALLBACKS 2
#endif
String Input(String message);

enum CommType {
   MQTT,
   HTTPS
};

struct IoTMetaData {
   std::vector<std::vector<float>> data_list;
   unsigned timestamp;

   void Clear_Meta_Data();
   void Append_Data(std::vector<float> data);
};

constexpr size_t MEMORY_THRESHOLD = 2000;

class IoT_Manager {
   public:
      CommType commtype;
      IoT_Manager(const char* Server, const char* Token, const uint16_t Port, const uint16_t MeasurementInterval, const std::array<RPC_Callback, MAX_RPC_CALLBACKS>* callbacks);
      IoT_Manager(const char* Server, const char* Token, const uint16_t Port, const uint16_t MeasurementInterval);
      bool IsThingsBoardConnected();
      bool NextMeasurement();
      bool SendData_MQTT(IoTMetaData& metaData, const std::vector<const char*>& AtributesList);
      bool SendData_HTTPS(IoTMetaData& metaData, const std::vector<const char*>& AtributesList);
      void Initialize(bool startSecondZero=true);
   private:
      bool Init_MQTT(bool SubscribeToRPC=false);
      bool IsReady2Send();
      String GetDataJson2String(std::vector<float>& data, const std::vector<const char*>& AtributesList, time_t& timestamp);
      const char* ThingsBoard_Server;
      const char* ThingsBoard_Token;
      uint16_t ThingsBoard_Port;
      const uint16_t Measurement_Interval; // Segundos
      static constexpr uint8_t MAX_MESSAGE_SEND_RECEIVE_SIZE = 256U;

      Arduino_MQTT_Client* mqttClient = nullptr;
      ThingsBoard* tb = nullptr;
      const std::array<RPC_Callback, MAX_RPC_CALLBACKS>* Callbacks;
      Server_Side_RPC<MAX_RPC_CALLBACKS, MAX_RPC_CALLBACKS> rpc;
      std::array<IAPI_Implementation*, 1> apis = { &rpc };

      time_t measurement_timestamp = 0; 
      unsigned long currentMillis = 0;
      unsigned long nextCycleMillis = 0;

      static constexpr const char* NTP_SERVER = "pool.ntp.org"; // Servidor para consultar tempo preciso
      static constexpr int32_t GMT = (-3 * 3600); // Fuso do GMT
      static constexpr int8_t DAYLIGHT_OFFSET = 0; // Fuso do horário de verão
};

#endif