/* Projeto Láb. Iot, Conexão Esp32 ao servidor Thingsboard, UFPR - Autor: João André A. da Silva */
#ifndef WIFI_MANAGER
#define WIFI_MANAGER

#include "IoT_Manager.h"
//#include <ESP8266WiFi.h>  // -> Se for um ESP8266
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_eap_client.h"

#ifndef ENCRYPTED
  #define ENCRYPTED false
#endif

struct WifiCredentials {
   String ssid;
   String password;
   String login_username;
   String login_password;
};

class WifiManager {
   public:
      static void SetStationMode();
      static void SetAP_StationMode(const char* AP_Ssid, const char* AP_Password);
      static void LoginWifi(const char* _Wifi_Ssid="", const char* _Wifi_Password="", const char* _Wifi_Login_Username="", const char* _Wifi_Login_Password="");
      static bool IsWifiConnected();
      static void SetWifiCredentials(WifiCredentials credentials);
      #if ENCRYPTED
	      static WiFiClientSecure wifiClient;
      #else	
         static WiFiClient wifiClient;
      #endif
      static String Wifi_Ssid;
      static String Wifi_Password;
      static String Wifi_Login_Username;
      static String Wifi_Login_Password;
   private:
      WifiManager() {}
      static void HandleEAP_Wifi();
      static bool InitWiFi();
      static constexpr uint8_t WIFI_CONNECTION_TIMEOUT = 10;
      //#define WIFI_PORTAL_URL "http://www.msftconnecttest.com/redirect" //-> UFPR_SEM_FIO
};
#endif