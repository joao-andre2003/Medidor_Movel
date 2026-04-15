/* Projeto Láb. Iot, Conexão Esp32 ao servidor Thingsboard, UFPR - Autor: João André A. da Silva */
#include "IoT_Manager.h"

String Input(String message) {
	Serial.println(message);
	while (!Serial.available());
	return Serial.readStringUntil('\n');
}

void IoTMetaData::Clear_Meta_Data() {
    data_list.clear();
    timestamp = 0;
}

void IoTMetaData::Append_Data(std::vector<float> data) {
	data_list.push_back(data);
}

// MQTT Constructor
IoT_Manager::IoT_Manager(const char* Server, const char* Token, const uint16_t Port, const uint16_t MeasurementInterval, const std::array<RPC_Callback, MAX_RPC_CALLBACKS>* callbacks)
	: ThingsBoard_Server(Server),
     ThingsBoard_Token(Token),
	  ThingsBoard_Port(Port),
	  Measurement_Interval(MeasurementInterval),
	  Callbacks(callbacks),
	  commtype(MQTT)
{
	mqttClient = new Arduino_MQTT_Client(WifiManager::wifiClient);
	tb = new ThingsBoard(*mqttClient, MAX_MESSAGE_SEND_RECEIVE_SIZE, MAX_MESSAGE_SEND_RECEIVE_SIZE, Default_Max_Stack_Size, apis);
	Serial.print("Configuracao MQTT feita.");
}

// HTTPS Constructor
IoT_Manager::IoT_Manager(const char* Server, const char* Token, const uint16_t Port, const uint16_t MeasurementInterval)
	: ThingsBoard_Server(Server),
     ThingsBoard_Token(Token),
	  ThingsBoard_Port(Port),
	  Measurement_Interval(MeasurementInterval),
	  commtype(HTTPS)
{ 
	Serial.print("Configuracao HTTPS feita.");
}

// Conectar ao ThingsBoard
bool IoT_Manager::Init_MQTT(bool SubscribeToRPC) {
  	Serial.println("Conectando ao ThingsBoard - " + String(ThingsBoard_Server));
  	if (!tb->connect(ThingsBoard_Server, ThingsBoard_Token, ThingsBoard_Port)) {
		Serial.printf("ERRO - Conexão ThingsBoard - %d\n", mqttClient->connected());
    	return false;
  	}
  	if (SubscribeToRPC && Callbacks->size() > 0) { // Lidar com inscrições RPC
    	if (!rpc.RPC_Subscribe(Callbacks->cbegin(), Callbacks->cend())) {
      	Serial.println("ERRO - Falha na inscrição do método RPC.");
      	return false;
   	 }
  	  Serial.println("Inscrição RPC realizada.");
	}
  	Serial.println("< Conectado ao ThingsBoard >");
  	return true;
}

void IoT_Manager::Initialize(bool startSecondZero) 
{
	if (commtype == MQTT)
		Init_MQTT(true);

	Serial.print("Estabelecendo conexão com NTP.");
	configTime(GMT, DAYLIGHT_OFFSET, NTP_SERVER);
	while (measurement_timestamp < 10000) { // 10000 porque a função time funciona de um jeito estranho antes da inicialização completa, ele recebe valores muito pequenos até pegar de fato o timestamp
		time(&measurement_timestamp);
		delay(500);
		Serial.print(".");
	}

	if (startSecondZero == false)
		return;

	int secondsToNextMinute = 60.0 - (measurement_timestamp % 60);
   nextCycleMillis = millis() + (secondsToNextMinute * 1000);
	Serial.println("\nDevice inicializado.");
}

bool IoT_Manager::IsThingsBoardConnected() {
   if (!tb->connected()) { // Reconectar ThingsBoard
		Serial.println("ThingsBoard desconectado. Tentando reconectar.");
		if (!Init_MQTT())
			return false;	
	}
   return true;
}

bool IoT_Manager::NextMeasurement() {
		currentMillis = millis();
	if (currentMillis < nextCycleMillis)
		return false;
	
	if (WifiManager::IsWifiConnected()) {
		time_t now;
		time(&now);
		unsigned long secondsToNextCycle = Measurement_Interval - (now % Measurement_Interval);
		nextCycleMillis = currentMillis + (secondsToNextCycle * 1000);

		measurement_timestamp = now;
	} else {
		unsigned long timeToNextCycle = Measurement_Interval - (measurement_timestamp % Measurement_Interval);
		nextCycleMillis = currentMillis + timeToNextCycle * 1000;
	}
	return true;
}

bool IoT_Manager::IsReady2Send() {
	if (ESP.getMaxAllocHeap() < MEMORY_THRESHOLD) {
		Serial.println("ERRO: Armazenamento de memoria no limite. Esperando conexao com rede para enviar dados armazenados. Novos dados serao perdidos.");
		return false;
	}

	if (WifiManager::IsWifiConnected() == false) {
		Serial.println("ERRO no envio da telemetria: Wifi não conectado. Dado armazenado na RAM.");
		return false;
	}
	return true;
}

String IoT_Manager::GetDataJson2String(std::vector<float>& data, const std::vector<const char*>& AtributesList, time_t& timestamp) {
	const uint8_t AtributesList_Size = AtributesList.size();
	const uint8_t Json_Size = JSON_OBJECT_SIZE(1) + JSON_ARRAY_SIZE(AtributesList_Size);
	DynamicJsonDocument Json_Data(Json_Size);
	Json_Data["ts"] = timestamp * 1000;

	JsonObject values = Json_Data.createNestedObject("values");
	for(int j=0; j<AtributesList_Size; ++j) 
		values[(AtributesList)[j]] = data[j];

	String String_Data;
	serializeJson(Json_Data, String_Data);
	return String_Data;
}

bool IoT_Manager::SendData_MQTT(IoTMetaData& metaData, const std::vector<const char*>& AtributesList) 
{
	metaData.timestamp = (metaData.timestamp==0)? measurement_timestamp : metaData.timestamp;
	if (!IsReady2Send())
		return false;

	if (!IsThingsBoardConnected()) {
		Serial.println("ERRO no envio da telemetria: TB incalcançavel. Dado armazenado na RAM.");
		return false;
	}

	time_t aux_timestamp = metaData.timestamp;
	for(int i=0; i<metaData.data_list.size(); ++i) {
		String mqttRequestData = GetDataJson2String(metaData.data_list[i], AtributesList, aux_timestamp);

		if (tb->sendTelemetryString(mqttRequestData.c_str())) {
			Serial.print("Envio MQTT de telemetria: ");
			Serial.println(mqttRequestData);
		} else {
			Serial.println("ERRO no envio da telemetria ao TB - MQTT");
			return false;
		}
		aux_timestamp += Measurement_Interval; 
	}
	tb->loop();
	return true;
}

bool IoT_Manager::SendData_HTTPS(IoTMetaData& metaData, const std::vector<const char*>& AtributesList) 
{
	metaData.timestamp = (metaData.timestamp==0)? measurement_timestamp : metaData.timestamp;
	if (!IsReady2Send())
		return false;

	int httpResponseCode;
	String serverUrl = "https://"+String(ThingsBoard_Server)+"/api/v1/"+String(ThingsBoard_Token)+"/telemetry";
	HTTPClient http;
	http.begin(serverUrl);
	http.addHeader("Content-Type", "application/json");

	time_t aux_timestamp = metaData.timestamp;
	for(int i=0; i<metaData.data_list.size(); ++i) {
		String httpRequestData = GetDataJson2String(metaData.data_list[i], AtributesList, aux_timestamp);
		httpResponseCode = http.POST(httpRequestData);

		if (httpResponseCode >= 200 && httpResponseCode < 300) {
			Serial.print("Envio HTTPS de telemetria:");
			Serial.println(httpRequestData);
		} else {
			Serial.print("Erro no envio HTTPS: ");
			Serial.println(httpResponseCode);
			String response = http.getString();
			Serial.println("Resposta do servidor: " + response);
			return false;
		}
		aux_timestamp += Measurement_Interval; 
	}
	http.end();
	return true;
}