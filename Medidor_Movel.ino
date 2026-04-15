/* Projeto Láb. Iot, Conexão Esp32 ao servidor Thingsboard, UFPR - Autor: João André A. da Silva */
#include "WifiManager.h"
#include "IoT_Manager.h"
#include "WifiCredentials.h"

#include <PicoMQTT.h>
//#include <ArduinoJson.h>
#include <DNSServer.h>
#include <time.h>
#include "DHT.h"

#define DHTPIN 4   
DHT dht(DHTPIN, DHT11);

#define DNS_PORT 53
DNSServer dnsServer;

#define THINGSBOARD_SERVER "tb.eletrica.ufpr.br"
#define THINGSBOARD_TOKEN "8ob8VfviVIWoNUwFK1MY"
#define THINGSBOARD_PORT 443
#define MEASUREMENT_INTERVAL 60 // segundos

#define AP_SSID "ESP32_Broker" 
#define AP_PASSWORD "brokertb"

PicoMQTT::Server mqtt;

std::vector<const char*> KRON_JSON_KEYS = {
	"U0", "U1", 
    "U2", "U3",
    "I0", "I1", 
    "I2", "I3",
    "P0", "P1", 
    "P2", "P3",
    "Q0", "Q1", 
    "Q2", "Q3",
    "EA", "ER",
};

std::vector<const char*> SENSOR_ATRIBUTES = {
	"humidity", "temperature"
};

IoTMetaData mdata_medidor;
IoTMetaData mdata_sensor;

IoT_Manager* device_HTTPS = nullptr;

int8_t devicesConnected = -1;
bool SendDataMedidor = false;

unsigned StringDateToTimestamp(const char* time_str) {
    int year, month, day, hour, minute, second;

    if (sscanf(time_str, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) != 6) {
        Serial.printf("Erro - JSON: Tempo não formatado corretamente.");
        return 0; 
    }

    struct tm t;
    t.tm_year = year - 1900; 
    t.tm_mon = month - 1;    
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;

    time_t _timestamp = mktime(&t);
    return (unsigned long)_timestamp;
}

std::vector<float> GetHumiTemp() {
    float humidity = dht.readHumidity();
    float temp = dht.readTemperature();

    if (isnan(humidity) || isnan(temp)) {
        Serial.println(F("Falha ao ler do sensor DHT11."));
        humidity = temp = NAN;
    }

    if (temp==0 && humidity==0) {
        Serial.println(F("Sensor DHT11 Inicializando ainda, valores nulos."));
        humidity = temp = NAN;
    }

    Serial.print("Umidade: ");
    Serial.print(humidity);
    Serial.print("%  |  ");
    Serial.print("Temperatura: ");
    Serial.print(temp);
    Serial.println("°C");

    std::vector<float> data = {humidity, temp};
    return data;
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Tempo para iniciar o Serial

	WifiManager::SetAP_StationMode(AP_SSID, AP_PASSWORD);
	WifiManager::SetWifiCredentials(eduroam);
	WifiManager::LoginWifi();

	device_HTTPS = new IoT_Manager(THINGSBOARD_SERVER, THINGSBOARD_TOKEN, THINGSBOARD_PORT, MEASUREMENT_INTERVAL);
	bool startSecondZero = true;
	device_HTTPS->Initialize(startSecondZero);

    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); 
    Serial.println("Servidor DNS ficticio iniciado.");

    dht.begin();

    mqtt.subscribe("data", [](const char* topic, const char* payload) 
    {
        Serial.println("MQTT Recebido do Medidor, topico: " + String(topic));

        if (SendDataMedidor) {
            Serial.println("MQTT Recebido mas dados anteriores ainda guardados.");
            return;
        }

        const uint8_t Json_Size = JSON_OBJECT_SIZE(1) + JSON_ARRAY_SIZE(KRON_JSON_KEYS.size());
	    DynamicJsonDocument doc(Json_Size);

        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.print("Erro no JSON: ");
            Serial.println(error.c_str());
            return;
        }

        Serial.println("Dado recebido: " + String(payload));
        if (!doc[0].containsKey("time") || !doc[0].containsKey("data")) 
            return;

        const char* json_time = doc[0]["time"]; 
        mdata_medidor.timestamp = StringDateToTimestamp(json_time);
    
        Serial.print("Data dos dados do Medidor: ");
        Serial.println(json_time);
        Serial.print("Em timestamp: ");
        Serial.println(mdata_medidor.timestamp);

        JsonObject json_data = doc[0]["data"];
        Serial.println("--- Dados Recebidos ---");

        std::vector<float> data;
        for(int i=0; i<KRON_JSON_KEYS.size(); ++i)
            if (json_data.containsKey(KRON_JSON_KEYS[i])) {
                float element = json_data[KRON_JSON_KEYS[i]];
                Serial.println("  - " + String(KRON_JSON_KEYS[i]) + ": " + String(element));

                data.push_back(element);
            } else {
                Serial.print("Erro no JSON: Atributo não encontrado no JSON - " + String(KRON_JSON_KEYS[i]));
                data.clear();
                return;
            }
        mdata_medidor.Append_Data(data);
        SendDataMedidor = true;
        Serial.println("-----------------------");
    });
    mqtt.begin();
}

void loop() {
    dnsServer.processNextRequest();
    mqtt.loop();

    if (SendDataMedidor) {
        bool httpsSent = device_HTTPS->SendData_HTTPS(mdata_medidor, KRON_JSON_KEYS);
        if (httpsSent)
            mdata_medidor.Clear_Meta_Data();
        SendDataMedidor = false;
	}

    if (devicesConnected != WiFi.softAPgetStationNum()) {
        devicesConnected = WiFi.softAPgetStationNum();
        Serial.printf("Dispositivos conectados no WiFi: %d\n", devicesConnected);
    }

    if (device_HTTPS->NextMeasurement()) {
        mdata_sensor.Append_Data(GetHumiTemp());
        bool httpsSent = device_HTTPS->SendData_HTTPS(mdata_sensor, SENSOR_ATRIBUTES);
        if (httpsSent)
            mdata_sensor.Clear_Meta_Data();
    }
}