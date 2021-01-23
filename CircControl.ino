/*
  Name:		HeatingTempMon.ino
  Created:	10/28/2019 2:46:05 PM
  Author:	nusbaum
*/


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <MQTT.h>

#include "Defines.h"
#include "Status.h"
#include "Constants.h"
#include "DeviceAddresses.h"
#include "TempSensor.h"

WiFiClient net;
bool configured = false;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "us.pool.ntp.org");

MQTTClient client(4096);
String mqtt_client_id;


int numbusses = 0;
SensorBus *busses = nullptr;


void configReceived(String &topic, String &payload) {
  DEBUG_PRINTLN("incoming config: " + topic + " - " + payload);
  publishStatus(client, timeClient, statusTopic.c_str(), "CONFIG RECEIVED");
  DynamicJsonDocument doc(4096);

  // shutdown and free existing config here
  if (busses) delete [] busses;
    
  // create new config and setup
  DeserializationError error = deserializeJson(doc, payload.c_str());
  DEBUG_PRINTLN(String("deserialization result: ") + error.c_str());
  mqtt_client_id = doc["client_id"].as<const char*>();
  DEBUG_PRINTLN(String("client id: ") + mqtt_client_id);
  numbusses = doc["num_interfaces"];
  DEBUG_PRINTLN(String("number of interfaces: ") + numbusses);
  busses = new SensorBus[numbusses];
  JsonArray interfaces = doc["interfaces"].as<JsonArray>();
  for (int x = 0; x < numbusses; ++x) {
    JsonObject jbus = interfaces[x];
    const int pin_number = jbus["pin_number"];
    DEBUG_PRINTLN(String("pin number: ") + pin_number);
    const int num_sensors = jbus["num_sensors"];
    DEBUG_PRINTLN(String("number of sensors: ") + num_sensors);
    busses[x].initialize(pin_number, num_sensors);
    JsonArray sensors = jbus["sensors"].as<JsonArray>();
    for (int y = 0; y < num_sensors; ++y) {
      JsonObject jsensor = sensors[y];
      const char *devname = jsensor["name"].as<const char*>();
      DEBUG_PRINTLN(String("sensor name: ") + devname);
      const char *daddress = jsensor["address"].as<const char*>();
      DEBUG_PRINTLN(String("sensor address: ") + daddress);
      busses[x].initsensor(y, devname, daddress);
    }
    busses[x].begin();
  }

  configured = true;
  publishStatus(client, timeClient, statusTopic.c_str(), "CONFIGURED");
}


void req_configure() {
  DEBUG_PRINTLN("configuring...");
  configured = false;
  client.publish(configRequestTopic.c_str());
  DEBUG_PRINTLN("config requested");
  publishStatus(client, timeClient, statusTopic.c_str(), "CONFIG REQUESTED");
}


void connect() {
  DEBUG_PRINT("Wait for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT(".");
    delay(500);
  }
  
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINTLN("IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());
  DEBUG_PRINTLN("MAC address: ");
  DEBUG_PRINTLN(WiFi.macAddress());

  DEBUG_PRINT("\nconnecting to MQTT...");
  while (!client.connect(mqtt_client_id.c_str())) {
    DEBUG_PRINT(".");
    delay(500);
  }
  DEBUG_PRINTLN("\nconnected!");
  
  client.subscribe(configReceiveTopic.c_str());
  DEBUG_PRINTLN(String("subscribed to ") + configReceiveTopic);
}


void setup() {
#ifdef DEBUG
  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin("nusbaum-24g", "we live in Park City now");
  client.begin(MQTTHOST, net);
  client.onMessage(configReceived);

  connect();

  timeClient.begin();
  timeClient.forceUpdate();

  publishStatus(client, timeClient, statusTopic.c_str(), "STARTING");

  req_configure();
}


int period = 10000;
unsigned long previousMillis = 0;

void loop() 
{
  client.loop();
  delay(10);
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > period)
  {
    if (!client.connected()) 
    {
      connect();
      publishStatus(client, timeClient, statusTopic.c_str(), "RECONNECTED");
    }

    timeClient.update();

    previousMillis = currentMillis;

    if (configured) {
      unsigned long etime = timeClient.getEpochTime();
      for (int x = 0; x < numbusses; ++x)
      {
        busses[x].requestTemps();
      }
      DEBUG_PRINTF("time = %u\n", etime);
      delay(500);
      for (int x = 0; x < numbusses; ++x)
      {
        busses[x].processTemps(etime);
        for (int y = 0; y < busses[x].numSensors(); ++y)
        {
          StaticJsonDocument<128> doc;
          doc["sensor"] = busses[x].deviceName(y);
          doc["timestamp"] = etime;
          doc["value"] = busses[x].getTempF(y);
          // Generate the minified JSON and put it in buffer.
          String topic = tempTopic + busses[x].deviceName(y);
          char buffer[128];
          int n = serializeJson(doc, buffer);
          DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", topic.c_str());
          client.publish(topic.c_str(), buffer, n);
        }
      }
      publishStatus(client, timeClient, statusTopic.c_str(), "RUNNING");
    }
  }
}
