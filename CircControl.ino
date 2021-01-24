/*
  Name:		HeatingTempMon.ino
  Created:	10/28/2019 2:46:05 PM
  Author:	nusbaum
*/


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
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
SensorBus *bus = nullptr;


void configReceived(String &topic, String &payload) {
  DEBUG_PRINTLN("incoming config: " + topic + " - " + payload);
  publishStatus(client, timeClient, statusTopic.c_str(), "CONFIG RECEIVED");
  DynamicJsonDocument doc(4096);

  // shutdown and free existing config here
  if (bus) delete bus;
    
  // create new config and setup
  DeserializationError error = deserializeJson(doc, payload.c_str());
  DEBUG_PRINTLN(String("deserialization result: ") + error.c_str());
  mqtt_client_id = doc["client_id"].as<const char*>();
  DEBUG_PRINTLN(String("client id: ") + mqtt_client_id);
  numbusses = doc["num_interfaces"];
  if (numbusses != 1)
  {
    // error, should only have one bus
    publishStatus(client, timeClient, statusTopic.c_str(), "BAD-CONFIG");
    return;
  }
  DEBUG_PRINTLN(String("number of interfaces: ") + numbusses);
  bus = new SensorBus();
  JsonArray interfaces = doc["interfaces"].as<JsonArray>();
  JsonObject jbus = interfaces[0];
  const int pin_number = jbus["pin_number"];
  DEBUG_PRINTLN(String("pin number: ") + pin_number);
  const int num_sensors = jbus["num_sensors"];
  if (num_sensors != 1)
  {
    // error, should only have one sensor
    publishStatus(client, timeClient, statusTopic.c_str(), "BAD-CONFIG");
    return;
  }
  DEBUG_PRINTLN(String("number of sensors: ") + num_sensors);
  bus->initialize(pin_number, num_sensors);
  JsonArray sensors = jbus["sensors"].as<JsonArray>();
  JsonObject jsensor = sensors[0];
  const char *devname = jsensor["name"].as<const char*>();
  DEBUG_PRINTLN(String("sensor name: ") + devname);
  const char *daddress = jsensor["address"].as<const char*>();
  DEBUG_PRINTLN(String("sensor address: ") + daddress);
  bus->initsensor(0, devname, daddress);
  bus->begin();

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


// pump control pin                                                             
int pinState = LOW;             // ledState used to set the LED                 
int pin = 12;

int period = PERIOD;
unsigned long previousMillis = 0;
unsigned long pump_start = 0;
bool pump_state = false;


void turn_off_pump()
{
  pump_state = false;
  DEBUG_PRINTLN("Pump off");
  // set the relay with the pinState of the variable:                           
  digitalWrite(pin, LOW);
  delay(10);
}


void turn_on_pump()
{
  pump_state = true;
  DEBUG_PRINTLN("Pump on");
  // set the relay with the pinState of the variable:                           
  digitalWrite(pin, HIGH);
  delay(10);
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

  // set the digital pin as output:                                             
  pinMode(pin, OUTPUT);
  turn_off_pump();


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

    if (configured) // if we aren't configured yet then skip processing
    {
      unsigned long etime = timeClient.getEpochTime();
      bus->requestTemps();
      DEBUG_PRINTF("time = %u\n", etime);
      delay(500);
      bus->processTemps(etime);

      // we know we only have one bus and one sensor
      float wheat_ret_temp = bus->getTempF(0);
      // wheat_ret_temp contains hot water return temp
      // if pump on
      if (pump_state)
      {
        //   has it been on for more than 10 minutes?
        if ((currentMillis - pump_start) > MIN_RUN_TIME)
        {
          // is return temp > high point?
          if (wheat_ret_temp > HIGH_POINT)
          {
            // turn off pump, set pump status
            turn_off_pump();
          }
        }
      }
      else
      {
        // is return temp < low point?
        if (wheat_ret_temp < LOW_POINT)
        {
          // turn pump on,
          turn_on_pump();
          // record time
          pump_start = currentMillis;
        }
      }

      if (pump_state) 
      {
        publishStatus(client, timeClient, statusTopic.c_str(), "PUMP-RUNNING");
      }
      else
      {
        publishStatus(client, timeClient, statusTopic.c_str(), "PUMP-STOPPED");
      }
    }
  }
}
