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
#include <MQTTClient.h>

#include "Defines.h"
#include "Status.h"
#include "Constants.h"
#include "DeviceAddresses.h"
#include "TempSensor.h"
#include "Relay.h"

WiFiClient net;
bool configured = false;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "us.pool.ntp.org");

MQTTClient client(4096);
String mqtt_client_id;

int numbusses = 0;
SensorBus *bus = nullptr;
int numrelays = 0;
Relay *relay = nullptr;

// previous temp value used for cleaning
float prevTempValue = 0;


void configReceived(String &topic, String &payload) {
  DEBUG_PRINTLN("incoming config: " + topic + " - " + payload);
  publishStatus(client, timeClient, "CONFIG RECEIVED");
  DynamicJsonDocument doc(4096);

  // shutdown and free existing config here
  if (bus) delete bus;
  if (relay) delete relay;
    
  // create new config and setup
  DeserializationError error = deserializeJson(doc, payload.c_str());
  DEBUG_PRINTLN(String("deserialization result: ") + error.c_str());
  mqtt_client_id = doc["client_id"].as<const char*>();
  DEBUG_PRINTLN(String("client id: ") + mqtt_client_id);
  numbusses = doc["num_interfaces"];
  if (numbusses != 1)
  {
    // error, should only have one bus
    publishStatus(client, timeClient, "BAD-CONFIG");
    return;
  }
  DEBUG_PRINTLN(String("number of interfaces: ") + numbusses);
  bus = new SensorBus();
  JsonArray interfaces = doc["interfaces"].as<JsonArray>();
  JsonObject jbus = interfaces[0];
  int pin_number = jbus["pin_number"];
  DEBUG_PRINTLN(String("pin number: ") + pin_number);
  const int num_sensors = jbus["num_devices"];
  if (num_sensors != 1)
  {
    // error, should only have one sensor
    publishStatus(client, timeClient, "BAD-CONFIG");
    return;
  }
  DEBUG_PRINTLN(String("number of sensors: ") + num_sensors);
  bus->initialize(pin_number, num_sensors);
  DEBUG_PRINTLN(String("bus ") + pin_number + String(" initialized"));
  JsonArray devices = jbus["devices"].as<JsonArray>();
  JsonObject jsensor = devices[0];
  const char *devname = jsensor["name"].as<const char*>();
  DEBUG_PRINTLN(String("sensor name: ") + devname);
  const char *daddress = jsensor["address"].as<const char*>();
  DEBUG_PRINTLN(String("sensor address: ") + daddress);
  bus->initsensor(0, devname, daddress);
  bus->begin();

  numrelays = doc["num_relays"];
  if (numrelays != 1)
  {
    // error, should only have one relay
    publishStatus(client, timeClient, "BAD-CONFIG");
    return;
  }
  DEBUG_PRINTLN(String("number of relays: ") + numrelays);
  relay = new Relay();
  JsonArray relays = doc["relays"].as<JsonArray>();
  JsonObject jrelay = relays[0];
  const char *rdevname = jrelay["name"].as<const char*>();
  DEBUG_PRINTLN(String("relay name: ") + rdevname);
  pin_number = jrelay["pin_number"];
  DEBUG_PRINTLN(String("pin number: ") + pin_number);
  relay->initialize(rdevname, pin_number);
  relay->begin();

  configured = true;
  publishStatus(client, timeClient, "CONFIGURED");
  turn_off_pump();
  
  while (prevTempValue < MIN_WATER_TEMP || prevTempValue > MAX_WATER_TEMP) {
    DEBUG_PRINTLN("Stabilizing temp sensor....");
    bus->requestTemps();
    delay(500);
    bus->processTemps();
    prevTempValue = bus->getTempF(0);
    DEBUG_PRINTF("got %f\n", prevTempValue);
  }
  DEBUG_PRINTF("starting temp %f\n", prevTempValue);
}


void req_configure() {
  DEBUG_PRINTLN("configuring...");
  configured = false;
  client.publish(configRequestTopic.c_str());
  DEBUG_PRINTLN("config requested");
  publishStatus(client, timeClient, "CONFIG REQUESTED");
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


unsigned long period = PERIOD;
unsigned long previousMillis = 0;
unsigned long pump_start = 0;

void turn_off_pump()
{
  DEBUG_PRINTLN("turning Pump off");
  relay->deEnergize();
  DEBUG_PRINTLN("Pump off");
}


void turn_on_pump()
{
  DEBUG_PRINTLN(String("Turning pump on - pin number: ") + relay->pin);                     
  relay->energize();
  DEBUG_PRINTLN("Pump on");
}


void setup() {
#ifdef DEBUG
  Serial.begin(9600);

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

  publishStatus(client, timeClient, "STARTING");

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
      publishStatus(client, timeClient, "RECONNECTED");
    }

    timeClient.update();

    previousMillis = currentMillis;

    if (configured) // if we aren't configured yet then skip processing
    {
      unsigned long etime = timeClient.getEpochTime();
      bus->requestTemps();
      DEBUG_PRINTF("time = %lu\n", etime);
      delay(500);
      bus->processTemps();

      // we know we only have one bus and one sensor
      float wheat_ret_temp = bus->getTempF(0);
      DEBUG_PRINTF("sampled temp = %f\n", wheat_ret_temp);
      // clean data
      if (wheat_ret_temp < MIN_WATER_TEMP || wheat_ret_temp > MAX_WATER_TEMP || abs(wheat_ret_temp - prevTempValue) > MAX_TEMP_MOVE) {
        DEBUG_PRINTF("got bad value %f\n", wheat_ret_temp);
        wheat_ret_temp = prevTempValue;
        DEBUG_PRINTF("using value %f\n", wheat_ret_temp);
      }
      else {
        prevTempValue = wheat_ret_temp;
      }
      
      publishTemp(client, timeClient, bus->deviceName(0), wheat_ret_temp);
      DEBUG_PRINTF("temp = %f\n", wheat_ret_temp);
      // wheat_ret_temp contains hot water return temp
      // if pump on
      if (relay->energized)
      {
        //   has it been on for more than 10 minutes?
        if ((currentMillis - pump_start) > MIN_RUN_TIME)
        {
          // is return temp > high point?
          if (wheat_ret_temp > HIGH_POINT)
          {
            // turn off pump, set pump status
            turn_off_pump();
            publishStatus(client, timeClient, "PUMP-STOPPED");
          }
        }
        publishState(client, timeClient, relay->devname, true);
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
          publishStatus(client, timeClient, "PUMP-RUNNING");
        }
        publishState(client, timeClient, relay->devname, false);
      }
    }
  }
}
