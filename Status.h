#ifndef Status_h
#define Status_h

#include <MQTT.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include "Defines.h"
#include "Constants.h"


void publishStatus(MQTTClient &client, NTPClient &timeClient, const char *statusstr)
{
  StaticJsonDocument<128> doc;
  doc["device"] = DEVICENAME;
  doc["status"] = statusstr;
  doc["timestamp"] = timeClient.getEpochTime();;
  char buffer[128];
  int n = serializeJson(doc, buffer);
  DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", statusTopic.c_str());
  client.publish(statusTopic.c_str(), buffer, n);
  DEBUG_PRINTLN("Published.");
}


void publishState(MQTTClient &client, NTPClient &timeClient, const char *relay, bool value)
{
  unsigned long etime = timeClient.getEpochTime();
  StaticJsonDocument<128> doc;
  doc["device"] = DEVICENAME;
  doc["relay"] = relay;
  doc["timestamp"] = etime;
  doc["value"] = value;
  // Generate the minified JSON and put it in buffer.
  String topic = relayTopic + relay;
  char buffer[128];
  int n = serializeJson(doc, buffer);
  DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", topic.c_str());
  client.publish(topic.c_str(), buffer, n);
}


void publishTemp(MQTTClient &client, NTPClient &timeClient, const char *sensor, float value)
{
  unsigned long etime = timeClient.getEpochTime();
  StaticJsonDocument<128> doc;
  doc["device"] = DEVICENAME;
  doc["sensor"] = sensor;
  doc["timestamp"] = etime;
  doc["value"] = value;
  // Generate the minified JSON and put it in buffer.
  String topic = tempTopic + sensor;
  char buffer[128];
  int n = serializeJson(doc, buffer);
  DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", topic.c_str());
  client.publish(topic.c_str(), buffer, n);
}

#endif
