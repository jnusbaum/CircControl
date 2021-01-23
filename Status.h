#ifndef Status_h
#define Status_h

#include <MQTT.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include "Defines.h"


void publishStatus(MQTTClient &client, NTPClient &timeClient, const char *statusTopic, const char *statusstr)
{
  StaticJsonDocument<128> doc;
  doc["status"] = statusstr;
  doc["timestamp"] = timeClient.getEpochTime();;
  char buffer[128];
  int n = serializeJson(doc, buffer);
  DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", statusTopic);
  client.publish(statusTopic, buffer, n);
  DEBUG_PRINTLN("Published.");
}

#endif
