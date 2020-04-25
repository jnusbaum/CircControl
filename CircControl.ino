/*
  Name:		CircControl.ino
  Created:	10/28/2019 2:46:05 PM
  Author:	nusbaum
*/


#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>

#define DEVICENAME "CIRC"
#define MQTTHOST "192.168.0.134"

#define HIGH_POINT 90
#define LOW_POINT 75

// 1 minutes
#define MIN_RUN_TIME 6000
// temp check interval
#define PERIOD 10000

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)     Serial.print (x)
#define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#endif


WiFiClient net;
MQTTClient client(4096);


String mqtt_client_id;
String baseTopic = "sorrelhills/";
String tempTopic = baseTopic + "temperature/";
String statusTopic = baseTopic + "device/status/" + DEVICENAME;
String configRequestTopic = baseTopic + "device/config-request/" + DEVICENAME;
String configReceiveTopic = baseTopic + "device/config/" + DEVICENAME;
bool configured = false;


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "us.pool.ntp.org");


void publishStatus(const char *statusstr)
{
  StaticJsonDocument<128> doc;
  doc["status"] = statusstr;
  doc["timestamp"] = timeClient.getEpochTime();;
  char buffer[128];
  int n = serializeJson(doc, buffer);
  DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", statusTopic.c_str());
  client.publish(statusTopic.c_str(), buffer, n);
  DEBUG_PRINTF("Published %s\n", statusstr);
}


#ifdef DEBUG

// function to print a device address
// only called when DEBUG
void printAddress(DeviceAddress &deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


// function to print the temperature for a device
// only called when DEBUG
void printTemperature(DeviceAddress &d, float tempC, float tempF)
{
  Serial.print("Temp for Address: ");
  printAddress(d);
  Serial.println();

  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(tempF);
}

#endif



class TempSensor {
  public:
    char devname[64];
    DeviceAddress devaddr;
    float temp;

    void initialize(const char *i_devname, const char *i_daddress) {
      strcpy(devname, i_devname);
      devaddr[0] = (uint8_t)strtoul(i_daddress, nullptr, 16);
      devaddr[1] = (uint8_t)strtoul(i_daddress+6, nullptr, 16);
      devaddr[2] = (uint8_t)strtoul(i_daddress+12, nullptr, 16);
      devaddr[3] = (uint8_t)strtoul(i_daddress+18, nullptr, 16);
      devaddr[4] = (uint8_t)strtoul(i_daddress+24, nullptr, 16);
      devaddr[5] = (uint8_t)strtoul(i_daddress+30, nullptr, 16);
      devaddr[6] = (uint8_t)strtoul(i_daddress+36, nullptr, 16);
      devaddr[7] = (uint8_t)strtoul(i_daddress+42, nullptr, 16);
      temp = 0.0;
    }
};


class SensorBus {
  public:
    int pin;
    OneWire *wire;
    DallasTemperature *bus;
    int numsensors;
    TempSensor *sensors;

    SensorBus() {
      pin = 0;
      wire = nullptr;
      bus = nullptr;
      numsensors = 0;
      sensors = nullptr;
    }

    void initialize(const int i_pin, const int i_numsensors) {
      pin = i_pin;
      wire = new OneWire(pin);
      bus = new DallasTemperature(wire);
      numsensors = i_numsensors;
      sensors = new TempSensor[numsensors];
    }

    void initsensor(const int i_sensoridx, const char *i_devname, const char *i_daddress) {
      sensors[i_sensoridx].initialize(i_devname, i_daddress);
    }

    void begin() {
      bus->begin();
      for (int y = 0; y < numsensors; ++y)
      {
        #ifdef DEBUG
        Serial.print("Device Address: ");
        printAddress(sensors[y].devaddr);
        Serial.println();
        #endif

        // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
        bus->setResolution(sensors[y].devaddr, 9);

        #ifdef DEBUG
        Serial.print("Device Resolution: ");
        Serial.print(bus->getResolution(sensors[y].devaddr), DEC);
        Serial.println();
        #endif
      }
    }

    void requestTemps() {
      bus->requestTemperatures();
    }

    void processTemps(unsigned long etime) {
      for (int y = 0; y < numsensors; ++y)
      {
        float tempC = bus->getTempC(sensors[y].devaddr);
        float tempF = bus->toFahrenheit(tempC);
        sensors[y].temp = tempF;
      #ifdef DEBUG
        printTemperature(sensors[y].devaddr, tempC, tempF); // Use a simple function to print out the data
      #endif

        StaticJsonDocument<128> doc;
        doc["sensor"] = sensors[y].devname;
        doc["timestamp"] = etime;
        doc["value"] = tempF;
        // Generate the minified JSON and put it in buffer.
        String topic = tempTopic + sensors[y].devname;
        char buffer[128];
        int n = serializeJson(doc, buffer);
        DEBUG_PRINTF("[MQTT] PUBLISHing to %s\n", topic.c_str());
        client.publish(topic.c_str(), buffer, n);
      }
    }

    ~SensorBus() {
      delete [] sensors;
      delete bus;
      delete wire;
    }
};

// temperature sensor bus
int numbusses = 0;
SensorBus *bus = nullptr;


void configReceived(String &topic, String &payload) {
  DEBUG_PRINTLN("incoming config: " + topic + " - " + payload);
  publishStatus("CONFIG RECEIVED");
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
    publishStatus("BAD-CONFIG");
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
    publishStatus("BAD-CONFIG");
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
  publishStatus("CONFIGURED");
}


void req_configure() {
  DEBUG_PRINTLN("configuring...");
  configured = false;
  client.publish(configRequestTopic.c_str());
  DEBUG_PRINTLN("config requested");
  publishStatus("CONFIG REQUESTED");
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
  while (!client.connect(mqtt_client_id.c_str())) 
  {
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


void setup() 
{
  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) 
  {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

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

  publishStatus("STARTING");

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
      publishStatus("RECONNECTED");
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
      float wheat_ret_temp = bus->sensors[0].temp;
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
        publishStatus("PUMP-RUNNING");
      }
      else
      {
        publishStatus("PUMP-STOPPED");
      }
    }
  }
}
