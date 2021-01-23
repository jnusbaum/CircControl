#ifndef TempSensor_h
#define TempSensor_h

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>




class TempSensor {
    char devname[64];
    DeviceAddress devaddr;

  public:
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
    }

    const char *device_name() {
      return devname;
    }

    DeviceAddress &device_address() {
      return devaddr;
    }
};


class SensorBus {
    int pin;
    OneWire *wire;
    DallasTemperature *bus;
    int numsensors;
    TempSensor *sensors;

  public:
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

    int numSensors() {
      return numsensors;
    }

    void begin() {
      bus->begin();
      for (int y = 0; y < numsensors; ++y)
      {
        #ifdef DEBUG
        Serial.print("Device Address: ");
        printAddress(sensors[y].device_address());
        Serial.println();
        #endif

        // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
        bus->setResolution(sensors[y].device_address(), 9);

        #ifdef DEBUG
        Serial.print("Device Resolution: ");
        Serial.print(bus->getResolution(sensors[y].device_address()), DEC);
        Serial.println();
        #endif
      }
    }

    void requestTemps() {
      bus->requestTemperatures();
    }

    float getTempC(int y) {
      float tempC = bus->getTempC(sensors[y].device_address());
      return tempC;
    }

    float getTempF(int y) {
      float tempC = getTempC(y);
      float tempF = bus->toFahrenheit(tempC);
      return tempF;
    }


    String deviceName(int y) {
      return sensors[y].device_name();
    }

    

    void processTemps(unsigned long etime) {
      for (int y = 0; y < numsensors; ++y)
      {
        float tempC = getTempC(y);
        float tempF = getTempF(y);
      #ifdef DEBUG
        printTemperature(sensors[y].device_address(), tempC, tempF); // Use a simple function to print out the data
      #endif
      }
    }

    ~SensorBus() {
      delete [] sensors;
      delete bus;
      delete wire;
    }

};

#endif
