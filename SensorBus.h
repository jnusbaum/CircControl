#ifndef SensorBus_h
#define SensorBus_h

// device addresses
typedef struct {
  char devname[64];
  DeviceAddress devaddr;
}
devinfo;

// temp sensor buss
typedef struct {
  int pin;
  OneWire *wire;
  DallasTemperature *bus;
  int numsensors;
  devinfo *sensors;
}
sensorbus;

#endif
