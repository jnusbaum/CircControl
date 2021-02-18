#ifndef Relay_h
#define Relay_h


class Relay {
    public:
    
    char devname[64];
    int pin;
    boolean energized;

    Relay() {
      pin = 0;
      energized = false;
    }
    
    void initialize(const char *i_devname, int ipin) {
      pin = ipin;
      pinMode(pin, OUTPUT);
      strcpy(devname, i_devname);
    }
    
    void deEnergize() {
      energized = false;                         
      digitalWrite(pin, LOW);
      delay(10);
    }

    void energize() {
      energized = true;                           
      digitalWrite(pin, HIGH);
      delay(10);
    }

    void begin() {
      deEnergize();
    }
};

#endif
