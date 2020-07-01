#include "esphome.h"
#define LED 2

class Blink : public Component{
 public:
  Blink(int pin, unsigned long onTimeMs, unsigned long offTimeMs) {
    this->pin = pin;
    offMs = offTimeMs;
    onMs = onTimeMs;
    c = 0;
  }

  void setup() override {
    pinMode(pin, OUTPUT);
  }

  void loop() override {
    switch(c) {
        case 0:
            tm = millis(); 
            c++;
            digitalWrite(pin, HIGH);
            break;
        case 1:
            if(millis()-tm>onMs) {
                c++;
                digitalWrite(pin, LOW); 
                tm = millis();                 
            }
            break;
        case 2:
            if(millis()-tm>offMs) {
                c=0;
            }
            break;
        default: c=0; break;
    }
  }
  private:
    int pin;
    unsigned long offMs;
    unsigned long onMs;
    unsigned long tm;
    int c;
};
