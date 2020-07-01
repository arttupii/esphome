#include "esphome.h"
#include <EEPROM.h>
#define PIN 5
#define LED 2

double imp_kWh = 10000;

unsigned long counter = 0;
void ICACHE_RAM_ATTR interruptHandler(){
    counter++;
    digitalWrite(LED,!digitalRead(LED));
}


class MyCustomSensor : public PollingComponent {
 public:
  Sensor *watt_sensor = new Sensor();
  Sensor *kWh_counter = new Sensor();
  Sensor *Wh_change = new Sensor();

  MyCustomSensor() : PollingComponent(15000) { }

  void setup() override {
    EEPROM.begin(512);
    pinMode(PIN, INPUT_PULLUP);
    pinMode(LED, OUTPUT);
    
      attachInterrupt(digitalPinToInterrupt(PIN), interruptHandler, RISING);
      //ESP_LOGE("main", "Hello World");

    EEPROM.get(0, counter);
  }

  void update() override {
    static unsigned long c = 0;
    kWh_counter->publish_state((double)counter/imp_kWh);
    
    unsigned long wat = (counter-c)/(get_update_interval()/360000.0);
    watt_sensor->publish_state(wat);
    
    Wh_change->publish_state((counter-c)/(imp_kWh/1000));
    c = counter;
    
    static unsigned long tm = millis();
    if(millis()-tm>(60*60*1000)) {
        EEPROM.put(0, counter);
        EEPROM.commit();
        tm = millis();
    }
  }
};
