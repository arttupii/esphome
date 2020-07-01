#include "esphome.h"
#define LED 2

#define RX_PIN 5 //D1
#define TX_PIN 4 //D2
#define KAMBAUD 1200                                                                                                            // The meter's IR runs only at 1200/2400 BAUD
#define KAMTIMEOUT 250                                                                                                         // Kamstrup timeout after transmit
#define NUMREGS 7                                                                                                               // Number of registers above

long crc_1021(byte const *inmsg, unsigned int len);
float kamDecode(unsigned short const kreg, byte const *msg);
unsigned short kamReceive(byte recvmsg[]);
void kamSend(byte const *msg, int msgsize);
float kamReadReg(unsigned short kreg);
int16_t const kregnums[] = {
  0x003C, 0x0050, 0x0056, 0x0057, 0x0059, 0x004a, 0x0044
};      
typedef struct {
  //********************************************Kamstrup Stuff Start********************************************

  float Energy;
  float CurrentPower;
  float TemperatureT1;
  float TemperatureT2;
  float TemperatureDiff;
  float Flow;
  float Volumen1;

  //********************************************Kamstrup Stuff End ********************************************

}
Payload;
Payload emontx;
class SerialS {
  public:
    SerialS(int rx, int tx, bool inverted) {
      pin_tx = tx;
      pin_rx = rx;
      ready = false;


      pinMode(pin_tx, OUTPUT);
      pinMode(pin_rx, INPUT_PULLUP);
 
      digWrite(HIGH);
    }
    ~SerialS() {};
    void begin(unsigned long baud) {
      us = 833;
      //Serial.println(us);
    }


    int available() {
      if (!digRead()) {
        unsigned long u = micros();

        //start bit
        u += us;
        while (micros() <= u) ESP.wdtFeed();;
        //delayMicroseconds(us);

        for (int i = 0; i < 8; i++) {
          u += 400;
          while (micros() < u) ESP.wdtFeed();;
          //delayMicroseconds(us/2);
          if (digRead()) buf = (buf >> 1) | 0x80;
          else buf = (buf >> 1) & 0x7f;

          u += 433;
          while (micros() < u) ESP.wdtFeed();
          //delayMicroseconds(us/2);
        }
            
          u += 233;
          while (micros() < u) ESP.wdtFeed();;
      
        ready = true;
        return 1;
      }
      else {
        return 0;
      }
    }
    
    void flush() {
      ready = false;
    }
    
    void write(int d) {
      unsigned long u = micros() + us;
     
      //start bit;
      digWrite(LOW);
      while (micros() <= u) ESP.wdtFeed();;
      //delayMicroseconds(us);
      for (int i = 0; i < 8; i++) {
        if (d & 1) digWrite(HIGH);
        else digWrite(LOW);
        d = d >> 1;
        u += us;
        //delayMicroseconds(us);
        while (micros() <= u) ESP.wdtFeed();;
      }
      
      digWrite(HIGH);
      //two stop bits
      for (int i = 0; i < 3; i++) {
        u += us;
        while (micros() <= u) ESP.wdtFeed();;
        //delayMicroseconds(us);
      }
      //delayMicroseconds(300);
    }

    int read() {
      if (ready) {
        ready = false;
        //ESP_LOGE("main",'R');
        //Serial.println(buf);
        return buf;
      }
      available();
      return -1;
    }
  private:
    int digRead() {
      return digitalRead(pin_rx);
    }
    void digWrite(int v) {
      digitalWrite(pin_tx, !v);
    }

    unsigned char buf;
    bool ready;
    int pin_tx;
    int pin_rx;
    unsigned long us;
};

SerialS kamSer(RX_PIN, TX_PIN, true);                                                                     // Initialize serial

class MyCustomSensor : public PollingComponent{
 public:
  MyCustomSensor(): PollingComponent(100){

  }

  void setup() override {
    pinMode(RX_PIN, INPUT);
    pinMode(TX_PIN, OUTPUT);
    pinMode(LED, OUTPUT);
    kamSer.begin(KAMBAUD);
    kreg=1;
  }

  Sensor *energy_sensor = new Sensor();
  Sensor *currentPower_sensor = new Sensor();
  Sensor *temperatureT1_sensor = new Sensor();
  Sensor *temperatureT2_sensor = new Sensor();
  Sensor *temperatureDiff_sensor = new Sensor();
  Sensor *flow_sensor = new Sensor();
  Sensor *volumen1_sensor = new Sensor();

  void update() override {
    /* for (int kreg = 0; kreg < NUMREGS; kreg++) {
            kamReadReg(kreg);
            delay(100);
            }*/
         if(kreg<=6) {
                  digitalWrite(LED,LOW);

          long t = millis();
          float val = kamReadReg(kreg);
          ESP_LOGD("main","Read kreg %d ---> %f    %dms",kreg,  val, millis()-t);
          if(val>=0) {
             switch (kreg) {                                                                                                                //This section provided by Robert Wall on http://openenergymonitor.org/emon/node/5718
              case 0 : //emontx.Energy = rval;
                energy_sensor->publish_state(val);
                break;
              case 1 : //emontx.CurrentPower = rval;
                currentPower_sensor->publish_state(val);
                break;
              case 2 : //emontx.TemperatureT1 = rval;
                temperatureT1_sensor->publish_state(val);
                break;
              case 3 : //emontx.TemperatureT2 = rval;
                temperatureT2_sensor->publish_state(val);
                break;
              case 4 : //emontx.TemperatureDiff = rval;
                temperatureDiff_sensor->publish_state(val);
                break;
              case 5 : //emontx.Flow = rval;
                flow_sensor->publish_state(val);
                break;
              case 6 : //emontx.Volumen1 = rval;
                volumen1_sensor->publish_state(val);
                break;
            }
        }
            digitalWrite(LED,HIGH);

    }
    kreg++;
    if(kreg>=NUMREGS+10*30) {
        kreg=0;
    }
     
  }
  private:
    int pin;
    int kreg;
    unsigned long offMs;
    unsigned long onMs;
    unsigned long tm;
    int c;
};




//********************************************Kamstrup Stuff Start********************************************

#define KAM_BUFFER_SIZE 100
// kamReadReg - read a Kamstrup register
float kamReadReg(unsigned short kreg) {

  byte recvmsg[KAM_BUFFER_SIZE];                                                                                                            // buffer of bytes to hold the received data
  float rval;                                                                                                                  // this will hold the final value

  // prepare message to send and send it
  byte sendmsg[] = {
    0x3f, 0x10, 0x01, (kregnums[kreg] >> 8), (kregnums[kreg] & 0xff)
  };
  
  kamSend(sendmsg, 5);
  // listen if we get an answer
  unsigned short rxnum = kamReceive(recvmsg);

  // check if number of received bytes > 0
  if (rxnum != 0) {


    // decode the received message
    rval = kamDecode(kreg, recvmsg);
      //ESP_LOGE("main","Luettu, %f", rval);
    return rval;
  }

return -1;
}

// kamSend - send data to Kamstrup meter
void kamSend(byte const *msg, int msgsize) {

  // append checksum bytes to message
  byte newmsg[msgsize + 2];
  for (int i = 0; i < msgsize; i++) {
    newmsg[i] = msg[i];
  }
  newmsg[msgsize++] = 0x00;
  newmsg[msgsize++] = 0x00;
  int32_t c = crc_1021(newmsg, msgsize);
  newmsg[msgsize - 2] = (c >> 8);
  newmsg[msgsize - 1] = c & 0xff;

  // build final transmit message - escape various bytes
  byte txmsg[20] = {
    0x80
  };                                                                                                  // prefix
  int txsize = 1;
  for (int i = 0; i < msgsize; i++) {
    if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
      txmsg[txsize++] = 0x1b;
      txmsg[txsize++] = newmsg[i] ^ 0xff;
    }
    else {
      txmsg[txsize++] = newmsg[i];
    }
  }
  txmsg[txsize++] = 0x0d;                                                                                                     // EOF

  // send to serial interface
  for (int x = 0; x < txsize; x++) {
    kamSer.write(txmsg[x]);
  }

}

// kamReceive - receive bytes from Kamstrup meter
unsigned short kamReceive(byte recvmsg[]) {

  byte rxdata[50];                                                                                                            // buffer to hold received data
  unsigned long rxindex = 0;
  unsigned long starttime = millis();

  kamSer.flush();                                                                                                             // flush serial buffer - might contain noise

  byte r=0;


  // loop until EOL received or timeout
  while (r != 0x0d) {

    // handle rx timeout
    if (millis() - starttime > KAMTIMEOUT) {
      ESP_LOGE("main","Timed out listening for data");
      return 0;
    }

    // handle incoming data
    if (kamSer.available()) {
     
      
      
      // receive byte
      r = kamSer.read();
      ESP_LOGV("main","ser 0x%x: ", r);
     //delayMicroseconds(100); //Won't work without this???
      if (r != 0x40) {                                                                                                        // don't append if we see the start marker
        // append data
          if(rxindex>=sizeof(rxdata)) {
            ESP_LOGE("main","BUF OVERFLOW error1: ");
            return 0;
        }
            rxdata[rxindex] = r;
      
        rxindex++;
      }

    }
    ESP.wdtFeed();

  }

if(rxindex>sizeof(rxdata)) {
            ESP_LOGE("main","BUF OVERFLOW error2: ");
            return 0;
        }
  // remove escape markers from received data
  unsigned short j = 0;
  for (unsigned short i = 0; i < rxindex - 1; i++) {
    if (rxdata[i] == 0x1b) {
      byte v = rxdata[i + 1] ^ 0xff;
      if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80) {
        ESP_LOGE("main","Missing escape %X", v);
      }
      if(j<KAM_BUFFER_SIZE) recvmsg[j] = v;
      i++;                                                                                                                   // skip
    }
    else {
      if(j<KAM_BUFFER_SIZE) recvmsg[j] = rxdata[i];
    }
    j++;
  }

  // check CRC
  if (crc_1021(recvmsg, j)) {
    ESP_LOGE("main","CRC error: ");
    return 0;
  }

  return j;

}

// kamDecode - decodes received data
float kamDecode(unsigned short const kreg, byte const *msg) {

  // skip if message is not valid
  if (msg[0] != 0x3f or msg[1] != 0x10) {
       ESP_LOGE("main","err1 ");
    return false;
  }
  if (msg[2] != (kregnums[kreg] >> 8) or msg[3] != (kregnums[kreg] & 0xff)) {
       ESP_LOGE("main","err2");
    return false;
  }

  // decode the mantissa
  int32_t x = 0;
  for (int i = 0; i < msg[5]; i++) {
    x <<= 8;
    x |= msg[i + 7];
  }
//ESP_LOGE("main","DEBUG, %d", x);
  // decode the exponent
  int32_t i = msg[6] & 0x3f;
  if (msg[6] & 0x40) {
    i = -i;
  };
  float ifl = pow(10, i);

  
  if (msg[6] & 0x80) {
    ifl = -ifl;
  }
  float tmp = x;
  // return final value
    //ESP_LOGE("main","DEBUG, %f   %f", ifl, tmp);

  return ifl * tmp;

}

// crc_1021 - calculate crc16
long crc_1021(byte const *inmsg, unsigned int len) {
  long creg = 0x0000;
  for (unsigned int i = 0; i < len; i++) {
    int mask = 0x80;
    while (mask > 0) {
      creg <<= 1;
      if (inmsg[i] & mask) {
        creg |= 1;
      }
      mask >>= 1;
      if (creg & 0x10000) {
        creg &= 0xffff;
        creg ^= 0x1021;
      }
    }
  }
  return creg;
}

//********************************************Kamstrup Stuff End********************************************