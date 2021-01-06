#include <Wire.h>
#include <math.h>
#define SLAVE_ADDRESS 0x04
#define version "pool-i2c-mega.ino v0.1"

const byte outPins[] = {34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 50};
const byte inPins[] = {32};
const byte thermistorPins[] = {100, 101};
const byte intellibritePins[] = {51, 52, 53};
const byte outPinsLength = (sizeof(outPins) / sizeof(outPins[0]));
const byte inPinsLength = (sizeof(inPins) / sizeof(inPins[0]));
const byte thermistorPinsLength = (sizeof(thermistorPins) / sizeof(thermistorPins[0]));
const byte intellibritePinsLength = (sizeof(intellibritePins) / sizeof(intellibritePins[0]));
const byte transformerPin = 50;
unsigned int waterPulse = 0;
byte gpio_pin = 0;
byte cmd = 0;
byte intellibriteCount[intellibritePinsLength];
byte intellibriteStatus[intellibritePinsLength];
byte transformerCounter = 0;
byte transformerTimer = 60;

enum {
  T_KELVIN=0,
  T_CELSIUS,
  T_FAHRENHEIT
};

#define EPISCO_K164_10k 4300.0f,298.15f,10000.0f  // B,T0,R0

float Temperature(int AnalogInputNumber,int OutputUnit,float B,float T0,float R0,float R_Balance)
{
  float R,T;

//  R=1024.0f*R_Balance/float(analogRead(AnalogInputNumber)))-R_Balance;
  R=R_Balance*(1024.0f/float(analogRead(AnalogInputNumber))-1);

  T=1.0f/(1.0f/T0+(1.0f/B)*log(R/R0));

  switch(OutputUnit) {
    case T_CELSIUS :
      T-=273.15f;
    break;
    case T_FAHRENHEIT :
      T=9.0f*(T-273.15f)/5.0f+32.0f;
    break;
    default:
    break;
  };

  return T;
}

void receiveData(int byteCount) {
  Serial.print("Received data.... ");
  while (Wire.available()) {
    gpio_pin = Wire.read();
    cmd = Wire.read();
    Serial.print("GPIO: ");
    Serial.print(gpio_pin);
    Serial.print(" : ");
    Serial.println(cmd);

    for (byte g=0; g<outPinsLength; g++) {
      if (outPins[g] == gpio_pin) {
        Serial.println("GPIO matches outPins list");
        if (cmd == 0x01) {
          Serial.println("GPIO ON");
          digitalWrite(gpio_pin, HIGH);
        } else if (cmd == 0x00)  {
          Serial.println("GPIO OFF");
          digitalWrite(gpio_pin, LOW);
        } else if (cmd == 0xFF)  {
          Serial.println("Read requested"); 
        } else {
          Serial.print(" No command for: ");
          Serial.println(cmd);
        }
      }
    }
    for (byte g=0; g<inPinsLength; g++) {
      if (inPins[g] == gpio_pin) {
        Serial.println("GPIO matches inPins list");
        if (cmd == 0xFF)  {
          Serial.println("Read requested"); 
        } else {
          Serial.print(" No command for: ");
          Serial.println(cmd);
        }
      }
    }
    for (byte g=0; g<thermistorPinsLength; g++) {
      if (thermistorPins[g] == gpio_pin) {
        Serial.println("GPIO matches thermistorPins list");
        if (cmd == 0xFF)  {
          Serial.println("Read requested"); 
        } else {
          Serial.print(" No command for: ");
          Serial.println(cmd);
        }
      }
    }
    for (byte g=0; g<intellibritePinsLength; g++) {
      if (intellibritePins[g] == gpio_pin) {
        Serial.println("GPIO matches intellibritePins list");
        if ((cmd != 255) && (intellibriteCount[g]==0)) {
          if ((cmd == 1) && (intellibriteStatus[g] != 0)) { 
          
          } else {
            intellibriteStatus[g]=cmd;
          }
        }
        if (cmd == 0)  {
          Serial.println("GPIO OFF");
          digitalWrite(intellibritePins[g], LOW);
        } else if ((cmd == 1) && (intellibriteCount[g]==0)) {
          Serial.print("GPIO cmd: ");
          Serial.println(cmd);
          
          if(!(digitalRead(transformerPin))) { 
            Serial.println("Turn transformer on");
            digitalWrite(transformerPin, HIGH); 
          }
      
          Serial.print("Intellibrite On ");
          digitalWrite(gpio_pin, HIGH);

        } else if ((cmd > 1) && (cmd <= 14) && (intellibriteCount[g]==0)) {
          Serial.print("GPIO cmd: ");
          Serial.println(cmd);
          intellibriteCount[g]=--cmd;
        } else if (cmd == 255)  {
          Serial.println("Read requested"); 
        } else {
          Serial.print(" No command for: ");
          Serial.println(cmd);
        }
      }
    }
  }
}

void sendData() {
  Serial.print("GPIO: ");
  Serial.print(gpio_pin);
  Serial.print(" ");
  if (gpio_pin == 255) {
    Serial.print("Discovery request.... Wire.write length: ");
    Serial.print(outPinsLength);
    Wire.write(outPinsLength);
    Wire.write(outPins, outPinsLength);
    Serial.println("...  Sent gpio list");
  } else {
    for (byte g=0; g<outPinsLength; g++) {
      if (outPins[g] == gpio_pin) {          
        Wire.write(digitalRead(gpio_pin));
        Serial.println("GPIO matches outPins list: ");
        Serial.println(digitalRead(gpio_pin));
      }
    }
    for (byte g=0; g<inPinsLength; g++) {
      if (inPins[g] == gpio_pin) {
        Serial.println("GPIO matches inPins list: ");
        Serial.println(digitalRead(gpio_pin));
      }
    }
    for (byte g=0; g<thermistorPinsLength; g++) {
      if (thermistorPins[g] == gpio_pin) {
        Serial.println("GPIO matches thermistorPins list: ");
        byte a=gpio_pin - 100;
        float temp=Temperature(a,T_FAHRENHEIT,EPISCO_K164_10k,10000.0f);
        byte * b = (byte *) &temp;
        Wire.write(b,4);
      }
    }
    for (byte g=0; g<intellibritePinsLength; g++) {
      if (intellibritePins[g] == gpio_pin) {
        Serial.print("Read GPIO matches intellibritePins list ");
        Serial.println(intellibriteStatus[g]);
        Wire.write(intellibriteStatus[g]);
      }
    }
  }
}



void setup()
{
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println(version);
  Serial.print("Ready! - Slave Address = ");
  Serial.println(SLAVE_ADDRESS);
  for (int i=0; i<outPinsLength; i++) {
    pinMode(outPins[i], OUTPUT);
    digitalWrite(outPins[i], LOW);
    Serial.print("outPins=");
    Serial.println(outPins[i]);
  }
  for (int i=0; i<inPinsLength; i++) {
    pinMode(inPins[i], INPUT);
    Serial.print("inPins=");
    Serial.println(inPins[i]);
  }
  for (int i=0; i<intellibritePinsLength; i++) {
    pinMode(intellibritePins[i], OUTPUT);
    digitalWrite(intellibritePins[i], LOW);
    intellibriteCount[i] = 0;
    intellibriteStatus[i] = 0;
    Serial.print("intellibritePins=");
    Serial.print(intellibritePins[i]);
    Serial.print(", Status=");
    Serial.println(intellibriteStatus[i]);
  }
  pinMode(transformerPin, OUTPUT);
  digitalWrite(transformerPin, LOW);
  // initialize timer1 

  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR3B |= (1 << WGM12);   // CTC mode
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  for (int i=0; i<intellibritePinsLength; i++) {
    if (intellibriteCount[i] > 0) {
      if(!(digitalRead(transformerPin))) { 
        Serial.println("Turn transformer on");
        digitalWrite(transformerPin, HIGH); 
      }
      if (!(digitalRead(intellibritePins[i]))) {
        digitalWrite(intellibritePins[i], HIGH);
        --intellibriteCount[i];
      } else {
        digitalWrite(intellibritePins[i], LOW);
      }
    }
  }
  bool inUse = false;
  
  for (byte t=0; t<intellibritePinsLength; t++) {
    if (intellibriteStatus[t] > 0) {
      inUse = 1;
      transformerCounter = transformerTimer;
    }
  }
  if ((!inUse) && (digitalRead(transformerPin))) {
    if (transformerCounter == 0) { 
      Serial.println("Transformer Off");
      digitalWrite(transformerPin, LOW); 
    } else {
      --transformerCounter;
    }     
  }
}


void loop()
{

}
