#include <Arduino.h>

int latchP = 10; //output parralel data
int clockP = 9; //clockcycle
int dataP = 8; //serial
uint16_t LEDS = 0; //led light data
int lastr; 

void setup() {
//set pins as output
pinMode(latchP, OUTPUT);
pinMode(clockP, OUTPUT);
pinMode(dataP, OUTPUT);
Serial.begin(9600);
}

void loop() {
if(Serial.available() > 0){
  int val = Serial.read();
}

LEDS = 0;
for(int i = 0; i < val; i++){
  bitSet(LEDS, i);
}

digitalWrite(latchP, LOW);
shiftOut(dataP, clockP, MSBFIRST, LEDS >> 8);
shiftOut(dataP, clockP, MSBFIRST, LEDS);
digitalWrite(latchP, HIGH);

delay(50);

}

