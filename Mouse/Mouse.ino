/*
USB Pinout (Left to Right, USB symbol up)
4: GND
3: Clk
2: Data
1: Vcc
int counterx;
int countery;
#include "ps2.h"

PS2 mouse(6,5);

void setup(){
  Serial.begin(9600);
  while(!Serial); 
  Serial.println("Setup");
  mouse.mouse_init();
  Serial.println("Mouse Ready");
  counterx = 0;
  countery = 0;
}

void loop(){
  char stat,x,y;
  mouse.mouse_pos(stat,x,y);
  
  Serial.print(stat, BIN);
  Serial.print("\tx=");
  Serial.print(x, DEC);
  Serial.print("\ty=");
  Serial.println(y, DEC);
  counterx += x;
  countery += y;
  Serial.println(counterx);
  Serial.println(countery);
  
  
  delay(100);
}
