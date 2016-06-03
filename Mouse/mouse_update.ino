
/*USB Pinout (Left to Right, USB symbol up)
4: GND
3: Clk
2: Data
1: Vcc*/
int counterx;
int countery;
int checkCounter;
int reset_counter;
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
  checkCounter = 0;
  reset_counter = 0
}
void loop(){
  char stat,x,y;
  mouse.mouse_pos(stat,x,y);
  
  counterx += x;
  countery += y;
  checkCounter += 1;

  if(x == y == 0) {
    reset_counter++; 
  }

  if(reset_counter >= 50) { //reset everything if still
    counterx = 0
    countery = 0
    checkCounter = 0
    reset_counter = 0
  }

  if (checkCounter >= 100){
    Serial.print(stat, BIN);
    Serial.print("\tx=");
    Serial.print(x, DEC);
    Serial.print("\ty=");
    Serial.println(y, DEC);
    Serial.print("total x distance: ");
    Serial.println(counterx);
    Serial.print("total y distance: ");
    Serial.println(countery);
    checkCounter = 0;
  }
  
  delay(1);
}

/*test robot moving one square forward, measure how much it tracks
if still for x counts, check if it's in the correct position (or close), adjust, then reset counter

find counter for 90 deg turn*/
