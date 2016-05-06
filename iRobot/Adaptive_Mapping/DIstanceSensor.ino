#include <SharpIR.h>

#define ir1 A2
#define ir2 A1
#define model 1080

boolean done=false;


SharpIR sharp1(ir1, 25, 93, model);
SharpIR sharp2(ir2, 25, 93, model);

// ir: the pin where your sensor is attached
// 25: the number of readings the library will make before calculating a mean distance
// 93: the difference between two consecutive measurements to be taken as valid
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)



void setup(){
  
  Serial.begin(9600);
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  Serial.print('.');
  
}

void loop(){

  delay(100);    // it gives you time to open the serial monitor after you upload the sketch
  
  int dis1=sharp1.distance();  // this returns the distance to the object you're measuring
  int dis2=sharp2.distance();  // this returns the distance to the object you're measuring

  //Serial.print("Mean distance: ");  // returns it to the serial monitor
  Serial.println(dis1);
  //Serial.print('--');
  Serial.println(dis2);
  //Serial.print('.');
 
}
  
