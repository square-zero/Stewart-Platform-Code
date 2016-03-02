
/*
 Controlling a 6-DOF Stewart Platform with Servos in real-time
 using a gyroscope and accelerometer to automatically keep the
 platform level on an uneven surface or during acceleration.
*/

#include <Servo.h>
#include <MatrixMath.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

// Calibration stage -- resets servos and does not run the program
//#define CALIBRATE

#ifndef CALIBRATE
  // DEBUG STATE -- (De)comment to enable various debug messages.
  //#define DEBUG
  #ifdef DEBUG
    // Debug messages print to serial port, generally slows things down.

    //#define DEBUG_ALL // prints debug messages for EVERYTHING -- consumes a lot of memory
    // otherwise, select which debug messages to use
    //#define DEBUG_ACCEL // accelerometer
    //#define DEBUG_ACCEL_DIR // accelerometer orientation
    //#define DEBUG_SERVO // prints debug messages for servos
    //#define DEBUG_PLAT // platform orientation
    //#define DEBUG_LEGS // leg lengths
    //#define DEBUG_TEST // run basic debug test
    #define DEBUG_SERVO_ANGLES // servo angles + height constants
  #elif defined(GYRO_TEST)
    // something to test GYRO
  #else
    // Otherwise, run the program
    #define RUN_PROG
  #endif



#endif

// MATH CONSTANTS
const float pi = 3.1415926536; // a few more digits than we should need
const float rad2deg = 180/pi; // convert radians to degrees
const float deg2rad = pi/180; // convert degrees to radians
const float r_o = 1600/pi; // constant used for converting servo angle into pulse width
const float microSec = 0.000001; // to convert micro-seconds to seconds

// Control Servos for Stewart Platform:
Servo servo[6];

// Parameters for Stewart Platform:
/* THESE NEED TO BE MEASURED */
const float bRad = 4; // distance between center of base and servo arm (inches)
const float pRad = 2.5; // distance between center of platform and mounting point on tray (inches)
float bPts[6][3]; // XYZ coords for six "mount points" on the base
float pPts[6][3]; // XYZ coords for six "mount points" on an untilted platform
const float armLen = 1; // length of servo arm, in inches
const float legLen = 7.25; // length of connecting rod between servo and platform, in inches
const float d_AngleTray = 110*deg2rad; // angle in radians between pairs of platform mounts
const float d_AngleBase = 50*deg2rad; // angle in radians between pairs of base mounts
#ifdef OLD
  // offset odd motors by pi radians.
  // turns out that this causes problems.
  const float beta[] = {0, pi, 2*pi/3, 5*pi/3, 4*pi/3, pi/3}; // beta values for servos
#else
  const float beta[] = {0, 0, 2*pi/3, 2*pi/3, 4*pi/3, 4*pi/3}; // beta values for servos
#endif

// Various matrices in the order we need them
float orientation[3]; // orientation vector [roll (x), pitch (y), yaw (z)]
float acceleration[3]; // acceleration vector [a_x, a_y, a_z]
float platformRotationMatrix[9];
float platformPosition[6]; // [X, Y, Z, x_a, y_a, z_a]
float tHome[3]; // default displacement vector from base to tray
float tDisp[3]; // actual displacement vector
float legLengths[6][3];  // lengths of individual legs of the platform represented as vectors
float platDisp[6][3]; // displacement vectors from center of base to connecting points on legs
float trayDisp[6][3]; // displacement vectors from center of tray to connecting points on legs
float servoAngles[6]; // desired angles of servos
float servoPulseWidths[6]; // current PWM pulse widths in uSeconds
float desiredPos[3];

float alpha_o; // default (resting) angle of servos
float h_o; // default (resting) height of platform 

// Code to set up and configure hardware
void setup() {
  // uses Serial port on baud 115200
  Serial.begin(115200);
  
  // calculate mount points for platform
  calcMountPoints(pPts, bPts, d_AngleTray, d_AngleBase);

  // calculate h_o (default platform height)
  h_o = sqrt(  armLen*armLen
             + legLen*legLen 
             - (bPts[0][0]*bPts[0][0] - pPts[0][0]*pPts[0][0])
             - (bPts[0][1]*bPts[0][1] - pPts[0][1]*pPts[0][1]));
  tHome[2] = h_o;

  // calc alpha_o (default servo angle)
  float L = 2*armLen*armLen;
  float M = 2*armLen*(pPts[0][0] - bPts[0][0]);
  float N = 2*armLen*(h_o + pPts[0][2]);
  alpha_o = asin(L/(M*M + N*N)) - atan(M/N);

  #ifdef DEBUG_SERVO_ANGLES
    Serial.println("Platform default height + angles:");
    Serial.println("Default height       : " + (String)(h_o) + " inches");
    Serial.println("Default servo angle  : " + (String)(alpha_o*rad2deg) + " degrees");
  #endif
  // ensures that all arduino power pins power-up fully
  delay(1000);
  
  // set up Servos
  // PWM's on pins 3, 5, 6, 9, 10, 11
  servo[0].attach(3);  // attaches the servo on pin 3 to the servo object
  servo[1].attach(5);  // attaches the servo on pin 5 to the servo object
  servo[2].attach(6);  // attaches the servo on pin 6 to the servo object
  servo[3].attach(9);  // attaches the servo on pin 9 to the servo object
  servo[4].attach(10);  // attaches the servo on pin 10 to the servo object
  servo[5].attach(11);  // attaches the servo on pin 11 to the servo object

  // Configure Gyroscope
  setupGyro();

  // Ensure that Gyroscope powers up fully
  delay(500);

  // Reset all Servo's to neutral position
  resetServos();

  // Input positional information about the platform
  /* SUBJECT TO CHANGE UPON PLATFORM CONSTRUCTION */
  for (int i = 0; i < 3; i++) {
    tDisp[i] = tHome[i];
  }

  // DEBUG SKETCH
  #if defined(DEBUG_TEST)
    setup_DEBUG();
  #elif !defined(PROG_RUN)
    // if TIME_TRIAL, will run program ten times and determine average speed
    // else, runs program one time (useful for debugging)
    //#define TIME_TRIAL
    #ifdef TIME_TRIAL
      unsigned long start, end;
      float delta, deltaSum, freq;
      // run method ten times, calculate average run time
      int max = 10;
      #define ONE_TIME
      for (int i = 0; i < max; i++) {
        start = micros();
        selfBalanceDEBUG();
        end = micros();
        delta = (end - start);
        deltaSum += delta;
        freq = 1.0 / (delta*microSec);
        #ifdef ONE_TIME
          Serial.println();
          Serial.println("selfBalance Method Run-time : approximately " + (String)(delta) + " usec");
          Serial.println("            with frequency of approximately " + (String)(freq) + " Hertz");
        #endif
      }
      deltaSum = 1.0 * deltaSum / max;
      freq = 1.0 / (deltaSum*microSec);
      Serial.println("Average run-time  : " + (String)(delta) + " microseconds");
      Serial.println("Average frequency : " + (String)(freq) + " Hertz");
    #else
      selfBalanceDEBUG();
    #endif
  #endif

}

// Some debug tests that run at startup
void setup_DEBUG() {

  // Calculate the cross-product of two platform mount point vectors
  float vecA[] = {pPts[0][0], pPts[0][1], pPts[0][2]};
  float vecB[] = {pPts[2][0], pPts[2][1], pPts[2][2]};
  float out[3];

  Serial.println("INITIAL CONDITIONS :: ");
  calcXProduct(vecA, vecB, out, "Normal vector to platform");
  printArbMat3(pPts, 6, "Platform points");

  // Calculate the cross-product of two platform displacement vectors
  // with no tilt in any direction --- SHOULD MATCH RESULTS ABOVE ---
  orientation[0] = 0.0;
  orientation[1] = 0.0;
  orientation[2] = 0.0;
  Serial.println();
  Serial.println("TEST CASE 1 :: NO TILT");
  selfBalanceDEBUG();

  for (int i = 0; i < 3; i++) {
    vecA[i] = platDisp[0][i];
    vecB[i] = platDisp[2][i];
  }

  calcXProduct(vecA, vecB, out, "Normal vector to platform");
  Serial.println("This should match above");
  printArbMat3(platDisp, 6, "Platform points");

  // change orientation, repeat selfBalanceDEBUG() and compare results
  orientation[0] = 10.0*deg2rad;
  orientation[1] = 0.0;
  orientation[2] = 0.0;
  Serial.println();
  Serial.println("TEST CASE 2 :: PLATFORM ROLL");
  selfBalanceDEBUG();

  for (int i = 0; i < 3; i++) {
    vecA[i] = platDisp[0][i];
    vecB[i] = platDisp[2][i];
  }
  calcXProduct(vecA, vecB, out, "Normal vector to platform");
  printArbMat3(platDisp, 6, "Platform points");
}

// Main loop
void loop() {
  #ifdef RUN_PROG
    selfBalance();
    // Repeat forever!
  #endif
}

// Self-balancing routine using reverse-kinematics
void selfBalance() {
  
  /* Following methods commented out until Gyroscope input desired.
   * Until such a time, this code will run on manual input, given 
   * one time at the beginning of the program.
   */

  // Read acceleration from Gyroscope
  readGyro(acceleration);

  // Calculate orientation from Gyroscope
  /* METHOD BEING WRITTEN, NOT COMPLETE */
  calcOrientation(orientation, acceleration);


  // Calculate rotational matrix
  /* METHOD WRITTEN, TESTED */
  calcRotMatrix(platformRotationMatrix, orientation);

  // Calculate effective leg lengths
  /* METHOD WRITTEN, TESTED. NOT FULLY VERIFIED */
  calcLegLengths2(legLengths, desiredPos);
  
  // Calculate the angles required for each servo
  /* METHOD WRITTEN, TESTED. NOT FULLY VERIFIED */
  calcServoAngles(legLengths, servoAngles);

  // Calculate the pulse widths required for each servo
  /* METHOD WRITTEN, NOT TESTED */
  calcPulseWidths(servoPulseWidths, servoAngles);

  // Output the pulse widths to the output PWM to drive each servo
  /* METHOD WRITTEN, NOT TESTED */
  setPulseWidths(servo, servoPulseWidths);
}

// Self-balancing routine using reverse-kinematics
// MOSTLY IDENTICAL to selfBalance, except for debugging 
// Refer to selfBalance() for comments
void selfBalanceDEBUG() {
  //readGyro(acceleration);
  //calcOrientation(orientation, acceleration);
  calcRotMatrix(platformRotationMatrix, orientation);
  calcLegLengths2(legLengths, desiredPos);
  calcServoAngles(legLengths, servoAngles);
  calcPulseWidths(servoPulseWidths, servoAngles);
  setPulseWidths(servo, servoPulseWidths);
}

// calculate the vectors for the upper and lower mounting points
// given physical information about the platform
///////
// Assumes three-way symmetry, with an angular difference (d_Angle) between pairs
void calcMountPoints(float pPts[][3], float bPts[][3], float d_AngleTray, float d_AngleBase) {
  // unit vector as a starting point
  float unit[] = {1, 0, 0};
  float tempLP[3];
  float tempRP[3];
  float tempLB[3];
  float tempRB[3];

  // temporary storage for matrix math
  float tempLP2[3];
  float tempRP2[3];
  float tempLB2[3];
  float tempRB2[3];

  // right-wards and left-wards rotation matrices
  // L/R = left- or right-rotation
  // P/B = platform or base
  float tempRotationMatrixLP[9];
  float tempRotationMatrixRP[9];
  float tempRotationMatrixLB[9];
  float tempRotationMatrixRB[9];

  // PLATFORM / BASE MOUNTS
  calcRotMatrixZ(tempRotationMatrixRP, -d_AngleTray/2);
  calcRotMatrixZ(tempRotationMatrixLP, d_AngleTray/2);
  calcRotMatrixZ(tempRotationMatrixRB, -d_AngleBase/2);
  calcRotMatrixZ(tempRotationMatrixLB, d_AngleBase/2);

  for (int i = 0; i < 3; i++) {

    // Rotate unit towards direction of mount 2*i, 2*i + 1
    Matrix.Multiply(tempRotationMatrixLP, unit, 3, 3, 1, tempLP);
    Matrix.Multiply(tempRotationMatrixRP, unit, 3, 3, 1, tempRP);
    Matrix.Multiply(tempRotationMatrixLB, unit, 3, 3, 1, tempLB);
    Matrix.Multiply(tempRotationMatrixRB, unit, 3, 3, 1, tempRB);
    
  
    // rotate by 120 degrees if not on last pair
    if (i < 2) {

      // three-way symmetry
      calcRotMatrixZ(tempRotationMatrixRP,  (i + 1)*120*deg2rad + d_AngleTray/2);
      calcRotMatrixZ(tempRotationMatrixLP,  (i + 1)*120*deg2rad - d_AngleTray/2);
      calcRotMatrixZ(tempRotationMatrixRB,  (i + 1)*120*deg2rad + d_AngleBase/2);
      calcRotMatrixZ(tempRotationMatrixLB,  (i + 1)*120*deg2rad - d_AngleBase/2);

      Matrix.Multiply(tempRotationMatrixLP, tempLP, 3, 3, 1, tempLP2);
      Matrix.Multiply(tempRotationMatrixRP, tempRP, 3, 3, 1, tempRP2);
      Matrix.Multiply(tempRotationMatrixLB, tempLB, 3, 3, 1, tempLB2);
      Matrix.Multiply(tempRotationMatrixRB, tempRB, 3, 3, 1, tempRB2);
    }

    // copy tempL/R to tempL2/R2 (for matrix math)
    for (int k = 0; k < 3; k++) {
      tempLP2[k] = tempLP[k];
      tempRP2[k] = tempRP[k];
      tempLB2[k] = tempLB[k];
      tempRB2[k] = tempRB[k];
    }

    // scale vectors to appropriate size
    scaleVec(tempLP2, pRad);
    scaleVec(tempRP2, pRad);
    scaleVec(tempLB2, bRad);
    scaleVec(tempRB2, bRad);

    // store value in pPts
    pPts[2*i    ][0] = tempLP2[0];
    pPts[2*i + 1][0] = tempRP2[0];
    pPts[2*i    ][1] = tempLP2[1];
    pPts[2*i + 1][1] = tempRP2[1];
    pPts[2*i    ][2] = tempLP2[2];
    pPts[2*i + 1][2] = tempRP2[2];

    bPts[2*i    ][0] = tempLB2[0];
    bPts[2*i + 1][0] = tempRB2[0];
    bPts[2*i    ][1] = tempLB2[1];
    bPts[2*i + 1][1] = tempRB2[1];
    bPts[2*i    ][2] = tempLB2[2];
    bPts[2*i + 1][2] = tempRB2[2];
  }


  #ifdef OLD
  for (int i = 0; i < 3; i++) {

    // Rotate unit towards direction of mount 2*i, 2*i + 1
    Matrix.Multiply(tempRotationMatrixLB, unit, 3, 3, 1, tempLB);
    Matrix.Multiply(tempRotationMatrixRB, unit, 3, 3, 1, tempRB);

    // copy tempL/R to tempL2/R2 (for matrix math)
    for (int k = 0; k < 3; k++) {
      tempLB2[k] = tempLB[k];
      tempRB2[k] = tempRB[k];
    }

    // rotate by 120 degrees if not on last pair
    if (i < 2) {
      // three-way symmetry (120*deg2rad)
      calcRotMatrixZ(tempRotationMatrixR,  (i + 1)*120*deg2rad + d_AngleBase/2);
      calcRotMatrixZ(tempRotationMatrixL,  (i + 1)*120*deg2rad - d_AngleBase/2);

      Matrix.Multiply(tempRotationMatrixL, tempL, 3, 3, 1, tempL2);
      Matrix.Multiply(tempRotationMatrixR, tempR, 3, 3, 1, tempR2);
    }

    // scale vectors to appropriate size
    scaleVec(tempL2, bRad);
    scaleVec(tempR2, bRad);

    // store value in pPts
    bPts[2*i    ][0] = tempL2[0];
    bPts[2*i + 1][0] = tempR2[0];
    bPts[2*i    ][1] = tempL2[1];
    bPts[2*i + 1][1] = tempR2[1];
    bPts[2*i    ][2] = tempL2[2];
    bPts[2*i + 1][2] = tempR2[2];
  }

  #endif
  

  // For some reason, first two points get swapped when they shouldn't.
  // This switches them back.
  // Feel free to remove this part if it turns out that I was wrong.
  for (int i = 0; i < 3; i++) {
    tempLB[i] = pPts[0][i];
    pPts[0][i] = pPts[1][i];
    pPts[1][i] = tempLB[i];

    tempLB[i] = bPts[0][i];
    bPts[0][i] = bPts[1][i];
    bPts[1][i] = tempLB[i];
  }


  // Prints to serial port for debugging
  #if defined(DEBUG_ALL) || defined(DEBUG_SERVO)
    printArbMat3(pPts, 6, "Platform mounts");
    printArbMat3(bPts, 6, "Base mounts");
  #endif

}

// scales a vector by the specified amount
void scaleVec(float vector[], float scale) {
  for (int i = 0; i < 3; i++) {
    vector[i] = scale*vector[i];
  }
}

// Calculates the Z-Axis rotation matrix
// used for creating mount points on base and platform
void calcRotMatrixZ(float rotationMatrix[], float zAngle) {

  rotationMatrix[0] = cos(zAngle);
  rotationMatrix[1] = -sin(zAngle);
  rotationMatrix[2] = 0;
  rotationMatrix[3] = sin(zAngle);
  rotationMatrix[4] = cos(zAngle);
  rotationMatrix[5] = 0;
  rotationMatrix[6] = 0;
  rotationMatrix[7] = 0;
  rotationMatrix[8] = 1;

}

// Read acceleration values from Gyroscope and store them in global array
void readGyro(float acceleration[]) {
  
  // read values from gyroscope
  imu.readAccel();

  acceleration[0] = imu.ax;
  acceleration[1] = imu.ay;
  acceleration[2] = imu.az;
  #if defined(DEBUG_ALL) || defined(DEBUG_ACCEL)
    Serial.println();
    Serial.println("Accel:");
    Serial.println((String)(acceleration[0]));
    Serial.println((String)(acceleration[1]));
    Serial.println((String)(acceleration[2]));
  #endif
}

// Calculate orientation of platform using acceleration vector
void calcOrientation(float orientation[], float acceleration[]) {
  float mag = calcVecMag(acceleration);
  // normalize acceleration vector
  float unit[3];
  for (int i = 0; i < 3; i++) {
    unit[i] = acceleration[i]/mag;
  }

  // For the purposes of this calculation, we assume a rotated set of axes, x'-y'-z' such that
  // x' == z      roll'  == yaw
  // y' == x      pitch' == roll
  // z' == y      yaw'   == pitch
  // thus, by calculating the pitch and yaw for our normal vector,
  // we can transform it's pitch and yaw into roll and pitch for the platform

  // PLATFORM ROLL  --- NORM YAW
  orientation[0] = asin(-unit[1]);

  // PLATFORM PITCH --- NORM ROLL
  orientation[1] = atan2(unit[0], unit[2]);

  // PLATFORM YAW   --- NORM PITCH
  // platform YAW is immaterial. This value can theoretically be set to anything
  // but to minimize the risk of problems, we'll leave it at zero for now.
  orientation[2] = 0;


  #if defined(DEBUG_ALL) || defined(DEBUG_ACCEL_DIR)
    Serial.println();
    Serial.println("Tilt:");
    Serial.println((String)(orientation[0]));
    Serial.println((String)(orientation[1]));
    Serial.println((String)(orientation[2]));
  #endif
}

// Calculates the rotational matrix for the inverse kinematics 
// problem of controlling a Stewart Platform
// In essence, the rotational matrix represents the rotated plane
// of the platform that we are trying to achieve.
/////
// INPUTS ARE MEASURED IN RADIANS
void calcRotMatrix(float platformRotationMatrix[], float orientation[]) {

  // Calculate trig values for faster operation
  // ROLL   - X Axis
  float cr = cos(orientation[0]);
  float sr = sin(orientation[0]);

  // PITCH  - Y Axis
  float cp = cos(orientation[1]);
  float sp = sin(orientation[1]);

  // YAW    - Z Axis
  float cy = cos(orientation[2]);
  float sy = sin(orientation[2]);

  // assign values to rotation matrix using product form
  // see "Maths of the Stewart Platform" for equations
  platformRotationMatrix[0] = cy*cp;
  platformRotationMatrix[1] = cy*sp*sr - sy*cr;
  platformRotationMatrix[2] = sy*sr + cy*sp*cr;
  platformRotationMatrix[3] = sy*cp;
  platformRotationMatrix[4] = cy*cr + sy*sp*sr;
  platformRotationMatrix[5] = sy*sp*cr - cy*sr;
  platformRotationMatrix[6] = -sp;
  platformRotationMatrix[7] = cp*sr;
  platformRotationMatrix[8] = cp*cr;

}

// Using the rotational matrix, calculates the lengths of the six legs
// that will move the platform to this position
void calcLegLengths2(float legLengths[][3], float pos[]) {
  // Find the positional displacement between the center of the base
  // and the center of the platform
  calcT2(tDisp, pos);

  // temp storage to play nice with Matrix.Multiply()
  float tempDisp[3];
  float tempOut[3];

  // For each leg
  for (int i = 0; i < 6; i++) {
    // grab the neutral displacement (that is, when the tray is flat)
    // between center of tray and where leg i connects
    tempDisp[0] = pPts[i][0];
    tempDisp[1] = pPts[i][1];
    tempDisp[2] = pPts[i][2];

    // use Matrix.Multiply to rotate that vector, stored in tempOut
    Matrix.Multiply(platformRotationMatrix, tempDisp, 3, 3, 1, tempOut); 

    // Copy tempOut to platDisp 
    platDisp[i][0] = tempOut[0];
    platDisp[i][1] = tempOut[1];
    platDisp[i][2] = tempOut[2];    

    // now we sum the following:
    //  + X/Y/Z platform displacement
    //  + rotated mounting point vector for each leg
    //  - distance between base center and servo axis
    legLengths[i][0] = tDisp[0] + tempOut[0] - bPts[i][0];  // X
    legLengths[i][1] = tDisp[1] + tempOut[1] - bPts[i][1];  // Y
    legLengths[i][2] = tDisp[2] + tempOut[2] - bPts[i][2];  // Z
  }

  #if defined(DEBUG_ALL) || defined(DEBUG_LEGS)
    float temp[3];
    float legMagnitudes[6];
    printArbMat3(legLengths, 6, "leg vectors");
    for (int i = 0; i < 6; i++) {
      temp[0] = legLengths[i][0];
      temp[1] = legLengths[i][1];
      temp[2] = legLengths[i][2];

      legMagnitudes[i] = calcVecMag(temp);
    }
    Matrix.Print(legMagnitudes, 6, 1, "leg magnitudes");
    Serial.println(); 
  #endif
}

// Calculated vector T, used for computing tray-platform displacement.
// Essentially, this is the desired XYZ-displacement of the platform.
// Since we are mostly interested in tilting, this fairly simple for now.
// INPUT displacement, an XYZ vector
void calcT2(float T[], float displacement[]) {
  for (int i = 0; i < 3; i++) {
    T[i] = displacement[i] + tHome[i];
  }
}

// Using the calculated leg lengths, determine the angle that the servos
// need to be at in order to achieve the desired effect
void calcServoAngles(float legLengths[][3], float servoAngles[]) {
  float temp[3];
  float x_p;
  float x_b;
  float y_p;
  float y_b;
  float z_p;
  float z_b;


  for (int i = 0; i < 6; i++) {

    // platform points of interest
    x_p = bPts[i][0] + legLengths[i][0];
    y_p = bPts[i][1] + legLengths[i][1];
    z_p = bPts[i][2] + legLengths[i][2];

    // base points of interest
    x_b = bPts[i][0];
    y_b = bPts[i][1];
    z_b = bPts[i][2];
    
    // grab the legLengths vector for leg i
    for (int k = 0; k < 3; k++) {
      temp[k] = legLengths[i][k];
    }

    // This part should be correct.
    // Calculate parameters L, M, N
    float L = calcVecMag(temp)*calcVecMag(temp) - (legLen*legLen - armLen*armLen);
    float M = 2*armLen*(z_p - z_b);
    float N = 2*armLen*(cos(beta[i])*(x_p - x_b) + sin(beta[i])*(y_p - y_b));

    // Calculate the angle
    if (i%2 == 0) {
      servoAngles[i] = asin(L/sqrt(M*M + N*N)) - atan(N/M);
    } else {
      servoAngles[i] = /* pi - */ (asin(L/sqrt(M*M + N*N)) - atan(N/M));
    }
    #ifdef DEBUG_ALL
      Serial.println();
      Serial.println("Calculating alpha for leg " + (String)(i));
      Serial.println("Beta = " + (String)(beta[i]));
      Serial.println("L = " + (String)(L));
      Serial.println("M = " + (String)(M));
      Serial.println("N = " + (String)(N));
    #endif
  }

  #if defined(DEBUG_ALL) || defined(DEBUG_SERVO_ANGLES)
    Serial.println();
    Serial.println("Servo angles (degrees) : ");
    for (int i = 0; i < 6; i++) {
      Serial.println("Servo " + (String)(i) + " : " + (String)(servoAngles[i]*rad2deg)); 
    }
  #endif
}

// From the angles, determine the length of the PWM for each servo.
// Angle equation is reflected for even / odd servos
void calcPulseWidths(float servoPulseWidths[], float angles[]) {
  for (int i = 0; i < 3; i++) {
    // EVEN SERVOS (0, 2, 4)
    servoPulseWidths[2*i    ] = DEFAULT_PULSE_WIDTH + ((angles[2*i] - alpha_o))*r_o;
    
    // ODD SERVOS (1, 3, 5)
    servoPulseWidths[2*i + 1] = DEFAULT_PULSE_WIDTH - ((angles[2*i + 1] - alpha_o))*r_o;
  }

  #if defined(DEBUG_ALL) || defined(DEBUG_SERVO_ANGLES)
    Serial.println();
    Serial.println("Servo Pulse Widths");
    for (int i = 0; i < 6; i++) {
      float temp = servoPulseWidths[i];
      Serial.println("Servo " + (String)(i) + " : " + (String)(temp) + " uSeconds");
    }
  #endif
}

// Specifies pulse duration for controlling servo PWM's
void setPulseWidths(Servo servo[], float servoPulseWidths[]) {
  for (int i = 0; i < 6; i++) {
    servo[i].writeMicroseconds(servoPulseWidths[i]);
  }
}

// Brings all servos to their default neutral position
void resetServos() {
  for (int i = 0; i < 6; i++) {
    servo[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
  }
}

// Calculates (and returns) the magnitude of a vectorin R3
float calcVecMag(float v[]) {
  float temp;
  for (int i = 0; i < 3; i++) {
    temp += v[i]*v[i];
  }
  return sqrt(temp);
}

// prints out an m x 3 matrix to the serial port
// used for debugging
void printArbMat3(float mat[][3], int rows, String id) {
    // mat = input matrix (m x 3)
    int i,j;
    Serial.println();
    Serial.println(id);
    for (i = 0; i < rows; i++){
        for (j = 0; j < 3; j++){
            Serial.print(mat[i][j]);
            Serial.print("\t");
        }
        Serial.println();
    }
}

// calculates the cross product of two vectors
// stores result in out[]
// out = A x B
void calcXProduct(float vecA[], float vecB[], float out[]) {
  out[0] = vecA[1]*vecB[2] - vecB[1]*vecA[2];
  out[1] = vecA[2]*vecB[0] - vecB[2]*vecA[0];
  out[2] = vecA[0]*vecB[1] - vecB[0]*vecA[1];

  float temp = calcVecMag(out);

  for (int i = 0; i < 3; i++) {
    out[i] = out[i]/temp;
  }

  #ifdef DEBUG_PLAT
    Serial.println();
    Serial.println("Cross-product: " + (String)(out[0]) + "*i, " + (String)(out[1]) + "*j, " + (String)(out[2]) + "*k");
  #endif
}

// calculates the cross product of two vectors
// stores result in out[]
// out = A x B
// String is used for debugging purposes
void calcXProduct(float vecA[], float vecB[], float out[], String id) {
  out[0] = vecA[1]*vecB[2] - vecB[1]*vecA[2];
  out[1] = vecA[2]*vecB[0] - vecB[2]*vecA[0];
  out[2] = vecA[0]*vecB[1] - vecB[0]*vecA[1];

  float temp = calcVecMag(out);

  for (int i = 0; i < 3; i++) {
    out[i] = out[i]/temp;
  }

  #ifdef DEBUG_PLAT
    Serial.println();
    Serial.println(id + ": " + (String)(out[0]) + "*i, " + (String)(out[1]) + "*j, " + (String)(out[2]) + "*k");
  #endif
}


