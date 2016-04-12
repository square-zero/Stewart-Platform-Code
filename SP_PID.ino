/*
 *    Stewart Platform Controller Code:
 *
 *    This code should serve to operate a custom Stewart platform, which is
 *    a 6-degrees-of-freedom (6DOF) hexapod platform which uses six rotary
 *    actuators to manipulate a tray by cartesian translation (moving up/down
 *    and to the sides) as well as rotation in any direction (pitch/roll/yaw).
 *
 *    The basis for this code is taken from a mathematical analysis of the platform
 *    model found online which uses reverse kinematics to determine the required
 *    length of each actuator in order to manipulate the platform to a specified
 *    position. See below for a link to the paper.
 *
 *
 *    ///////////////////////
 *    // TO BE IMPLEMENTED //
 *    /////////////////////////////////////////////////////////////////////////////
 *    //  This basic model is then extended to use a feedback loop using real-time data
 *    //  taken from a LSM9DS1 Inertial Measurement Unit (IMU), which measures the current
 *    //  tilt of the platform and then attempts to correct the platform so that the 
 *    //  tray is in a flat, level position. 
 *    //////////////////////////////////////
 *
 *      Reference Material:
 *    Mathematics Of The Stewart Platform
 *    https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf    
 */
#include <Servo.h>
#include <MatrixMath.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>

/*
 *  BOOLEAN DEFINITIONS
 */
#define TRUE   1
#define FALSE   0

/*
 *  SERIAL BAUD
 */
#define SERIAL_BAUD 115200

/*
 *  DEBUG PARAMETERS
 *
 *  De/comment any of the #define statements below to modify or change the behavior of the
 *  program for debugging purposes. 
 *
 *  The most useful are described below. Refer to the in-line comments for what the others do.
 *  CALIBRATE   -- moves all servos to their middle position, with arms as close to horizontal as they can.
 *  DEBUG     -- enables debug messages which print to the serial port. Specific messages can be chosen.
 *  RUN_PROG  -- enables only if CALIBRATE and DEBUG are disabled. Causes the program to run indefinitely.
 *
 *        You cannot enable RUN_PROG manually -- It only #defines if CALIBRATE and DEBUG are not #defined
 *
 *  By default, CALIBRATE and DEBUG should NOT be defined.
 *  This will define RUN_PROG and allow the program to execute indefinitely.
 *
 *  Heirarchy :
 *
 *  CALIBRATE >> DEBUG >> RUN_PROG
 */

//#define CALIBRATE // Disables program, pushes neutral PWM to all servos for calibration
//#define DEBUG     // Disables program, runs auxiliary debug routine

#ifdef CALIBRATE
  //#define SET_ANGLE_ALPHA // sets each servo to "neutral" resting angle for platform
  #define SET_ANGLE_CENTER  // else, sets each servo to 0-degree offset (horizontal)
#elif defined(DEBUG)
  #define TIME_TRIAL          // runs a series of timed runs to experimentally determine run-time
  //#define DEBUG_ACCEL         // enables custom input for accelerometer readings (for orientation)
  //#define DEBUG_SERVO         // prints debug messages for servos
  //#define DEBUG_PLAT          // prints debug messages for the mount points
  //#define DEBUG_LEGS          // prints debug messages for the leg lengths
  #define DEBUG_PID           // prints debug messages about PID controller
#else
  #define RUN_PROG
#endif

/*
 *  GYROSCOPE PARAMETERS
 *  
 *  Taken more-or-less directly from the LSM9DS1 IMU demo-program
 *  provided by SparkFun with the LSM9DS1 9-DOF IMU
 */

// I2C Addresses
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Declination values
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 16.0 // Declination (degrees) in Seattle, WA.

// Print settings
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

/*
 *  MATH CONSTANTS
 *
 *  Various numbers used for maths operations.
 *
 *  pi      -- ratio of the circumference of a circle to its diameter
 *  rad2deg   -- used for converting from radians to degrees
 *  deg2rad   -- used for converting from degrees to radians
 *  r_o     -- used for calculating the angles of servos when moving the platform
 *  microSec  -- used for converting micro-seconds to seconds
 */
const float pi      = 3.1415926536; // a few more digits than we should need
const float rad2deg   = 180/pi;     // convert radians to degrees
const float deg2rad   = pi/180;     // convert degrees to radians
const float r_o     = 1600/pi;    // constant used for converting servo angle into pulse width
const float microSec  = 0.000001;   // to convert micro-seconds to seconds

/*
 *  Instantiating the LSM9DS1 Inertial Measurement Unit (imu)
 *  that we are using for sensing platform tilt. It will be referenced
 *  at various points in the code, and configured during setup if needed.
 */
LSM9DS1 imu;
 
/*
 *  Instantiating six servos in an array for easy access. Each servo is
 *  represented as a Servo object, which contains methods for assigning
 *  the PWM associated with each servo.
 */
Servo servo[6];

/*
 *  STEWART PLATFORM CONSTANTS
 *
 *  These constants represent measurements of physical quantities about the Stewart platform
 *  that will affect how the program runs. 
 *
 *  Quantities are described below:
 *  B_RAD     -- distance (in inches) between the center of the base and the middle of the servo-arm. 
 *  P_RAD   -- distance (in inches) between the center of the platform and the mounting points.
 *          ((These two values should be identical for each arm))
 *  ARM_LEN   -- the length (in inches) of the arm of the servo.
 *  LEG_LEN   -- the length (in inches) of the rod that joins the servo to the platform.
 *  h_o     -- Height (in inches) when the platform is at rest.
 */
const float B_RAD = 4;
const float P_RAD = 2.5;
const float ARM_LEN = 1;
const float LEG_LEN = 14;
float h_o;

/*
 *  Angular quantities:
 *  These are used during setup to calculate where the base and platform connection points are 
 *  in the mathematical representation of our platform. Each servo is associated with a single 
 *  base point and a single platform point that are paired together. These quantities measure the
 *  angular difference between the points associated with servo's [0] and [1] for the base and platform
 *  (although one could use servo's [2] and [3], or [4] and [5] as well)
 *
 *  ANGLE_DIFF_PLAT   -- Angular displacement (in radians) between adjacent points on the platform
 *  ANGLE_DIFF_BASE   -- Angular displacement (in radians)between adjacent points on the base
 *  BETA[]        -- Angular displacement (in radians) between X-axis and servo arm
 *                ((used for converting from linear actuator to rotary actuator))
 *                ((each pair of servos gets a beta value))
 *  alpha_o       -- Angle of servo's when platform is at rest. Calculated during setup().
 */

// #defined because these are only referenced one time during set-up
#define   ANGLE_DIFF_PLAT   110*deg2rad
#define   ANGLE_DIFF_BASE   50*deg2rad
const float BETA[]      = {pi/3, 3*pi/3, 5*pi/3};
float alpha_o;

/*
 *  GLOBAL VARIABLES
 *
 *  bPts[6][3]        -- R^3 vectors mapping from center of base to the servo connection points
 *  pPts[6][3]        -- R^3 vectors mapping from center of (untilted) platform to mounting points
 *  platRMat[9]       -- 3x3 rotation matrix, used to tilt the mathematical model of our platform
 *              ((Stored as a 9x1 vector for compatability with MatrixMath))
 *  legVectors[6][3]  -- R^3 vectors which store the displacement between the i'th base point
 *                        and the i'th platform mount
 *  legLength[6]      -- scalar magnitudes which measure the length of the i'th leg (in inches)
 *  legLength2[6]     -- snap-shots of second most recent leglengths
 *  legLength3[6]     -- snap-shops of third most recent leglengths
 *  legVelocity[6]    -- velocities for the i'th leg (in inches / sec)
 *  legLengthRef[6]   -- reference value for leg lengths (in inches)
 *  legError[6]       -- integrals of error values (from beginning of time) for each leg
 *  tDisp[3]          -- platform displacement vector (XYZ) which controls the cartesian coordinates of the platform
 */
float bPts[6][3];
float pPts[6][3];
float platRMat[9];
float legVectors[6][3];
float legLength[6];
float legLength2[6];
float legLength3[6];
float legVelocity[6];
float legLengthRef[6];
float legError[6];
float tDisp[3];


/*
 *  PID Parameters
 *
 *  k_P -- proportional gain
 *  k_I -- integral gain
 *  k_D -- derivative gain
 */
#define PROPORTIONAL_GAIN   1
#define INTEGRAL_GAIN       0
#define DERIVATIVE_GAIN     0
float k_P, k_I, k_D;

/*
 *  SETUP METHOD
 *
 *  Initializes various aspects of our program to prepare it for continuous operation.
 *  This includes wiring the servos and configuring PWM ports, as well as allowing the
 *  IMU to power up sufficiently. The rotational matrix for the platform is initialized,
 *  and the mounting points for the base and platform are calculated using existing data.
 *  Finally, the default height of the platform, and angle of actuators is calculated.
 */
void setup() {
  // Initialize the serial port for debugging. Not optional, as IMU
  // also prints error messages to the serial port if a failure occurs.
  Serial.begin(SERIAL_BAUD);
  Serial.println("Initializing set-up...");

  /* 
   *  First, we calculate the points (in R^3) for the base and platform of our
   *  mathematical platform representation. This must be done before we can
   *  calculate some platform constants.
   */
  Serial.print("Calculating mount points...        ");
  calcMountPoints();
  Serial.println("DONE.");

  /*
   *  Next, we calculate some platform constants. Specifically, we are calculating
   *  the resting height, h_o; and the resting angle, alpha_o.
   *  
   *  Please refer to the Maths of the Stewart Platform for details on the math.
   */

  Serial.print("Calculating platform constants...  ");
  // calculate h_o (default platform height)
  h_o = sqrt(  ARM_LEN*ARM_LEN
             + LEG_LEN*LEG_LEN 
             - (bPts[0][0]*bPts[0][0] - pPts[0][0]*pPts[0][0])
             - (bPts[0][1]*bPts[0][1] - pPts[0][1]*pPts[0][1]));

  // set default height into platform displacement vector, tDisp
  tDisp[2] = h_o;

  // calc alpha_o (default servo angle)
  float L = 2*ARM_LEN*ARM_LEN;
  float M = 2*ARM_LEN*(pPts[0][0] - bPts[0][0]);
  float N = 2*ARM_LEN*(h_o + pPts[0][2]);
  alpha_o = asin(L/(M*M + N*N)) - atan(M/N);
  Serial.println("DONE.");

  /*
   *  Assign servo's to PWM pins.
   *  On Uno, these are pin 3, 5, 6, 9, 10, 11
   */
  Serial.print("Initializing servos...             ");
  servo[0].attach(3);     // attaches the servo on pin 3 to the servo object
    servo[1].attach(5);     // attaches the servo on pin 5 to the servo object
    servo[2].attach(6);     // attaches the servo on pin 6 to the servo object
    servo[3].attach(9);     // attaches the servo on pin 9 to the servo object
    servo[4].attach(10);    // attaches the servo on pin 10 to the servo object
    servo[5].attach(11);    // attaches the servo on pin 11 to the servo object
    delay(500);           // allow them to power up fully
    resetServos();        // reset to neutral position
    Serial.println("DONE.");

    /*
     *  Configure the IMU to gather acceleration data in near-real-time
     */
    Serial.print("Initializing IMU...                ");
    setupGyro();
    delay(300);
    Serial.println("DONE.");

    /*
     *  Initialize the rotation matrix for the platform.
     */
    float temp[] = {0,0,1};
    calcRMat(temp);

    /*
     *  Initialize PID coefficients
     */
    k_P = PROPORTIONAL_GAIN;
    k_I = INTEGRAL_GAIN;
    k_D = DERIVATIVE_GAIN;

    /*
     *  Initialize legLength[]
     */
    for (int i = 0; i < 6; i++) {
      legLength[i] = h_o;
    }
    

    Serial.println("Set-up complete.");
    Serial.println();

  #ifdef RUN_PROG
    Serial.println("Executing program.");
    Serial.println();
  #elif defined(DEBUG)
    Serial.println("Commencing debug routine.");
    Serial.println();
    debug_routine();
  #else
    Serial.println("Calibration mode. ");
    Serial.println("  Resting height: " + (String)(h_o) + " inches.");
    Serial.println("  Resting angle : " + (String)(alpha_o) + " radians.");
    Serial.println();
    #ifdef SET_ANGLE_ALPHA
      Serial.println("Servos set to neutral, resting position for platform.");
      float angles[6];
      for (int i = 0; i < 6; i++) {
        angles[i] = alpha_o;
      }
      calcPulseWidths(angles);
    #else
      Serial.println("Position servo arms to neutral position");
      Serial.println("by removing the central screw and aligning it manually.");
      Serial.println("When finished,  undefine CALIBRATE and reset the program.");
      resetServos();
    #endif
  #endif
}
 
 /*
  *  ARDUINO LOOP
  *
  *  If program is set to run, this loop executes indefinitely.
  *  Otherwise, it will not (for instance, if running a debug routine)
  */
void loop() {
    #ifdef RUN_PROG
      selfBalance();
    #endif
}
 
/*
 *  selfBalance()
 *
 *  Self-Balance Routine
 *
 *  High-level recipe for self-balancing on the platform.
 *  
 *  1. Read data from gyroscope, determine current tilt of platform.
 *  2. Based on current tilt, decide which way to tilt platform to make it level.
 *  3. Modify rotational matrix to adjust for tilt
 *  4. Using results from (3), calculate the length of each leg (assuming linear)
 *  5. Using results from (4), determine angle of each servo to achieve leg length
 *  6. Using results from (5), set servos to proper angle
 *  7. Repeat indefinitely
 *
 */
void selfBalance() {

  /*
   *  Calculate the reference trajectory 
   *
   */
  calcRef();

  /*
   *  Input to PID controller
   *
   */
  sp_PID(k_P, k_I, k_D);

  /*
   *  Adjust servo angles as needed
   *
   */
  // Calculate the angles required for each servo
  calcServoAngles();
}

/*
 *  AUXILIARY SET-UP METHODS
 *
 *  The following methods are used during the setup() routine and prepare
 *  the program for running by doing things such as initializing variables
 *  or hardware for usage.
 */

/*
 *  calcMountPoints(bPts, pPts)
 *
 *  Calculates the base and platform mounting points for the mathematical model
 *  of our Stewart platform. Runs ONE TIME during start-up. 
 *
 *  INPUTS:
 *  none
 *  
 *  OUTPUTS:
 *  bPts[6][3]  -- contains the base mount points
 *  pPts[6][3]  -- contains the platform mount points
 *
 *  To modify the angles used for this code, refer to the #define-d constants
 */
void calcMountPoints() {

  /*
   *  To generate the mount points, we will first take a unit vector pointing in the X-direction,
   *  rotate it by a small amount in each direction (which is different between the base and platform), 
   *  then rotate both vectors 120 degrees around the Z-axis. This creates three pairs of two vectors
   *  pointing from the center of the base (or platform) outwards to where the servo connection or 
   *  mount point is on the platform.
   *
   *  In summary, this is the recipe for the base:
   *  
   *  1. Scale vector to match RADIUS
   *  2. Offset unit vector by ANGLE/2
   *  3. Rotate by pi/3, copy vector
   *  4. Repeat 3 twice   --  this sets the "right" mount point for each pair
   *  5. Reset orientation to -ANGLE/2
   *  6. Repeat 3, 4    --  this sets the "left" mount point for each pair
   *  7. Repeat 1-6 for the platform 
   */

  // unit vector as a starting point
    float unit[] = {1, 0, 0};

    // temporary storage for matrix calculation
    float temp[3];

    /*
     *  BASE POINT CALCULATION
     */

    // scale unit vector to base radius
    scaleVec(unit, B_RAD);

  // temporary orientation vector
      // zero roll (X)
      // zero pitch (Y)
      // non-zero yaw (Z)
    float orientation[3] = {0, 0, ANGLE_DIFF_BASE/2 + pi/3};

    /*
     *  EVEN BASE MOUNTS -- Mounts for servos 0, 2, 4
     */
    for (int i = 0; i < 3; i++) {
      // adjust rotation angle
      orientation[2] -= 2*pi/3;

      // calculate rotation matrix for tempOrientation
      calcRMat(orientation);

      // rotate "unit" vector
      Matrix.Multiply(platRMat, unit, 3, 3, 1, temp);

      // store results in bPts
      for (int j = 0; j < 3; j++) {
        bPts[2*i][j] = temp[j];
      }
    }

    // re-adjust orientation for reflection
        orientation[2] = -ANGLE_DIFF_BASE/2 + pi/3;

    /*
     *  ODD BASE MOUNTS -- Mounts for servos 1, 3, 5
     */
    for (int i = 0; i < 3; i++) {
      // adjust rotation angle
      orientation[2] -= 2*pi/3;

      // calculate rotation matrix for tempOrientation
      calcRMat(orientation);

      // rotate "unit" vector
      Matrix.Multiply(platRMat, unit, 3, 3, 1, temp);

      // store results in bPts
      for (int j = 0; j < 3; j++) {
        bPts[2*i + 1][j] = temp[j];
      }
    }


    /*
     *  PLATFORM POINT CALCULATION
     */

    // First, reset and re-scale the basis "unit" vector
    unit[0] = 1;
    scaleVec(unit, P_RAD);

    // temporary orientation vector
    orientation[2] = ANGLE_DIFF_PLAT/2 + pi/3;

    /*
     *  EVEN PLATFORM MOUNTS -- Mounts for servos 0, 2, 4
     */ 
    for (int i = 0; i < 3; i++) {
      // adjust rotation angle
      orientation[2] -= 2*pi/3;

      // calculate rotation matrix for tempOrientation
      calcRMat(orientation);

      // rotate "unit" vector
      Matrix.Multiply(platRMat, unit, 3, 3, 1, temp);

      // store results in pPts
      for (int j = 0; j < 3; j++) {
        pPts[2*i][j] = temp[j];
      }
    }

    // re-adjust orientation vector for reflection
    orientation[2] = -ANGLE_DIFF_PLAT/2 + pi/3;

    /*
     *  ODD PLATFORM MOUNTS -- Mounts for servos 1, 3, 5
     */
    for (int i = 0; i < 3; i++) {
      // adjust rotation angle
      orientation[2] -= 2*pi/3;

      // calculate rotation matrix for tempOrientation
      calcRMat(orientation);

      // rotate "unit" vector
      Matrix.Multiply(platRMat, unit, 3, 3, 1, temp);

      // store results in pPts
      for (int j = 0; j < 3; j++) {
        pPts[2*i + 1][j] = temp[j];
      }
    }
}

/*
 *  AUXILIARY LOOP METHODS
 *
 *  The following methods comprise the high-level steps for running our program,
 *  and each one performs a different but equally important step during execution.
 */

/*
 *  calcRMat(orientation)
 *
 *  Calculates a rotation matrix around a standard set of cartesian axes, given
 *  the desired orientation. To do this, one would have to compute three separate
 *  rotation matrices, one for the Z-axis (yaw), Y-axis (pitch), and X-axis (roll)
 *  before multiplying them together in that order.
 * 
 *  Luckily, we can jump straight to the final result, since the general form
 *  for this matrix is always the same. 
 *  
 *  First, we compute short-hand notation for all of the trig values for each angle,
 *  then we can directly compute the resulting matrix.
 *
 *  INPUTS:
 *  orientation[3]  -- 3x1 vector storing the X/Y/Z (roll/pitch/yaw) orientation we want
 * 
 *  OUTPUTS:
 *  platRMat[9]   -- matrix is modified to match orientation[]
 */
void calcRMat(float orientation[]) {
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
    platRMat[0] = cy*cp;
    platRMat[1] = cy*sp*sr - sy*cr;
    platRMat[2] = sy*sr + cy*sp*cr;
    platRMat[3] = sy*cp;
    platRMat[4] = cy*cr + sy*sp*sr;
    platRMat[5] = sy*sp*cr - cy*sr;
    platRMat[6] = -sp;
    platRMat[7] = cp*sr;
    platRMat[8] = cp*cr;
}


/*
 *  calcRef()
 *
 *  Calculates the reference trajectory for each leg for use in our PID controller
 *
 *  First, it reads accelerometer data from the IMU to determine the reference tilt.
 *  This data is then used (in PID()) to determine the required lengths of each leg in order
 *  to position the upper platform in the desired orientation.
 *
 */
void calcRef() {
  // Calculate orientation from Gyroscope
  calcOrientation();

  // Calculate effective leg lengths 
  calcLegLengths();
}

/*
 *  sp_PID(k_P, k_I, k_D)
 *
 *  Stewart Platform PID Controller method which applies a PID controller
 *  to each individual leg in order to help it achieve it's optimal position
 */
void sp_PID(float k_P, float k_I, float k_D) {
  // Verify that all values are non-negative
  k_P = abs(k_P);
  k_I = abs(k_I);
  k_D = abs(k_D);

  /*
   *  PLACEHOLDER STORAGE
   *
   *  Temporary storage for 
   *  REF_POS   -- Reference position
   *  POS       -- Actual position
   *  VEL       -- Actual velocity
   *  ERR_OLD   -- Most recent recorded error signal
   *
   *  To be replaced, this is only here so that code compiles.
   */
//  float REF_POS[6];
//  float POS[6];
//  float ERR_OLD[6];
//  float VEL[6];

  #ifdef DEBUG_PID
    // optional debug routine for PID
    debugPID(k_P, k_I, k_D);
  #endif

  // PID action for each leg
  for (int i = 0; i < 6; i++) {
    // current error term
    legLengthRef[i] -= legLength[i];

    // integral error term
    legError[i] += legLengthRef[i];

    // derivative term
    getVel();

    // new position +=    proportional * error
    //                  + integral gain * integral error
    //                  - derivative gain * derivative (velocity)
    legLength[i] +=   k_P*legLengthRef[i] 
                    + k_I*legError[i] 
                    - k_D*legVelocity[i];
  }
}

/*
 *
 *  calculat the velocity vector for PID
 */
void getVel() {
  for (int i = 0; i < 6; i++) {
    legVelocity[i] = legLength2[i] - legLength3[i];
  }
}

/*
 *  calcOrientation()
 *
 *  Reads the value of the accelerometer in the IMU and uses that information to determine
 *  which way the platform should tilt, then computes the rotational matrix for that orientation.
 *  Simply put, our goal is to have the net-acceleration experienced by 
 *  the IMU (attached to platform) be entirely in the negative Z' axis. 
 *
 *  In the special case of no acceleration (such as the robot not moving), Z' will point 
 *  straight down (in the direction of gravity) and our platform should orient itself to match
 *  such that the normal vector on the platform is {0,0,1}.
 *
 *  After calculating the desired orientation, we have to perform a transformation of variables
 *  to convert from the set of axes on the IMU to the axes of the mathematical model of our platform.
 *
 *  For the purposes of this calculation, we assume a rotated set of axes, x'-y'-z' such that
 *  x' == z      roll'  == yaw
 *  y' == x      pitch' == roll
 *  z' == y      yaw'   == pitch
 *  thus, by calculating the pitch and yaw for our normal vector,
 *  we can transform it's pitch and yaw into roll and pitch for the platform
 *
 *  INPUTS
 *  None
 *
 *  OUTPUTS
 *  platRMat[9]   -- Updates platRMat[] using calculated orientation
 */
void calcOrientation() {
  float orientation[3];
  float accel[3];

  // Read acceleration from Gyroscope
  readGyro();
  #ifdef DEBUG_ACCEL
    accel[0] = 1;
    accel[1] = 1;
    accel[2] = 9;
  #else
    accel[0] = imu.ax;
    accel[1] = imu.ay;
    accel[2] = imu.az;
  #endif

  // Normalize the acceleration vector
  float magnitude = calcVecMag(accel);
  float unit[3];
  for (int i = 0; i < 3; i++) {
    unit[i] = accel[i]/magnitude;
  }

  /*
   *  For the purposes of this calculation, we assume a rotated set of axes, x'-y'-z' such that
   *  x' == z      roll'  == yaw
   *  y' == x      pitch' == roll
   *  z' == y      yaw'   == pitch
   *  thus, by calculating the pitch and yaw for our normal vector,
   *  we can transform it's pitch and yaw into roll and pitch for the platform
   */

  // PLATFORM ROLL  --- NORM YAW
    orientation[0] = asin(-unit[1]);

    // PLATFORM PITCH --- NORM ROLL
    orientation[1] = atan2(unit[0], unit[2]);

    // PLATFORM YAW   --- NORM PITCH
    // platform YAW is immaterial. This value can theoretically be set to anything
    // but to minimize the risk of problems, we'll leave it at zero for now.
    orientation[2] = 0;


    // MAXIMUM TILT CONDITION, IF DEFINED.
    #ifdef MAX_TILT
      for (int i = 0; i < 3; i++) {
        if (orientation[i] > MAX_TILT) {
          orientation[i] = MAX_TILT;
        } else if (orientation[i] < -MAX_TILT) {
          orientation[i] = -MAX_TILT;
        }
      }
    #endif

    // ARBITRARY TILT CONDITION
    #ifdef DEBUG_SERVO_OLD
      orientation[0] = 0;
      orientation[1] = 0;
      orientation[2] = 0;
    #endif

    // Calculate rotational matrix
  calcRMat(orientation);

  #ifdef DEBUG_SERVO
    Matrix.Print(platRMat, 3, 3, "Rotation Matrix");
  #endif
}

/*
 *  calcLegLengths()
 *
 *  Using the information about the base points, platform points, and the rotation matrix of the platform,
 *  as well as the desired height and position of the platform, we can calculate the lengths of each actuator
 *  connected to the platform. To do this, we use a reverse-kinematic model which works backwards:
 *
 *  Instead of computing where the platform would be based on the lengths of the actuators, we figure out what
 *  position we want the platform to be and then compute the lengths for each actuator that will achieve this.
 *
 *  This is where it all comes together.
 *
 *  We use the rotation matrix we computed earlier, platRMat, and multiply it by the platform mount vectors, pPts.
 *  This creates a copy of those vectors that are now rotated in some way. We then add the result of this to our 
 *  translation vector, pDisp, to get the vector displacement between the center of the base and the platform mounts.
 *
 *  At this stage, we can determine the lengths of the actuators by subtracting the base point vectors from each of
 *  the displacement vectors calculated above.
 *
 *  To save on space, this operation is done one leg at a time.
 *
 *  The recipe for this is as follows:
 *      - after calculating desired platRMat
 *  For each leg:
 *  1. left-multiply platRMat with pPts[i][:], store the results in temporary memory
 *  2. translate the results of this operation by a specified amount (same amount for all legs)
 *  3. for the i'th leg in this matrix, subtract the i'th base points in bPts
 */
void calcLegLengths() {
  // Storage for rotated pPts[][]
  float temp[3];
  float tempOut[3];

  #ifdef DEBUG_LEGS
    printMat3(pPts, 6, "Platform Points");
    float orientation[] = {0.1, 0.1, 0};
    calcRMat(orientation);
    Matrix.Print(platRMat, 3, 3, "Rotation Matrix");
    Matrix.Print(tDisp, 3, 1, "tDisp");
    printMat3(pPts, 6, "pPts");
  #endif

  // for each leg
  for (int i = 0; i < 6; i++) {
    // grab the i'th pPts
    for (int j = 0; j < 2; j++) {
      temp[j] = pPts[i][j];
    }
    /*
     *  Manually assigning temp[2] = 0
     *
     *  For pPts, the z component (or pPts[:][2]) is always 0.
     *  We encountered a bug that would make this value very large
     *  and decided to manually force it to 0. Hacky, but effective.
     */
    temp[2] = 0;

    // Left-multiply pPts[i][:] by platRMat, store in tempOut
    Matrix.Multiply(platRMat, temp, 3, 3, 1, tempOut);

    // now we sum the following:
    //  + X/Y/Z platform displacement
    //  + rotated mounting point vector for each leg
    //  - distance between base center and servo axis

    for (int j = 0; j < 3; j++) {
      tempOut[j] += tDisp[j] - bPts[i][j];
      legVectors[i][j] = tempOut[j];
    }

    /* 
     *  legVectors[i][0] = tDisp[0] + tempOut[0] - bPts[i][0];  // X
     *  legVectors[i][1] = tDisp[1] + tempOut[1] - bPts[i][1];  // Y
     *  legVectors[i][2] = tDisp[2] + tempOut[2] - bPts[i][2];  // Z
     */

    // calculate and store the magnitude for each respective leg
    legLengthRef[i] = calcVecMag(tempOut);
  }

  #ifdef DEBUG_LEGS
    printMat3(legVectors, 6, "Leg vectors");
    float tempVec[3];
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 3; j++) {
        tempVec[j] = legVectors[i][j];
      }
      Serial.println("Leg " + (String)(i) + " : " + (String)(calcVecMag(tempVec)));
    }
  #endif
}

/*
 *  calcServoAngles()
 *
 *  Calculates the angles required for each servo, based on the required leg-lengths.
 *  After calculating these angles, automatically updates servos to move to those angles.
 *
 *  INPUTS
 *
 *
 *  OUTPUTS
 *  Servos should automatically update to new positions
 */
void calcServoAngles() {
  // initialize local storage
  float angles[6];
  float temp;

  /*
   *  Operation for each leg
   *
   */
  for (int i = 0; i < 6; i++) {
    // First, grab a copy of the i'th leg length
    temp = legLength[i];

    // Calculate parameters L, M, and N
    // see Maths of the Stewart Platform for reference
    float L = temp*temp - (LEG_LEN*LEG_LEN - ARM_LEN*ARM_LEN);
    float M = 2*ARM_LEN*(legVectors[i][2]);
    float N = 2*ARM_LEN*(cos(BETA[i/2])*legVectors[i][0] + sin(BETA[i/2])*legVectors[i][1]);
                            // (( i/2 because each pair of motors uses the same BETA ))
                            
    // Calculate the servo angle using these parameters
    /*
     *  So, for some reason, these two conditionals produce the same result.
     *  I remember there being a difference for even/odd servos at this stage,
     *  but I don't remember off-hand what it was... Probably in that paper.
     *
     *  In any case, this should be tested again.
     */
    if (i%2 == 0) {
      angles[i] = asin(L/sqrt(M*M + N*N)) - atan(N/M);
    } else {
      angles[i] = /* pi - */ (asin(L/sqrt(M*M + N*N)) - atan(N/M));
    }
  }

  // Debug messages for servo angles
  #ifdef DEBUG_SERVO
    Serial.println("Servo angles");
    for (int i = 0; i < 6; i++) {
      Serial.println("Servo " + (String)(i) + ":  " + (String)(angles[i]) + " radians");
    }
    Serial.println();
  #endif

  // Safety condition: restricting servo angle to limit destructive forces in case of failure.
  #ifdef MAX_SERVO_ANGLE
      if (servoAngles[i] > MAX_SERVO_ANGLE) {
        servoAngles[i] = MAX_SERVO_ANGLE;
      } else if (servoAngles[i] < -MAX_SERVO_ANGLE) {
        servoAngles[i] = -MAX_SERVO_ANGLE;
      }
    #endif

  // Use angle data to calculate pulse widths
    calcPulseWidths(angles);
}

/*
 *  calcPulseWidths()
 *
 *  Using the angle data, calculates the pulse-width required for each of the servos
 *  and then sends those signals to the servos to control them.
 *
 *  INPUTS
 *  angles[]  -- six angles corresponding to where each servo should rotate
 *
 *  OUTPUTS
 *  Servos should automatically update to new positions
 *
 */
void calcPulseWidths(float angles[]) {
  float pWidth[6];
  for (int i = 0; i < 3; i++) {
    // EVEN SERVOS (0, 2, 4)
    pWidth[2*i    ] = (int) (DEFAULT_PULSE_WIDTH + ((angles[2*i    ] - alpha_o))*r_o);
    
    // ODD SERVOS (1, 3, 5)
    pWidth[2*i + 1] = (int) (DEFAULT_PULSE_WIDTH - ((angles[2*i + 1] - alpha_o))*r_o);
  }

  // Debug messages for pulse widths
  #ifdef DEBUG_SERVO
    for (int i = 0; i < 6; i++) {
      Serial.println("PWM" + (String)(i) + " :  " + (String)(pWidth[i]) + " microseconds");
    }
    Serial.println();
  #endif

  setPulseWidths(pWidth);
}

/*
 *  setPulseWidths(pWidth)
 *
 *  Assigns the specified pulse widths to each servo PWM
 *
 *  INPUTS
 *  pWidth    -- pulse widths (in microseconds) for each of the servos
 *
 *  OUTPUTS
 *  all servo PWMs should be set to their position
 */
void setPulseWidths(float pWidth[]) {
  for (int i = 0; i < 6; i++) {
    servo[i].writeMicroseconds(pWidth[i]);
  }
}



  /*
   *  HELPER METHODS
   *
   *  Methods for various small tasks, mostly dealing with small or repeated math,
   *  such as scaling a vector, calculating a cross-product, or printing the contents
   *  of a Nx3 matrix to the serial port for debugging.
   */


/*
 *  calcVecMag(v)
 *
 *  Calculates the magnitude of a 3-dimensional vector
 *
 *  INPUTS
 *  v[]  -- 3-dimensional vector of floats
 *  
 *  OUTPUTS
 *  mag     -- magnitude of the vector
 */
// Calculates (and returns) the magnitude of a vector in R3
float calcVecMag(float v[]) {
  float magnitude = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  return magnitude;
}

/*
 *  resetServos()
 *
 *  Sets each servo to return to its neutral middle position by
 *  over-riding whatever the PWM is doing and reseting it to
 *  approx. 1500uSec.
 *
 *  INPUTS:
 *  none
 *
 *  OUTPUTS:
 *  all servos set to neutral
 */
void resetServos() {
  for (int i = 0; i < 6; i++) {
    servo[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
  }
}

/*
 *  scaleVec(vector, scale)
 *
 *  Scales a vector by a specified amount.
 *  
 *  INPUTS:
 *  vector[]  -- vector to be scaled
 *  scale     -- scaling factor
 *
 *  OUTPUTS:
 *  vector[]  -- vector is scaled
 */
void scaleVec(float vector[], float scale) {
  for (int i = 0; i < 3; i++) {
    vector[i] = scale*vector[i];
  }
}

/*
 *  printMat3(mat[][3], size, name)
 *
 *  Prints the contents of an Nx3 matrix to the serial port
 *
 *  INPUTS:
 *  mat[][3]  -- matrix to be printed
 *  size    -- number of rows in the matrix to print
 *  name    -- label to identify the matrix
 */
void printMat3(float mat[][3], int size, String name) {
    Serial.println(name);
    for (int i = 0; i < size; i++){
        for (int j = 0; j < 3; j++){
            Serial.print(mat[i][j]);
            Serial.print("\t");
        }
        Serial.println();
    }
}

/*
 *  printMat(mat, name)
 * 
 *  Same as printMat3() but takes a column-vector of any length
 *  for it's input. Only prints as a column.
 * 
 */
void printMat(float mat[], int size, String name) {
  Serial.println(name);
  for (int i = 0; i < size; i++) {
    Serial.println("Index " + (String)(i) + " :  " + (String)(mat[i]));
  }
  Serial.println();
}

/*
 *  GYROSCOPE AUXILIARY METHODS
 *
 *  Some methods taken from LSM9DS1 stock library
 */

/*
 *  setupGyro()
 *
 *  configures the IMU for use
 *  (taken from IMU library)
 */
 void setupGyro() 
{
  
  Serial.begin(SERIAL_BAUD);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

/*
 *  readGyro()
 *
 *  reads from the IMU
 */
void readGyro() {
    // read the acceleration values from the gyroscope
    imu.readAccel();
}


#ifdef DEBUG

  /*
   *  AUXILIARY DEBUG METHODS
   *
   *  The following methods are used for debugging purposes.
   */

  /*
   *  debug_routine()
   *
   *  Executes the program in debug-mode.
   *  Allows the user to run the program a fixed number of times,
   *  for the purposes of testing individual routines or 
   *  for timing the overall run-time of the algorithm
   */
  void debug_routine() {
    // Print debug messages to console
    printDebug();
    Serial.println();

    #ifdef TIME_TRIAL
      timeTrial(FALSE);
    #else
      selfBalance();
    #endif
  }


  /*
   *  timeTrial(run_one_time)
   *
   *  Runs a set of timed trials for experimentally determining the run-time of the program
   *  If run_one_time == TRUE, program runs exactly once. Else, it runs ten times and averages.
   */
  void timeTrial(bool run_one_time) {
      int count;
      if (run_one_time) {
        count = 1;
      } else {
        count = 10;
      }

      // Should it print for every cycle, or only the average?
      bool print_every_time = TRUE;

      // used to measure time from system
      long start, end;
      float delta, deltaSum, freq;

      /*
       *  Time-trial
       *  
       *  Runs through the program a set number of times, each time taking a measurement
       *  of the time required to run the program once. At the end, these times are
       *  averaged to determine the expected run-time and run-frequency of the algorithm
       */
      Serial.println("Beginning time-trial...");
      for (int i = 0; i < count; i++) {
        // measure start and end times
        start = micros();
        selfBalance();
        end = micros();
        delta = (end - start);
        deltaSum += delta;
        freq = 1.0 / (delta*microSec);
        if (print_every_time) {
          Serial.println("Trial " + (String)(i) + ":\t" + (String)(delta) + "usec,\t" + (String)(freq) + " Hz.");
        }
      }

      // Calculate average values and print to Serial
      deltaSum /= count;
      freq = 1.0 / (deltaSum*microSec);
      Serial.println();
      Serial.println("Average performance characteristics:");
      Serial.println("  Run-time: \t" + (String)(deltaSum) + " microseconds.");
      Serial.println("  Frequency:\t" + (String)(freq) + "Hz.");
      Serial.println();
  }

  /*
   *  printDebug()
   *
   *  Prints various debug messages to the serial port.
   *
   *  To enable this method, first #define DEBUG and then #define the relevant options
   */
  void printDebug() {
    //vectorDebug();
    #ifdef DEBUG_PLAT
      printMat3(pPts, 6, "Platform Mount Points:");
      printMat3(bPts, 6, "Base Mount Points:");
    #endif
  }

  /*
   *  debugPID(k_P, k_I, k_D)
   *
   *  Prints debug messages about PID controller to console
   */
  void debugPID(float k_P, float k_I, float k_D) {
        Serial.println("PID parameters:");
    bool hasP, hasI, hasD;
    Serial.println("Proportional gain   : " + (String)(k_P));
    Serial.println("Integral gain       : " + (String)(k_I));
    Serial.println("Derivative gain     : " + (String)(k_D));
    Serial.print("Controller type: ");
    if (k_P == 0 && k_I == 0 && k_D == 0) {

    }
    else {
      if (k_P > 0.0) {
        Serial.print("P");
      }
      if (k_I > 0.0) {
        Serial.print("I");
      }
      if (k_D > 0.0) {
        Serial.print("D");
      }
      Serial.println(" controller");
    }
  }

#endif


