/* variables.h
 *  Header file that contains all the variable declarations for the 
 *  MARS CONTROL program.
 *  
 *  Sivakumar Balasubramanian.
 */

#include "Arduino.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include "SoftwareSerial.h"
#include "HX711_ADC.h"
#include "Wire.h"
#include <MPU6050_light.h>

// Define the PIN numbers
// #define IMU_DEBUG
#define LASER_PIN                 28
#define SAFETY_PIN                40
#define MARS_BUTTON               20
#define MOTOR_PWM                 37
#define MOTOR_ENABLE              38
#define MOTOR_DIR                 39
#define MOTOR_T2I                 3.35    // A / Nm
#define MOTOR_TORQ_CONST          3.7153  // Nm / A

// Robot Encoders
#define ENC1A                     2//4
#define ENC1B                     3//5
#define ENC2A                     4//11
#define ENC2B                     5//10
#define ENC3A                     6//12
#define ENC3B                     7//13
#define ENC4A                     8//36
#define ENC4B                     9//35

// Encoder parameters     
#define ENC1MAXCOUNT              4 * 4096 * 53
#define ENC1COUNT2DEG             0.25f * 0.00166f
#define ENC2MAXCOUNT              4 * 1024
#define ENC2COUNT2DEG             0.25f * 0.3515625f
#define ENC3MAXCOUNT              4 * 1024
#define ENC3COUNT2DEG             0.25f * 0.3515625f
#define ENC4MAXCOUNT              4 * 1024
#define ENC4COUNT2DEG             0.25f * 0.3515625f

// Load cells
#define LOADCELL1_DOUT_PIN        14//15
#define LOADCELL1_SCK_PIN         15//16
#define LOADCELL2_DOUT_PIN        16//17
#define LOADCELL2_SCK_PIN         17//18
#define LOACELL_CALIB_FACTOR      12866.6
#define ARM_REST_WEIGHT           1.46    // New

// Limb type
#define NOLIMB                    0x00
#define RIGHT                     0x01
#define LEFT                      0x02

// Control type
#define NONE                      0x00
#define POSITION                  0x01

// Out data type
#define VERSION                   0x00
#define SENSORSTREAM              0x01
#define CONTROLPARAM              0x02
#define DIAGNOSTICS               0x03

// In data type
#define GET_VERSION               0x00
#define RESET_PACKETNO            0x01
#define SET_LIMB                  0x02
#define CALIBRATE                 0x03
#define START_STREAM              0x04
#define STOP_STREAM               0x05
#define SET_CONTROL_TYPE          0x06
#define SET_CONTROL_TARGET        0x07
#define SET_DIAGNOSTICS           0x08
#define HEARTBEAT                 0x80

// Control Law Related Definitions
#define INVALID_TARGET            999.0
#define INTEGRATOR_LIMIT          4.0
#define POS_ERROR_CAP             10.0    // Degrees
#define POS_ERROR_DIFF_CAP        2.0     // Degrees
#define POS_ERROR_DIFF_LIMIT      30      // Degrees
#define PWMRESOLN                 12      // This has been changed from 8. Suggestions from Aravind.
#define MINPWM                    410     // 10% of 4095
#define MAXPWM                    3686    // 90% of 4095
#define MAXDELPWM                 40      // Changed from 5
#define MAX_CURRENT               10
#define POS_CTRL_DBAND            0
#define POSITION_TARGET_MIN       -120    // Degrees
#define POSITION_TARGET_MAX       20      // Degrees
#define POSITION_RATE_LIMIT       5       // Degrees / sec
#define SAFETY_DAMP_VEL_TH        10.0    // deg / sec
#define SAFETY_DAMP_VALUE         20.0    // PWM / (deg / sec)

// Error types 
#define NOHEARTBEAT               0x0001
#define ANG1MISMATCHERR           0x0002
#define ANG234MISMATCHERR         0x0004
#define ANG1JUMPERR               0x0008
#define ANG234JUMPERR             0x0010
#define ANG1LIMITERR              0x0020
#define ANG234LIMITERR            0x0040

// Safety timer thresholds
#define TARGET_SET_BACKOUT        2000     // millisec

// Kinematic calib status
#define NOCALIB                   0x00
#define YESCALIB                  0x01

// Recent command status
#define COMMAND_NONE              0x00
#define COMMAND_SUCCESS           0x01
#define COMMAND_FAIL              0x02

// Button bounce threshold
#define BOUNCE_THRESHOLD          5

// IMU offsets
#define IMU1PITCHOFFSET           0.00    // Radians
#define IMU1ROLLOFFSET            0.00    // Radians
#define IMU2PITCHOFFSET           0.00    // Radians
#define IMU2ROLLOFFSET            0.00    // Radians
#define IMU3PITCHOFFSET           0.00    // Radians
#define IMU3ROLLOFFSET            0.00    // Radians

// Calibration angle limits
#define CALIB_IMU_ANGLE_MIN       -50.0
#define CALIB_IMU_ANGLE_MAX       +50.0

// Angle limits.
#define ANGLE1_MIN_LIMIT          -100.0  // Degrees
#define ANGLE1_MAX_LIMIT          +010.0  // Degrees
#define ANGLE2_MIN_LIMIT          -045.0  // Degrees
#define ANGLE2_MAX_LIMIT          +045.0  // Degrees
#define ANGLE3_MIN_LIMIT          -160.0  // Degrees
#define ANGLE3_MAX_LIMIT          +160.0  // Degrees
#define ANGLE4_MIN_LIMIT          -300.0  // Degrees
#define ANGLE4_MAX_LIMIT          +300.0  // Degrees
#define DELTA_FOR_ERROR           +005.0  // Degrees

// Angle sensor error thresholds
#define ENC_IMU_MISMATCH_ERR_TH   20.0    // Degrees
#define ENC_IMU_MISMATCH_TIME_TH  0.5     // Seconds
#define ANG_JUMP_ERROR            3.5     // Degrees
#define ENC_LIM_MISMATCH_TIME_TH  0.1     // Seconds

// Control related constant
#define MARS_GRAV_COMP_ADJUST     1.5
#define MIN_TARGET_DUR            2.0     // Seconds

// MARS robot parameters.
#define L1                        0.475   // meters
#define L2                        0.291   // meters

// Heart beat related variable
#define MAX_HBEAT_INTERVAL        5.0 // Seconds

// Radians to degree conversion
#define RAD2DEG(x)                180.0 * x / PI
#define DEG2RAD(x)                PI * x / 180.0

//    Sigmoid function
#define SIGMOID(x)                1 / (1 + exp(-5 * x))

// Sign function
#define SIGN(x)                   x >= 0 ? 1 : -1

// Version and device ID.
const char* fwVersion = "h1.0.1";
const char* deviceId  = "MARS-HOMER";
const char* compileDate = __DATE__ " " __TIME__;

// MARS Gravity Compensation Parameters
const float marsGCParam[] = {
  -0.00423728,
  -1.18992423,
   3.74296361,
   1.19731609,
   1.99763005,
   0.24580473,
  -0.16474236,
   1.30690941
};

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// Program status
byte streamType = SENSORSTREAM;
byte cmdStatus = COMMAND_NONE;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
uint16union_t deviceError;
bool fatalError = false;
// Encoder-IMU mismathc timers.
byte enc1ImuMatchCount  = 0; 
byte enc1ImuMismatchCount  = 0;
// Encoder limits timer.
byte encImuMismatchErrTimer;
byte enc1LimitErrTimer;
byte enc234LimitErrTimer; 

// Packet Counter.
uint16union_t packetNumber;

// run time
unsigned long startTime;
ulongunion_t runTime;
unsigned long currMilliCount;
unsigned long prevMilliCount;
float delTime;

// Current limb
byte currLimb;

// MARS and Calibration buttons states.
int8_t marsBounceCount = 0;
byte marsButton = 0;
int8_t calibBounceCount = 0;
byte calibButton = 0;
byte devButtons = 0;

// Control related variables.
float target;
Buffer actual;
Buffer desired;
Buffer control;

// Joint kinematics related variables.
Encoder angle1(ENC1A, ENC1B);
long _enccount1;
Encoder angle2(ENC2A, ENC2B);
long _enccount2;
Encoder angle3(ENC3A, ENC3B);
long _enccount3;
Encoder angle4(ENC4A, ENC4B);
long _enccount4;
float limbAngleScale;
float limbControlScale; 
float theta1, theta2, theta3, theta4;
float theta1Prev, theta2Prev, theta3Prev, theta4Prev;
float theta1r, theta2r, theta3r, theta4r;
float omega1, omega2, omega3, omega4;

// Cosine and sine terms of the individual angles.
float cos1, cos2, cos3, cos4;
float sin1, sin2, sin3, sin4;

// Calibration related variables.
float theta1Offset, theta2Offset, theta3Offset, theta4Offset;

// IMU angles and their codes.
MPU6050 mpu(Wire);
MPU6050 mpu2(Wire1);
MPU6050 mpu3(Wire1);
float imuTheta1, imuTheta2, imuTheta3, imuTheta4;

// Endpoint kinematics of the robot.
float xEp, yEp, zEp;

// Endpoint force.
float epForce;
float torque, torquePrev;
float dTorque;

// Controller gains
float pcKp = 1.5;
float pcKd = 7.8;
float pcKi = 0;
// Control related buffers
float err;
float errdiff;
float errsum;
float marsGCTorque;
// Desired target related variables.
float strtPos, strtTime, initTime, tgtDur;
// Control Scale
float ctrlScale = 1.0;
// Control Bound
float ctrlBound = 1.0;

// Temporary variable for parsing incoming data.
float tempArray[8];

// Safety Time Flags.
unsigned long targetSetTime;

// Read stream interval timer
IntervalTimer readStream;

// Bluetooth software serial.
SoftwareSerial bt(0,1);

// Loadcell amplifiers.
HX711_ADC scale1(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
HX711_ADC scale2(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

// Serial Reader object
SerialReader serReader;
// Out data buffer
OutDataBuffer4Float outPayload;
