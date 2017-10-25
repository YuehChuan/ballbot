#define ENABLE_MOUSE//17millis
#define ENABLE_MOTOR//0.2millis
//#define ENABLE_LED
#define ENABLE_MATLAB
//#define ENABLE_GYRO//4millis
#define ENABLE_RUNTIME

#define PRINT_MOUSE
#define PRINT_MOTOR
//#define PRINT_LED
#define PRINT_MATLAB
//#define PRINT_GYRO
#define PRINT_RUNTIME


#ifndef cbi
#define cbi(sfr, Bit) (_SFR_BYTE(sfr) &= ~_BV(Bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#define displayT 100//millis if too fast matlab will delay
#define PIDSampleTime 20//millis

// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include "Arduino.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>
#include "LSM9DS0_AHRS.h"
#include <LedControlMS.h>


#include <ps2.h>
PS2 mouse1(23, 25);
PS2 mouse2(31, 29);
PS2 mouse3(35, 33);
long mouse_x[3];
long mouse_y[3];
char mouse_vx[3];
char mouse_vy[3];
double mousexyz[3];
double oldmousexyz[3];
double mouseV[3];

#include <PID_v1.h>

#define BRAKEVCC 0
#define CW 1
#define CCW 2
#define BRAKEGND 3
#define CS_THRESHOLD 60   // Definition of safety current (Check: "1.3 Monster Shield Example").

//#define angle 0.95539323254169601040802832667067

#define deg120 2.0/3.0*PI
#define deg90 1.0/2.0*PI
#define a_a 35.3*PI/180
#define b_a 0*PI/180
#define a_s 43.9*PI/180
#define b_s 180.0*PI/180

#define rb=242.0/2
#define rw=37.5/2
#define ro=34.0/2

#define FILTER_A 0.15

double Value[3];

//#define alpha 54.74/180*PI
//#define beta 60.0/180*PI
//#define alphaMouse 48.0/180*PI
//#define betaMouse 60.0/180*PI

//Define Variables we'll be connecting to
double Setpoint[3], Input[3], Output[3];

//Define the aggressive and conservative Tuning Parameters
double aggKp = 100.0, aggKi = 0.0, aggKd = 0.0;
double consKp = 1.8, consKi = 0.2, consKd = 0.0;

//Specify the links and initial tuning parameters
PID myPID0(&Input[0], &Output[0], &Setpoint[0], consKp, consKi, consKd, DIRECT);
PID myPID1(&Input[1], &Output[1], &Setpoint[1], consKp, consKi, consKd, DIRECT);
PID myPID2(&Input[2], &Output[2], &Setpoint[2], consKp, consKi, consKd, DIRECT);



//Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
LedControl lc = LedControl(22, 26, 24, 2);

unsigned long nowtime, oldtime, dtime, rate;
double startTime, realTime;

//enum {gyro, runt, led, mouse, motor};
//const bool displayOnConst[5] = {0, 0, 1, 0, 0};
//bool displayOn[5] = {0};

//double read_x, read_y, read_z;
double setMotorSpeed[3];
double setMotorSpeedinRad[3];

double read_p = consKp;
double read_i = consKi;
double read_d = consKd;



int inApin[3] = {42, 36, 30}; // INA: Clockwise Direction Motor0 and Motor1 (Check:"1.2 Hardware Monster Motor Shield").
int inBpin[3] = {40, 48, 28}; // INB: Counterlockwise Direction Motor0 and Motor1 (Check: "1.2 Hardware Monster Motor Shield").
int pwmpin[3] = {46, 44, 45};          // PWM's input
int cspin[3] = {A8, A9, A11};            // Current's sensor input
int enpin[3] = {38, 34, 32};
int mspeed[3];

#define motor1 OCR5A
#define motor2 OCR5C
#define motor3 OCR5B

//bool matlab=1;

void setup() {

  Serial.begin(115200); // Start serial at 38400 bps
  Serial.setTimeout(10);
  Serial1.begin(115200);

//Serial.println(deg120);
//Serial.println(deg90);
//Serial.println(a_a);
//Serial.println(b_a);
//delay(10000);
  
#ifdef ENABLE_LED
  LED_setup();
#endif
#ifdef ENABLE_MOTOR
  motor_setup();
#endif
#ifdef ENABLE_GYRO
  ahrs_set();
#endif
#ifdef ENABLE_MOUSE
  mouse_init();
#endif
  PID_setup();
#ifdef  ENABLE_MATLAB
  //establishContact();  // send a byte to establish contact until receiver responds
  startTime = millis();
#endif
}

void loop() {
  if (millis()-startTime > 1000)
  {
    //setMotorSpeedinRad[2]=1;
    
  }
  if (millis()-startTime > 10000)
  {
    //setMotorSpeedinRad[0]=0;
    //setMotorSpeedinRad[1]=0;
    //setMotorSpeedinRad[2]=0;
  }
  
  
  if ((millis() - rate) > displayT)
  {
    rate = millis();
#ifdef PRINT_MOUSE
    mousePrint();
#endif
#ifdef PRINT_MOTOR
    motorPrint();
#endif
#ifdef PRINT_LED
    LED_control();
#endif
#ifdef PRINT_MATLAB
    matlabPrint();
#endif
#ifdef PRINT_GYRO
    printGyro();
#endif
#ifdef PRINT_RUNTIME
    printRunTime();
#endif
  }

#ifdef ENABLE_MOUSE
  mouse_read();
#endif
#ifdef ENABLE_RUNTIME
  runtime();
#endif
#ifdef ENABLE_GYRO
  ahrs();
#endif

#ifdef ENABLE_MOTOR
  //Input[0] = -roll;
  //Input[1] = pitch;
  //Input[2] = yaw;
  for (int i = 0; i < 3; i++)
  {
    Setpoint[i]=setMotorSpeedinRad[i]*57.2958*1.4236;
    Input[i] = mouseV[i]/7200*360*59;//deg
  }

  myPID0.Compute();
  myPID1.Compute();
  myPID2.Compute();

  for (int i = 0; i < 3; i++)
  {
    //setMotorSpeed[i] = Output[i];
    //setMotorSpeed[i] = Setpoint[i];
  }

  motor_vector();
#endif
}
