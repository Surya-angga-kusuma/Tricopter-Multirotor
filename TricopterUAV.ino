//========== Tricopter Program ==========
//Libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"
#include "HMC5883L.h"

//********** compass HMC5883L **********
HMC5883L compass;

float headingDegrees;
float heading, setHeading;
float heading_reference;
float heading_control;
float acc_x, acc_y;
float compensateRoll, compensatePitch;
float cosComRoll, sinComRoll, cosComPitch, sinComPitch;
float Yh, Xh, Ymag_correct, Xmag_correct;


//********** mpu 6050 **********
MPU6050 mpu;
MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;

float G_Dt = 0.005;

double rad_yaw, rad_pitch, rad_roll;
double roll_kalman, pitch_kalman, yaw_kalman;
double accum_roll = 0;
double accum_pitch = 0;
double k_acc = 0;
double k_gps = 0;
double yaw_deg;
double pitch_deg;
double roll_deg;
double pitch_deg_previous, roll_deg_previous;
float gyro_roll_input  = 0.0;
float gyro_pitch_input = 0.0;
float gyro_yaw_input  = 0.0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_ACCELGYRO

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
boolean interruptLock = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

//********** brushless motor **********
#define MOTOR1 21
#define MOTOR2 20
#define MOTOR3 22

Servo brushless1;
Servo brushless2;
Servo brushless3;

int motor1;
int motor2;
int motor3;

//********** servo motor **********
Servo myservo;
int servoAngleInit = 49;
int servo;
float yawControlServo;
float ControlYaw_Filter;

//********** remote channels **********
volatile int channel1         = 0;
volatile int roll_channel     = 0;
volatile int channel2         = 0;
volatile int throttle_channel = 0;
volatile int channel3         = 0;
volatile int pitch_channel    = 0;
volatile int channel4         = 0;
volatile int yaw_channel      = 0;
volatile int channel5         = 0;
volatile int ch5_channel      = 0;
volatile int channel6         = 0;
volatile int ch6_channel      = 0;

//********** inputs variable **********
int ch5, throttle, throttle_input, heading_mode, heading_mode1;
int roll_input, pitch_input, yaw_input;
unsigned long timeProgram, previousTimeProgram;
unsigned long timeServo, previousTimeServo;
unsigned long deltaTime_control_rate, lastTime_control_rate;
char inChar;

//********** controller **********
float gyroRoll, gyroPitch, gyroYaw;
float rollSetpoint, pitchSetpoint, yawSetpoint;
float rollLevel, pitchLevel;
float errorRoll, errorPitch, errorYaw;
float ControllerIRoll, ControllerDRoll, ControllerDRollLast;
float ControllerIPitch, ControllerDPitch, ControllerDPitchLast;
float ControllerIYaw, ControllerDYaw, ControllerDYawLast;
float ControlRoll, ControlPitch, ControlYaw;
float roll_rate_input    = 0.0;
float pitch_rate_input   = 0.0;
float roll_level_adjust  = 0.0;
float pitch_level_adjust = 0.0;

//////////////terbang///////////
float GainProll  = 3.09;
float GainIroll  = 0.0015;
float GainDroll  = 0.0642;
///////////////////////////////
//float GainProll  = 3.87;
//float GainIroll  = 0.0015;
//float GainDroll  = 0.0652;
float GainPpitch = 3.96;
float GainIpitch = 0.0020;
float GainDpitch = 0.0474;
float GainPyaw   = 3.15;
float GainIyaw   = 0.0;
float GainDyaw   = -0.0717;

float roll_correction = -2.49;
float pitch_correction = -0.09;

int start;
int maxController = 200;
int rpm_motor1 = 0;
int rpm_motor2 = 0;
int rpm_motor3 = 0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(57600);

  //sensors
  init_MPU();
  //compass_init();

  //remote signals
  remote_init();
  delay(50);

  //actuators
  myservo.attach(3);
  servo_setup();
  motor_setup();

}

void loop()
{
  get_YPR();
  //compass_update();
  mapremote();
  
  gyro_yaw_input   = (gyro_yaw_input*0.7)  +(gz*0.3);
  
  control_update();

  serialEvent();
  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    
//********** Roll **********
//    Serial2.print("roll= ")        ; Serial2.print(roll_deg)          ; Serial2.print(" ");
//    Serial2.print("P= ")           ; Serial2.print(GainProll, 2)      ; Serial2.print(" ");
//    Serial2.print("I= ")           ; Serial2.print(GainIroll, 4)      ; Serial2.print(" ");
//    Serial2.print("D=  ")          ; Serial2.print(GainDroll, 4)      ; Serial2.print(" ");
//    Serial.print("mtr1= ")         ; Serial.print(motor1)             ; Serial.print("  ");
//    Serial.print("mtr2= ")         ; Serial.print(motor2)             ; Serial.print("  ");
    
//********** Pitch **********
//    Serial2.print("Pct= ")         ; Serial2.print(pitch_deg)         ; Serial2.print(" ");
//    Serial2.print("P= ")           ; Serial2.print(GainPpitch, 2)     ; Serial2.print(" ");
//    Serial2.print("I= ")           ; Serial2.print(GainIpitch, 4)     ; Serial2.print(" ");
//    Serial2.print("D=  ")          ; Serial2.print(GainDpitch, 4)     ; Serial2.print(" ");
//*********** Yaw ***********
//    Serial2.print("Yaw= ")         ; Serial2.print(heading_reference) ; Serial2.print(" ");
//    Serial2.print("P= ")           ; Serial2.print(GainPyaw, 2)       ; Serial2.print(" ");
//    Serial2.print("I= ")           ; Serial2.print(GainIyaw, 4)       ; Serial2.print(" ");
//    Serial2.print("D=  ")          ; Serial2.print(GainDyaw, 4)       ; Serial2.print(" ");

//    Serial2.print("m1:")          ; Serial2.print(motor1)       ; Serial2.print(" ");
//    Serial2.print("m2:")          ; Serial2.print(motor2)       ; Serial2.print(" ");
//    Serial2.print("m3:")          ; Serial2.print(motor3)       ; Serial2.print(" ");

//    Serial2.print("servoAngleInit:")          ; Serial2.print(servoAngleInit)       ; Serial2.print(" ");
//    Serial2.print("ch5 :")          ; Serial2.print(ch5_channel)       ; Serial2.print(" ");

//    Serial.print("roll :")          ; Serial.print(roll_deg)         ; Serial.print(" ");
//    Serial.print("pitch :")         ; Serial.print(pitch_deg)        ; Serial.print(" ");
//    Serial.print("yaw :")           ; Serial.print(heading_reference)       ; Serial.print(" ");

//    Serial2.print("Roll_input :")           ; Serial2.print(roll_input)       ; Serial2.print(" ");
//    Serial2.print("Pitch_input :")          ; Serial2.print(pitch_input)      ; Serial2.print(" ");
//    Serial2.print("yaw_input :")            ; Serial2.print(yaw_input)        ; Serial2.print(" ");
//    Serial2.print("R_c :")          ; Serial2.print(roll_correction)      ; Serial2.print(" ");
//    Serial2.print("P_c :")            ; Serial2.print(pitch_correction)        ; Serial2.print(" ");
    Serial.print("ch 1 = ");        Serial.print(roll_channel);      Serial.print(" ");
    Serial.print("ch 2 = ");        Serial.print(pitch_channel);     Serial.print(" ");
    Serial.print("ch 3 = ");        Serial.print(throttle_channel);  Serial.print(" ");
    Serial.print("ch 4 = ");        Serial.print(yaw_channel);       Serial.print(" ");
    Serial.print("ch 5 = ");        Serial.print(ch5_channel);       Serial.print(" ");
    Serial.print("ch 6 = ");        Serial.print(ch6_channel);       Serial.print(" ");
   
    Serial2.println();
    Serial.println();

    previousTimeProgram = micros();
  }
}
