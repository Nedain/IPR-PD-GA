#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

unsigned long lastPrintTime = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 5 //175

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//Encoder
int encoderA_L = 19;
int encoderB_L = 18;

int pulsesChanged_L = 0;
int totalPPR_L = 750;

int pulses_L;

float angle_L;

//PID
double originalSetpoint = 3.15;//4.73;//5.87;//5.74;//4.26;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//PD IMU
double Kp = 175.0;
double Kd = 3.0;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//PD Encoder
double Kp_enc = 0.006;
double Kd_enc = 0.21;
double Ki_enc = 0;
double input_enc, output_enc;
double setpoint_enc = 0.0;
PID pid_enc(&input_enc, &output_enc, &setpoint_enc, Kp_enc, Ki_enc, Kd_enc, DIRECT);

double output_tot;
double motorSpeedFactorLeft = 0.4;
double motorSpeedFactorRight = 0.43;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 44;
int IN2 = 46;
int IN3 = 52;
int IN4 = 50;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);
  attachInterrupt(4, A_CHANGE_L, CHANGE);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(155);
  mpu.setYGyroOffset(56);
  mpu.setZGyroOffset(-5);
  mpu.setZAccelOffset(957); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
 
    //setup PD IMU
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(5);
    pid.SetOutputLimits(-250, 250);

    //setup PD Encoder
    pid_enc.SetMode(AUTOMATIC);
    pid_enc.SetSampleTime(10);
    pid_enc.SetOutputLimits(-250, 250);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
//    Serial.print(F("DMP Initialization failed (code "));
//    Serial.print(devStatus);
//    Serial.println(F(")"));
  }
}

//ENCODERRR------------------------------------------------
void A_CHANGE_L() {
  if ( digitalRead(encoderB_L) == 0 ) {
    if ( digitalRead(encoderA_L) == 0 ) {
      pulses_L--;
    }
    else {
      pulses_L++;
    }
  }
  else {
    if ( digitalRead(encoderA_L) == 0 ) {
      pulses_L++;
    }
    else {
      pulses_L--;
    }
  }
  pulsesChanged_L = 1;
}

//MAIN-----------------------------------------------------
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
//    angle_L = (pulses_L / (float)totalPPR_L) * 360;
    angle_L = (pulses_L / (float)totalPPR_L) * 2 * PI; 
    input_enc = angle_L;
//    Serial.println(input_enc);

    pid.Compute();
    pid_enc.Compute();
    output_tot = output + output_enc;
    if (output_tot >= 250){
      output_tot = 250;
    }
    else if (output_tot <= -250){
      output_tot = -250;
    }
//    Serial.println(output);
//    Serial.println(output_enc);
//    Serial.println(output_tot);
    motorController.move(output_tot, MIN_ABS_SPEED);
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 29) {
      Serial3.print(currentTime-372);
      Serial3.print("_");
      Serial3.print(input);
      Serial3.print("_");
      Serial3.println(input_enc);
  //    Serial.println(currentTime-372);
      lastPrintTime = currentTime;
    }
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
//    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
 
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI;
//    Serial3.println(input);
    Serial.println(input);
  }
}
