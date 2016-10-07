

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

MPU6050 mpu;

//Accel, Gyro, Mag LSB/RAW values  16-bit
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

//Accel, Gyro, Mag sensitivities , and other constants
const float accel_s = 16384; //LSB/g
const float gyro_s = 131;  //LSB/(degrees/s)
const float Pi = 3.141593;
const int outMax = 255;
const int outMin = -255;

//Complementary Filter variables
double accel_x, accel_y, accel_z; //acceleration in x,y,z direction
double accel;  //acceleration of gravity
double accelAngle;
double gyroAngleAccel;
unsigned long time; //to find dt
unsigned long old_time; //to find dt
double dt; //miliseconds
double angle;

//Motor Variables
int DIR_A = 4; //Determines direction of Motor A
int PWM_A = 5; //Determines power of Motor A
int DIR_B = 7; //Determines direction of Motor B
int PWM_B = 6; //Determines power of Motor B

//PID Control Variables
double uprightAngle = 0; //the empirically deduced upright position of the robot where pitch = 0
double tilt; //tilt = 0 when robot is at upright angle, therefore tilt = angle - uprightAngle
double oldTilt;
double sumTilt; // sum of all the tilts
int kp = 4.5; //Constant for proportional control
int ki = 0.3; //Constant for integrative control
int kd = 1; //Constant for derivative control
int Output; //Output for motor

void setup(){
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();
  //Setup Channel A
  pinMode(PWM_A, OUTPUT); //Initiates Motor Channel A pin
  pinMode(DIR_A, OUTPUT); //Initiates Direction Channel A pin

  //Setup Channel B
  pinMode(PWM_B, OUTPUT); //Initiates Motor Channel B pin
  pinMode(DIR_B, OUTPUT);  //Initiates Direction Channel B pin
  time = millis();
  angle = uprightAngle; //stable at 87.5
  tilt = 0;
  sumTilt = 0; //Integral of all calculated tilt,if this is perpetually high, change setpoint
}
void loop(){
  
    // calculating dt  
    old_time = time;
    time = millis();
    dt = (time - old_time)/1000;
    
    // getting the axes of motion (divide variable by accelerometer sensitivity
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    accel_x = ax/accel_s; 
    accel_y = ay/accel_s;
    accel_z = az/accel_s;
    
    //Processing Accelerometer and Gyroscope Data into pitch in degrees/ degrees per second
    accel = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
    accelAngle = (acos (accel_z/accel))*180/Pi; 
    gyroAngleAccel = gy/gyro_s; //gyroscope measurement of pitch is divided by sensitivity to yield degress rotation per second
    
    angle = (0.98)*(angle + gyroAngleAccel*(dt)) + (0.02)*(accelAngle); //Complementary Filter
    
    // All this is PID
    oldTilt = tilt;
    tilt = angle - uprightAngle; //how much robot is deviating from upright position
    sumTilt += (ki * tilt); // tilt can be positive or negative, hopefully stays near 0
    
    //However, output cannot be bigger than 255
    if(sumTilt > outMax) sumTilt = outMax; // to prevent over accumulation of integral value
    else if(sumTilt < outMin) sumTilt = outMin;
    
    Output = kp*tilt + sumTilt - kd*(tilt - oldTilt)*dt; //applying the PID Control algorithm
    
    if(Output > 0) Output += 80;
    else if(Output < 0) Output -= 80;
    if(Output > outMax) Output = outMax;
    else if(Output < outMin) Output = outMin;
    Serial.println(tilt);

    if (Output >= 0){
      PORTD &=~_BV(PD7);
      //digitalWrite(7, HIGH); //Establishes forward direction of Channel A
      analogWrite(6, Output);   //Spins the motor on Channel A at full speed
      
      PORTD |=_BV(PD4);
      //digitalWrite(4, LOW); //Establishes forward direction of Channel A
      analogWrite(5, Output);   //Spins the motor on Channel A at full speed
    }
  
    else{
      PORTD |=_BV(PD7);
      //digitalWrite(7, LOW); //Establishes forward direction of Channel A
      analogWrite(6, -Output);   //Spins the motor on Channel A at full speed
      PORTD &=~_BV(PD4);
      //digitalWrite(4, HIGH); //Establishes forward direction of Channel A
      analogWrite(5, -Output);   //Spins the motor on Channel A at full speed
    }

    
}

