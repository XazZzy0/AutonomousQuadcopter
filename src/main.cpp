// Authors: Jacob Wood & Nico Wood
// Based on ideas given from : Carbon_Aeronautics_Quadcopter_Manual.pdf 

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MPU6050.h"
#include "BasicLinearAlgebra.h"
#include "NewPing.h"
using namespace BLA;

#define MPU6050_Address 0X68
#define TRIGGER_PIN 8
#define ECHOPIN 9
#define MAX_DISTANCE 200
#define RED_LED 10
#define GREEN_LED 12

Adafruit_MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHOPIN, MAX_DISTANCE);

// ----- IMU Set up -----
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float Inertial_AccZ, VelocityZ;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;

float K_AngleRoll = 0, K_UncertaintyAngleRoll = 2*2;
float K_AnglePitch = 0, K_UncertaintyAnglePitch = 2*2;
float K_1DOutput[] = {0,0};

float temp = 0;
float batterylife;

// ----- Barometer Set up -----
// https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7 ,dig_P8, dig_P9;

float Altitude, AltitudeStartUp, Pressure;

//----- Altitude Filtering Set up -----
float K_Altitude, K_VelocityZ;
BLA::Matrix<2,2, float> F; BLA::Matrix<2,1, float> G;
BLA::Matrix<2,2, float> P; BLA::Matrix<2,2, float> Q;
BLA::Matrix<2,1, float> S; BLA::Matrix<1,2, float> H;
BLA::Matrix<2,2, float> I; BLA::Matrix<1,1, float> Acc;
BLA::Matrix<2,1, float> K; BLA::Matrix<1,1, float> R;
BLA::Matrix<1,1, float> L; BLA::Matrix<1,1, float> M;

// ----- Environment Set up -----
float ge = 9.81; // gravity

// ----- 1D Kalman Filter function -----
// https://en.wikipedia.org/wiki/Kalman_filter
void k1d(float state, float uncertainty, float input, float measurement){
  state = state + 0.004*input;
  uncertainty = uncertainty + 0.004*0.004*4*4;
  float k_gain = uncertainty/(1*uncertainty + 3*3);

  state = state + k_gain*(measurement-state);
  uncertainty = (1-k_gain)*uncertainty;

  K_1DOutput[0] = state;
  K_1DOutput[1] = uncertainty;
}

// ----- 2D Kalman Filter function -----
// https://www.youtube.com/watch?v=GZevJyabMdI&list=PLeuMA6tJBPKsAfRfFuGrEljpBow5hPVD4&index=22
void k2d(void){
  Acc = {Inertial_AccZ};
  S=F*S+G*Acc;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K=P*~H*Inverse(L);
  M={Altitude};
  S=S+K*(M-H*S);
  K_Altitude = S(0,0);
  K_VelocityZ = S(1,0);
  P=(I-K*H)*P;
}

//Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf & https://www.youtube.com/watch?v=7VW_XVbtu9k 
void gyro_signals(void) {
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x1A);  // Low-pass filter activation
  Wire.write(0x05);  // Cut-off frequency of 10 hz -> DLPF setting of 5 = 00000101
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x1C); // Setting the sensitivity scale factor
  Wire.write(0x10); // Setting the sensitivity scale factor to 10
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x3B); // access registers which store accel measurements
  Wire.endTransmission(); 
  Wire.requestFrom(MPU6050_Address,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x1B); // access registers which store gyro measurements
  Wire.write(0x8);
  Wire.endTransmission(); 
  AccX = (float)AccXLSB/4096 - 0.03;  //converts the LSB reading to g's (with calibration)
  AccY = (float)AccYLSB/4096 + 0.00;
  AccZ = (float)AccZLSB/4096 + 0.16;
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_Address,6); 
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  AngleRoll = atan(AccY/sqrt(pow(AccX,2)+pow(AccZ,2)))/(PI/180); // X-axis (Phi) --  Result in degrees
  AnglePitch = -atan(AccX/sqrt(pow(AccY,2)+pow(AccZ,2)))/(PI/180); // Y-axis (Theta) -- 

  Inertial_AccZ = -sin(AnglePitch*RAD_TO_DEG)*AccX + cos(AnglePitch*RAD_TO_DEG)*sin(AngleRoll*RAD_TO_DEG)*AccY 
                  + cos(AnglePitch*RAD_TO_DEG)*cos(AngleRoll*RAD_TO_DEG)*AccZ;
  Inertial_AccZ = (Inertial_AccZ-1)*9.81;
  VelocityZ = VelocityZ + Inertial_AccZ*.004;
 }

// Determines current readings 
void barometer_signals(void){
 Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);

  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);

  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13 ))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));

  double pressure=(double)p/100; 

  Pressure = pressure/10; //converts into kilo-pascal
  Altitude = 44330*(1-pow(pressure/1013.25, 1/5.255)) * 100; // currently in meters
}

// Verifies current battery life
float bms_check(bool init){
  float voltage, batteryremaining, batteryatstart;
 // float current, currcons, precurrcons;
  float batterydefault = 1200;
  float R1 = 1850;
  float R2 = 535;
  float RVdrop = 464;
  float vdropratio = (RVdrop/100)/14;

  voltage = (float)analogRead(A1)/1023*5*((R1+R2)/R2);  //VOLTAGE DIVIDER READING
 // current = (float)analogRead(A6)/(1023/5)*vdropratio;

  if (voltage>8.3){
    batteryatstart=batterydefault;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH); //keep green led on
  }else if(voltage<7.5){
    batteryatstart=30/100*batterydefault;
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW); // keep green led on
  }else{
    batteryatstart=(82*voltage-580)/100*batterydefault;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH); // turn on green led
  }
    return voltage;
  }


void setup(void) {
 /*Makes contact with the serial interface*/
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);   
  Wire.endTransmission();
  Wire.beginTransmission(0x76); 
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();  

  // Barometric reading
  uint8_t data[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);

  /*Battery LED Set-up*/
  pinMode(RED_LED, OUTPUT); // Red LED pinout
  pinMode(GREEN_LED, OUTPUT); // Green LED pinout
  batterylife=bms_check(true);
 // checks current battery life

  /*IMU Calibration timer*/
  if(true) // Skips calibration if false
  {
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++)
    {
      barometer_signals();
      AltitudeStartUp += Altitude;
      gyro_signals();
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      delay(1);
    }
    AltitudeStartUp /= 2000;
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
  }

  //Defining Kalman Matrix Values
    F = {1, 0.004, 0, 1};
    G = {0.5*0.004*0.004, 0.004};
    H = {1, 0};
    I = {1, 0, 0, 1};
    Q = G*~G*10.0f*10.0f;
    R = {30*30};
    P = {0, 0, 0, 0};
    S = {0, 0};
    LoopTimer = micros();
} 

void loop() 
{
  batterylife=bms_check(false);
  barometer_signals();
  Altitude -= AltitudeStartUp;
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  int distance = sonar.ping_cm(); // sonar reading in cm

  // Applied Kalman Filters
  k1d(K_AngleRoll, K_UncertaintyAngleRoll, RateRoll, AngleRoll);
  K_AngleRoll = K_1DOutput[0], K_UncertaintyAngleRoll = K_1DOutput[1];
  k1d(K_AnglePitch, K_UncertaintyAnglePitch, RatePitch, AnglePitch);
  K_AnglePitch = K_1DOutput[0], K_UncertaintyAnglePitch = K_1DOutput[1];
  k2d();

  Serial.print(">AngleRoll:");
  Serial.println(K_AngleRoll);
  Serial.print(">Altitude:");
  Serial.println(K_Altitude);
  Serial.print(">Batterylife:");
  Serial.println(batterylife);
  Serial.print(">K_VelocityZ:");
  Serial.println(K_VelocityZ);
  Serial.print(">Distance:");
  Serial.println(distance);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}