///// Author(s): Jacob Wood

/*
 This code is used for a completely open-source (hackable) drone, the current concept is based on ideas given from : Carbon_Aeronautics_Quadcopter_Manual.pdf, but 
 modified quite a bit. The long term goals of this project are to make this drone completely autonomous and have the ability to communicate to a drone swarm.
 
 Currently, this drone operates off of an ELEGOO NANO, but will scope to an ESP32 in the future - giving me the availability for data communication 
 through webservers.
*/

// ===========================================================================================================================================================
// ======================================================================= PRE-INIT ==========================================================================
// ===========================================================================================================================================================

// --- LIBRARIES ---
#include <Wire.h> // i2c communication (A4 = SDA, A5 = SCL)
#include <math.h> // math
#include <VL53L0X.h> // distance sensor
#include <Adafruit_MPU6050.h> // imu
#include <Adafruit_BMP280.h> // atmospheric pressure sensor
#include <PPMReader.h> // radio reciever and control
#include <BasicLinearAlgebra.h> // for linear algebra control
  using namespace BLA;

// --- Objects/PINOUT ---
#define RED_LED 11           // OUTPUT PIN ON THE RED LED -> VOLTAGE LOW
#define GREEN_LED 12         // OUTPUT PIN ON THE GREEN LED -> VOLTAGE NOMINAL
uint8_t V_DIV = A7;          // Voltage divider location
Adafruit_MPU6050 mpu;        // IMU unit
Adafruit_BMP280 bmp;         // BMP280 Sensor
VL53L0X tofsensor;           // Distance Sensor (time of flight)

// --- Global Variables ---
// Constants
float mm2in = 0.03937008;
float update_hz = 250;
float dt = 1/update_hz*1E6; // update time of the drone in micro-seconds (250 hz)

// FOR IMU
float tempimu_f; // temperature
float accX, accY, accZ; // actual measurement
float rateroll, ratepitch, rateyaw; // actual measurement
float anglepitch, angleroll, angleyaw; // direct calculation
float k_roll, k_pitch, k_alt; // filtered measurement [measurement, uncertainty]
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // calibration constants
float prev_dvdt, velocity_z; // The prior velocity change value (for estimating when climbing)

// FOR TOF SENSOR AND HEIGHT CALCULATIONS
float alt_tof;

// FOR BAROMETER
float altitude, pressure_baro, tempbaro_f;
float alt_Cal, alt_pressureCal, alt_temperatureCal; // calibration constants

// FOR BATTERY MONITOR
float volt_battery;

// FOR RADIO RECIEVER
byte interruptPin = 3;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
int arrayindex = 0;
unsigned int Controller_CH[5]; // channels 0-6

// ===========================================================================================================================================================
// ======================================================================= FUNCTIONS =========================================================================
// ===========================================================================================================================================================

// --- Filter ---
// FILTERS NOISY DATA FROM IMU for pitch and roll
void FILTER(float U_roll, float U_pitch, float alt){
  static float xkr, pkr, xkp, pkp, xka, pka; // Current Kalman values x = meas, p = uncertainty
  static const float R1 = 9;
  static const float H1 = 1;
  static const float Q1 = 4, QA = 30; // expected variance (rough estimate)
  static float Kr, Kp, Ka; //kalman gain

  xkr = xkr + (float)(1/update_hz)*U_roll;
  xkp = xkp + (float)(1/update_hz)*U_pitch;
  xka = xka + (float)(1/update_hz)*alt;
  pkr = pkr + (float)(1/update_hz)*Q1;
  pkp = pkp + (float)(1/update_hz)*Q1;
  pka = pka + (float)(1/update_hz)*QA;

  Kr = pkr*H1/(H1*pkr*H1+R1);
  Kp = pkp*H1/(H1*pkp*H1+R1);
  Ka = pka*H1/(H1*pka*H1+R1);
  xkr = xkr + Kr*(U_roll-H1*xkr);
  xkp = xkp + Kp*(U_pitch-H1*xkp);
  xka = xka + Ka*(alt-H1*xka);
  pkr = pkr-(Kr*H1*pkr);
  pkp = pkp-(Kp*H1*pkp);
  pka = pka-(Ka*H1*pka);
  
  k_roll = xkr;
  k_pitch = xkp;
  k_alt = xka;
}

// --- Filter ---
// FILTERS NOISY DATA WITH BAROMETER AND ACCELEROMETER
void FILTER2D(float AccZIntertial){

}

// --- IMU SIGNALS ---
// UPDATES THE CURRENT ATTITUDE OF THE DRONE
void imu_signals(bool filter) {
  sensors_event_t Acc, Rate, temp; // can grab temp if wanted
  mpu.getEvent(&Acc, &Rate, &temp);

  // grab temp, acc, and roll values with calibration attached 
  tempimu_f = temp.temperature*1.8+32 - 6;
  accX = (float)Acc.acceleration.x - .95; 
  accY = (float)Acc.acceleration.y + .55;
  accZ = -(float)Acc.acceleration.z - 1.05;
  rateroll = (float)Rate.gyro.x-RateCalibrationRoll;
  ratepitch = (float)Rate.gyro.y-RateCalibrationPitch; 
  rateyaw = (float)Rate.gyro.z-RateCalibrationYaw;

  angleroll = atan(accY/sqrt(pow(accX,2)+pow(accZ,2)))/(PI/180); // X-axis (Phi) --  
  anglepitch = -atan(accX/sqrt(pow(accY,2)+pow(accZ,2)))/(PI/180); // Y-axis (Theta) --

  float Inertial_AccZ;
  if(filter==true) // checks if complimentary filter is neccessary
  {
    FILTER(angleroll, anglepitch, altitude);
    Inertial_AccZ = cos(k_roll*DEG_TO_RAD)*sin(k_pitch*DEG_TO_RAD)*accX - sin(k_roll*DEG_TO_RAD)*accY + cos(k_pitch*DEG_TO_RAD)*cos(k_roll*DEG_TO_RAD)*accZ; // see the vectrix math
  }
  else{
    Inertial_AccZ = cos(angleroll*DEG_TO_RAD)*sin(anglepitch*DEG_TO_RAD)*accX - sin(angleroll*DEG_TO_RAD)*accY + cos(anglepitch*DEG_TO_RAD)*cos(angleroll*DEG_TO_RAD)*accZ; // see the vectrix math

    // attempt to mitigate the steady state error and calibrate for accurate changes
  float dvdt_z = (Inertial_AccZ+9.81)*(1/update_hz); // in m/s
  if (abs(dvdt_z) > .0005) velocity_z = velocity_z + (prev_dvdt - dvdt_z);   
  prev_dvdt = dvdt_z;
  }   

  // attempt to mitigate the steady state error and calibrate for accurate changes
  float d_yaw = rateyaw*(dt/1E6)*RAD_TO_DEG; 
  if (abs(d_yaw) > .00175){ 
    angleyaw = angleyaw + d_yaw*2.65; // Z-axis (Beta) -- there is steady state error only viable for short term changes;
    if (abs(angleyaw) > 360) angleyaw = angleyaw-angleyaw;};
}

// --- GET ENVIRONMENT ---
// Gathers the current altitude readings from the TOF and Barometric sensors.
void get_env(bool startup){
  tempbaro_f = bmp.readTemperature()*1.8+32 - 5;
  pressure_baro = bmp.readPressure(); 

  if (startup==true) altitude = bmp.readAltitude(1013.25)*100; // measurement in cm
  else altitude = bmp.readAltitude(1013.25)*100 - alt_Cal;
  
  alt_tof = (float)tofsensor.readRangeContinuousMillimeters()*mm2in; 
}

// --- BMS CHECK ---
// MONITORS THE BATTERY IN THE DRONE
float bms(void)
{
  static float batterydefault = 1200, batteryatstart;
  float voltage;
  float R1 = 1994;
  float R2 = 547;

  voltage = (float)analogRead(V_DIV)/1023*5*((R1+R2)/R2);  //VOLTAGE DIVIDER READING

  if (voltage>8.3){
    batteryatstart=batterydefault;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH); //turn green led on
  }else if(voltage<7.5){
    batteryatstart=30/100*batterydefault;
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW); // turn red led on
  }else{
    batteryatstart=(82*voltage-580)/100*batterydefault;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH); // turn green led on
  }
    return voltage;
}

// --- CALIBRATE ---
// CALIBRATES THE DRONE AT START-UP (specify calibration length in ms)
void calibrate(int cal_time){
  for (int RateCalibrationNumber=0; RateCalibrationNumber<cal_time; RateCalibrationNumber++){
      get_env(true);
      alt_Cal += altitude;
      imu_signals(false);
      RateCalibrationRoll += rateroll;
      RateCalibrationPitch += ratepitch;
      RateCalibrationYaw += rateyaw;
      delay(1); 
    }

    alt_Cal /= cal_time;
    RateCalibrationRoll /= cal_time;
    RateCalibrationPitch /= cal_time;
    RateCalibrationYaw /= cal_time;
}

// ===========================================================================================================================================================
// ======================================================================= SET-UP ============================================================================
// ===========================================================================================================================================================

void setup(){
  Serial.begin(230400);                         // initialize serial monitor
  Wire.begin();                                 // initialize i2c communication

  while (!Serial) delay(10);                    // checks to make sure that serial communication is achieved
  Serial.println("===== Drone Initalization =====");

// --- Barometer (BMP280) ---   
  if(!bmp.begin(0x76)){
    Serial.println("Failed to find BMP280 chip");
    while (1) delay(10);
  }
  Serial.println("BMP280 Found");

// --- TIME OF FLIGHT SENSOR ---
  if (!tofsensor.init(0x52)){
    Serial.println("Failed to detect VL53L0X");
    while(1) delay(10);
  }
  Serial.println("VL53L0X Found");    
  tofsensor.startContinuous();       

// --- MPU 6050 (IMU) ---                               
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Found");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // setting g range -> OPTIONS: MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // setting the angular range of gyro -> OPTIONS: MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // setting the bandwith filter of data -> OPTIONS: MPU6050_BAND_260_HZ, MPU6050_BAND_184_HZ, MPU6050_BAND_94_HZ, MPU6050_BAND_44_HZ, MPU6050_BAND_21_HZ, MPU6050_BAND_10_HZ, MPU6050_BAND_5_HZ;
  calibrate(500);                            // calibration for x seconds     

// --- BATTERY MONITORING ---
  pinMode(RED_LED, OUTPUT);                     // sets the LED's and gathers first datapoint for battery monitoring
  pinMode(GREEN_LED, OUTPUT);
  volt_battery = bms();
}

// ===========================================================================================================================================================
// =========================================================================== MAIN ==========================================================================
// ===========================================================================================================================================================

void loop(){
  // --- Sensor updates ---
  get_env(false);                   // determines the environment
  imu_signals(true);                // determines the attitude
  volt_battery = bms();             // monitors the battery

  //Reading the remote inputs
  for (byte channel = 1; channel <= channelAmount; ++channel) {
    if(channel == 1) arrayindex = 0;
    Controller_CH[arrayindex] = ppm.latestValidChannelValue(channel, 0)-1000;
    ++arrayindex;
  }
  
  // Controller
  //Serial.print(">CH1:");
  //Serial.println(Controller_CH[0]);
  //Serial.print(">CH2:");
  //Serial.println(Controller_CH[1]);
  //Serial.print(">CH3:");
  //Serial.println(Controller_CH[2]);
  //Serial.print(">CH4:");
  //Serial.println(Controller_CH[3]);
  //Serial.print(">CH5:");
  //Serial.println(Controller_CH[4]);
  //Serial.print(">CH6:");
  //Serial.println(Controller_CH[5]);

  // IMU
  //Serial.print(">Roll:");
  //Serial.println(k_roll);
  //Serial.print(">Pitch:");
  //Serial.println(k_pitch);
  //Serial.print(">Yaw:");
  //Serial.println(angleyaw);

  // Barometer and height
  Serial.print(">Z_Height:");
  Serial.println(alt_tof);
  //Serial.print(">Z_vel:");
  //Serial.println(velocity_z);
  //Serial.print(">Altitude (1d filter):");
  //Serial.println(k_alt);
  //Serial.print(">Altitude (2d filter):");
  //Serial.println(altitude);

  //Environmental readings
  //Serial.print(">Temp (imu):");
  //Serial.println(tempimu_f);
  //Serial.print(">Temp (baro):");
  //Serial.println(tempbaro_f);
  //Serial.print(">Pressure (baro):");
  //Serial.println(pressure_baro);
  //Serial.print(">Battery Voltage:");
  //Serial.println(volt_battery);

  delayMicroseconds(dt);
}