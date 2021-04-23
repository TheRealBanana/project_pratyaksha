// Ok here we go, the culmination of all the small test programs.
#include <Wire.h>
#include <Math.h>
#include <DFRobot_QMC5883.h>
#include <FreeSixIMU.h>
#include <ArduinoQueue.h>

int Rpin = 44;
int Gpin = 45;
int Bpin = 46;


//IMU sensor data
struct ID {
  long accelx;
  long accely;
  long accelz;
  long gyrox;
  long gyroy;
  long gyroz;
  long yaw;
  long pitch;
  long roll;
};
//magnetic sensor data
struct MD {
  long magx;
  long magy;
  long magz;
  float magh;
  long rgbxnorm;
  long rgbynorm;
  long rgbznorm;
};
DFRobot_QMC5883 compass;
FreeSixIMU sixDOF = FreeSixIMU();
float angles[3]; // yaw pitch roll
int accval[3]; //Raw acc
float gyroval[3]; // Raw gyro
float declinationAngle = (11 + (37.0 / 60.0)) / (180 / PI);
//Range values for sensor
//The min constrain here is based on the ambient magnetic field at my location to get rid of some noise
//It really needs proper compensation but this works for now
int SENSOR_MIN = -32767;
int AMBIENT_MIN = 2000;
int SENSOR_MAX = 32767;
bool ENABLE_AMBIENT_MIN_CAL = false; //Try and figure out a noise floor at startup. Overrides AMBIENT_MIN.
bool ENABLE_MAG_CALIBRATION = true;
Vector mag = {0,0,0};
int compass_angle = 0;
//Magnetic calibration using this page as a guide:
// https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
int mag_bias[3] = {0,0,0};
float mag_scale[3] = {0.0f,0.0f,0.0f};
int mag_max[3] = {-32767,-32767,-32767};
int mag_min[3] = {32767,32767,32767};
int mag_temp[3] = {0,0,0};
int zmag_polarity_bias = 0;
int mag_cal_duration = 15; //In seconds

const int NUM_MOTORS = 6;
int MOTOR_ANGLE = (360/NUM_MOTORS); //Number of degrees between motors when mapped to a circle
int MOTORS[NUM_MOTORS] = {8, 9, 10, 5, 6, 7,}; //490hz only PWM pins on mega (not using 980hz on 4 and 13)
int MOTOR_PWM_MIN = 100; //Min value before the motor starts to move. Its actually 60 but you dont feel much until 70.
int MOTOR_PWM_MAX = 255; //Max range of our PWM output
int POT_PIN = A0; //Dont have to set to input pinmode to do analogRead()
float PWM_DELAY = 1.0; //Trying to get rid of weird flickering now

struct MotorOutput {
  int motor1_pin;
  int motor1_pwm;
  int motor2_pin;
  int motor2_pwm;
};

int motorpin_index;
int pot_value = 0;
int pot_angle = 0;
float pot_percent = 0.0f;

//My first idea for trying to smooth out the magnetic data
//Take the average heading over the last buffer_size readings (rolling average).
bool ENABLE_FILTERING = false;
int buffer_size = 25;
ArduinoQueue<int> heading_buffer(buffer_size);
int heading_buffer_rolling_sum = 0;

//Calibrate our magnetometer for dur seconds
void magcal(int dur) {  
  //Turn on red LED to indicate we are calibrating
  analogWrite(Rpin, 255);
  //Calibrate for dur seconds
  for (int t=0; t<dur*10; t++) {
    //Figure out the min and max of our sensor on all three axis
    mag = compass.readRaw();
    mag_temp[0] = mag.XAxis;
    mag_temp[1] = mag.YAxis;
    mag_temp[2] = mag.ZAxis;
    for (int j=0; j<3; j++) {
       if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
       if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
    } 
    delay(100);
  }
  //Calculate the bias and scaling factors
  //To use these correction values we subtract the bias factor and multiply by the scale factor
  mag_bias[0] = (mag_max[0] + mag_min[0])/2;
  mag_bias[1] = (mag_max[1] + mag_min[1])/2;
  mag_bias[2] = (mag_max[2] + mag_min[2])/2;
  int mag_scale_tmp[3] = {
    (mag_max[0] - mag_min[0])/2,
    (mag_max[1] - mag_min[1])/2,
    (mag_max[2] - mag_min[2])/2
  };
  float avg_rad = (mag_scale_tmp[0] + mag_scale_tmp[1] + mag_scale_tmp[2]) / 3.0f;
  mag_scale[0] = avg_rad/((float)mag_scale_tmp[0]);
  mag_scale[1] = avg_rad/((float)mag_scale_tmp[1]);
  mag_scale[2] = avg_rad/((float)mag_scale_tmp[2]);
  //Try and figure out a dynamic noise floor
  //Currently we just ignore anything below 2000 but maybe we can fine tune that at runtime
  //Maybe try just using the largest value we find in our min/max cal?
  //int m = max(abs(mag_min[0]), abs(mag_min[1]));
  //m = max(m, abs(mag_min[2]));
  //m = max(m, mag_max[0]);
  //m = max(m, mag_max[1]);
  //m = max(m, mag_max[2]);
  //Or maybe add up all the mins and maxes and take the average?
  int m = (abs(mag_min[0]) + abs(mag_min[1]) + abs(mag_min[2]) + mag_max[0] + mag_max[1] + mag_max[2])/6;
  if (ENABLE_AMBIENT_MIN_CAL) AMBIENT_MIN = m;
  //Indicate we are checking zero
  analogWrite(Bpin, 255);
  delay(5000);
  mag = compass.readRaw();
  zmag_polarity_bias = mag.ZAxis;
  //Indicate we are done calibrating 
  analogWrite(Rpin, 0);
  analogWrite(Bpin, 0);
  delay(500);
  rgbflash(Rpin, 1);
  rgbflash(Gpin, 1);
  rgbflash(Bpin, 1);
}

Vector applymagcal(Vector mag) {
  if (!ENABLE_MAG_CALIBRATION) return mag;
  Vector out = mag;
  //Add our bias factor to recenter our data to 0
  out.XAxis = out.XAxis - mag_bias[0];
  out.YAxis = out.YAxis - mag_bias[1];
  out.ZAxis = out.ZAxis - mag_bias[2];
  //And multiply by the scaling factor to align the min/max range
  out.XAxis = mag.XAxis * mag_scale[0];
  out.YAxis = mag.YAxis * mag_scale[1];
  out.ZAxis = mag.ZAxis * mag_scale[2];
  
  return out;
}

Vector correctHeadingWithIMU(Vector mag, struct ID * imudata) {
  Vector out = mag;
  
  //Correcting heading data by applying a rotation to align the different frames of reference
  //Using MTD-0801_1_0_Calculating_Heading_Elevation_Bank_Angle.pdf as a reference guide
  //Convert to radians first
  float pitch = imudata->pitch * PI/180.0f; //Alpha
  float roll = imudata->roll * PI/180.0f;   //Beta
  //float pitch = atan2(imudata->accelx, imudata->accelz);
  //float roll = atan2(imudata->accely, imudata->accelz);
  //We only need the X/Y data to calculate heading so we dont need to rotate the Gamma angle
  out.XAxis = mag.XAxis*cos(pitch) + mag.YAxis*sin(roll)*sin(pitch) - mag.ZAxis*cos(roll)*sin(pitch); //Might need to subtract Z
  out.YAxis = mag.YAxis*cos(roll) - mag.ZAxis*sin(roll);

  //Correct for hard and soft iron deposits
  out = applymagcal(out);
  
  //Now calculate heading using arctan(magx/magy) and convert to degrees
  float magh_c = atan2(out.YAxis, out.XAxis) + declinationAngle;
  if (magh_c < 0.0f) magh_c += 2*PI;
  if (magh_c >= 2*PI) magh_c -= 2*PI;
  magh_c = magh_c * 180/PI;
  out.HeadingDegress = magh_c;
  //Serial.println("MAGH/MAGH_C: " + (String)mag.HeadingDegress + "  |  " + (String)out.HeadingDegress);
  //Serial.println("P/R: " + (String)(pitch* 180/PI) + "/" + (String)imudata->pitch + "  |  " + (String)(roll* 180/PI) + "/" + (String)imudata->roll);

  return out;
}

int getFilteredHeading(int newheading) {
  if (!heading_buffer.isFull()) {
    heading_buffer.enqueue(newheading);
    heading_buffer_rolling_sum += newheading;
    Serial.println(" - notfull returning last heading");
    return newheading;
  }
  // Now that we have a full buffer of heading values, we take the average of them all
  // Only two things that change are removing the oldest heading value and adding a new value
  heading_buffer_rolling_sum -= heading_buffer.dequeue();
  heading_buffer.enqueue(newheading);
  heading_buffer_rolling_sum += newheading;
  //Serial.println(" - AVG HEADING: " + (String)(heading_buffer_rolling_sum/buffer_size));
  return heading_buffer_rolling_sum/buffer_size; 
}
int getCompassHeading(struct ID *imudata) {
  struct MD returndata;
  
  mag = compass.readRaw();
  
  //Serial.println("heading: " + (String)mag.HeadingDegress);
  mag.XAxis = constrain(mag.XAxis, SENSOR_MIN, SENSOR_MAX);
  mag.YAxis = constrain(mag.YAxis, SENSOR_MIN, SENSOR_MAX);
  mag.ZAxis = constrain(mag.ZAxis, SENSOR_MIN, SENSOR_MAX);
  //Apply tilt compensation and our calibration factors to the magnetic data
  mag = correctHeadingWithIMU(mag, imudata);
  Serial.println(mag.HeadingDegress);
  return 360-mag.HeadingDegress;
};

struct ID getIMUData() {
  struct ID returndata;
  
  sixDOF.gyro.readGyro(gyroval);
  sixDOF.acc.readAccel(accval);
  //Sensor drift, especially for the yaw axis, is really bad for some reason with getAngles()
  //sixDOF.getAngles(angles); // These values are absolute values from 0-360
  sixDOF.getYawPitchRoll(angles); // This is the type of values we need, relative different in angle from rest.
  //sixDOF.getEuler(angles);
  returndata.accelx = accval[0];
  returndata.accely = accval[1];
  returndata.accelz = accval[2];
  returndata.gyrox = gyroval[0];
  returndata.gyroy = gyroval[1];
  returndata.gyroz = gyroval[2];
  returndata.yaw = angles[0];
  returndata.pitch = angles[1];
  returndata.roll = angles[2];

  return returndata;
}

void rgbflash(int pin, int num) {
  for (int i=0; i<num; i++) {
    analogWrite(pin, 255);
    delay(100);
    analogWrite(pin, 0);
    delay(100);
  }
}

void setup() {
  analogWrite(Rpin, 0);
  analogWrite(Gpin, 0);
  analogWrite(Bpin, 0);
  Serial.begin(9600);
  // Loop through and set output mode on all MOTOR pins
  for (int i=0; i<6; i++) pinMode(MOTORS[i], OUTPUT);
  sixDOF.init(true);
  //Magnetometer init
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  if (ENABLE_MAG_CALIBRATION) magcal(mag_cal_duration);
  compass.setDeclinationAngle(declinationAngle);
}

void getOutputPinsFromAngle(int angle, struct MotorOutput *motorvals) {
  angle = constrain(angle, 0, 360);
  motorpin_index = angle/MOTOR_ANGLE;// - 1;
  //Serial.print("  |  motorpin_index 1: " + (String)motorpin_index);
  if (motorpin_index < 0) motorpin_index = NUM_MOTORS - 1;
  else if (motorpin_index == NUM_MOTORS) motorpin_index = 0;
  motorvals->motor1_pin = MOTORS[motorpin_index];
  //Serial.print("  |  motorpin_index 2: " + (String)motorpin_index);
  if (motorpin_index == NUM_MOTORS-1) motorvals->motor2_pin = MOTORS[0];
  else motorvals->motor2_pin = MOTORS[motorpin_index+1];
  //Serial.print("  |  Motor pins: " + (String)motorvals->motor1_pin + " - " + (String)motorvals->motor2_pin);
}

struct MotorOutput interpFromAngle(int angle, struct MotorOutput *motorvals){ 
  float ratio = 1.0 - ((float)(angle%MOTOR_ANGLE)/(float)MOTOR_ANGLE);
  motorvals->motor1_pwm = (int)(ratio * MOTOR_PWM_MAX);
  motorvals->motor2_pwm = MOTOR_PWM_MAX - motorvals->motor1_pwm;
  //Serial.println("  |  Motor PWM vals: " + (String)motorvals->motor1_pwm + " - " + (String)motorvals->motor2_pwm);
}

void zeroPWMvals() { for (int i=0; i<NUM_MOTORS; i++) analogWrite(MOTORS[i], 0); }
void writeToMotors(struct MotorOutput *motorvals) {
  zeroPWMvals(); 
  //In testing I've found the motors don't start working until the PWM value reaches about 60
  //Because we are writing a zero first we don't have to worry about 70 being too low and motors never switching off
  analogWrite(motorvals->motor1_pin, map(motorvals->motor1_pwm, 0, 255, MOTOR_PWM_MIN, MOTOR_PWM_MAX)); 
  analogWrite(motorvals->motor2_pin, map(motorvals->motor2_pwm, 0, 255, MOTOR_PWM_MIN, MOTOR_PWM_MAX)); 
  delay(PWM_DELAY);
}

int a = 0;
bool f = true;

void loop() {
  struct MotorOutput motor_outputs; //Should be destroyed and recreated each loop iteration, not leaking.
  
  // Map the pot output from 0-1023 to 0-360
  pot_value = analogRead(POT_PIN);
  //Serial.print("Pot value: " + (String)pot_value);
  //pot_angle = map(pot_value, 0, 1023, 0, 360);  
  struct ID imudata = getIMUData();
  compass_angle = getCompassHeading(&imudata);
  //pot_angle = a;
  //if (f) a++;
  //else a--;
  //if (a == 300) f=false;
  //if (a == 0) f=true;
  //Serial.print("Pot angle: " + (String)pot_angle);
  //smooth the value out
  if (ENABLE_FILTERING) compass_angle = getFilteredHeading(compass_angle);
  //Get the output pins based on the angle
  getOutputPinsFromAngle(compass_angle, &motor_outputs);
  //Get the PWM values for those motors, interpolated between the two
  interpFromAngle(compass_angle, &motor_outputs);
  //We should now have everything we need to output the correct PWM value to the correct pin
  writeToMotors(&motor_outputs);
}
