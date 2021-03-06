// Ok here we go, the culmination of all the small test programs.
#include <Wire.h>
#include <Math.h>
#include <DFRobot_QMC5883.h>
#include <FreeSixIMU.h>
#include <ArduinoQueue.h>

int Rpin = 46;
int Gpin = 44;
int Bpin = 45;



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
int raw_compass_angle = 0;
//Magnetic calibration using this page as a guide:
// https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
int mag_bias[3] = {0,0,0};
float mag_scale[3] = {0.0f,0.0f,0.0f};
int mag_max[3] = {-32767,-32767,-32767};
int mag_min[3] = {32767,32767,32767};
int mag_temp[3] = {0,0,0};
int zmag_polarity_bias = 0;
int mag_cal_duration = 15; //In seconds
int ZERO_WAIT_DELAY = 6; //How long do we wait before we take the zero-heading reading?
int mag_heading_offset = 0; // How far off is the sensor's current orientation from true north? Cant perfectly aligning the sensor's north with our body so why try.


const int NUM_MOTORS = 6;
int MOTOR_ANGLE = (360/NUM_MOTORS); //Number of degrees between motors when mapped to a circle
// getOutputPinsFromAngle() returns index 0 when you give an angle of 0 or 360 (north). So straight forward should be index0 which should be the belly buzzer which should be the first motor on the belt (red motor). 
int MOTORS[NUM_MOTORS] = {4, 2, 3, 5, 7, 9,}; //Pins are not sequential anymore since the custom PCB layout
int MOTOR_PWM_MIN = 100; //Min value before the motor starts to move. Its actually 60 but you dont feel much until 70.
int MOTOR_PWM_MAX = 255; //Max range of our PWM output
int INTENSITY_ADJUST_POT_PIN = A0; // For changing the intensity of vibrations (PWM MAX)
int FILTER_SIZE_POT_PIN = A1; // For changing the size of the heading filter buffer
int MANUAL_HEADING_OFFSET_POT = A3; // For manually adjusting the heading offset value
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
bool ENABLE_FILTERING = true;
int buffer_size = 25;
double sinheading = 0.0;
double cosheading = 0.0;
ArduinoQueue<double> sin_heading_buffer(buffer_size);
ArduinoQueue<double> cos_heading_buffer(buffer_size);
double sin_heading_buffer_rolling_sum = 0;
double cos_heading_buffer_rolling_sum = 0;
double atan_angle = 0;

//Calibrate our magnetometer for dur seconds
void magcal(int dur) {  
  //Turn on red LED to indicate we are calibrating and give two long pulses of the belt
  analogWrite(Rpin, 255);
  beltPulseIndicator(2, false);
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
  //Indicate we are checking zero/aligning north pole with blue light and two fast pulses
  analogWrite(Bpin, 255);
  beltPulseIndicator(3, false);
  delay(ZERO_WAIT_DELAY*1000);
  setCompassOffset();
  mag = compass.readRaw();
  zmag_polarity_bias = mag.ZAxis;
  //Indicate we are done calibrating 
  analogWrite(Rpin, 0);
  analogWrite(Bpin, 0);
  delay(500);
  beltPulseIndicator(2, false);
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
//Having trouble using normal averaging when the angle is around 0/360
//Taking the sin and cosine of the current angle will give us a smooth periodic value so there shouldnt be any swapping issues anywhere
//Then we take the atan2 of the average of the sine/cosine values to get our filtered heading.
int getFilteredHeading(int newheading) {
  sinheading = sin(degToRads(newheading)); 
  cosheading = cos(degToRads(newheading)); 
  if (!sin_heading_buffer.isFull() or !cos_heading_buffer.isFull()) {
    sin_heading_buffer.enqueue(sinheading);
    sin_heading_buffer_rolling_sum += sinheading;
    cos_heading_buffer.enqueue(cosheading);
    cos_heading_buffer_rolling_sum += cosheading;
    Serial.println(" - notfull returning last heading");
    return newheading;
  }
  // Now that we have a full buffer we take the average of all values and then take the atan2 of that value and return it
  // Only two things that change are removing the oldest value and adding a new value
  sin_heading_buffer_rolling_sum -= sin_heading_buffer.dequeue();
  sin_heading_buffer.enqueue(sinheading);
  sin_heading_buffer_rolling_sum += sinheading;
  cos_heading_buffer_rolling_sum -= cos_heading_buffer.dequeue();
  cos_heading_buffer.enqueue(cosheading);
  cos_heading_buffer_rolling_sum += cosheading;
  atan_angle = radsToDeg(atan2(sin_heading_buffer_rolling_sum/buffer_size, cos_heading_buffer_rolling_sum/buffer_size));
  if (atan_angle < 0) atan_angle += 360; //shift the negative portion of our output to the right
  Serial.println(" - AVG HEADING: " + (String)atan_angle);
  return atan_angle; 
}
//Makes the math look cleaner and hopefully it doesnt drag performance down 
double degToRads(int angle_in_degrees) { return angle_in_degrees*PI/180; } 
int radsToDeg(double angle_in_rads) { return angle_in_rads * 180/PI; }

//This function will either be called at the end of magcal or on a button press.
void setCompassOffset() {
  //Need to get an accurate heading to set the offset so we need IMU data here
  struct ID imudata = getIMUData();
  raw_compass_angle = getCompassHeading(&imudata);
  if (raw_compass_angle <= 180) mag_heading_offset = -raw_compass_angle;
  else mag_heading_offset = 360-raw_compass_angle;
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
  //Correct for alignment of sensor
  mag.HeadingDegress += mag_heading_offset;
  Serial.println(360-mag.HeadingDegress);
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
// Instead of the visual indicator I'm going to use the belt to signal states
// Temporary until I can get v2 of the PCB designed and built, which will include the status LED.
void beltPulseIndicator(int count, bool fastmode) {
  for (int i=0; i<count; i++) {
    //I think index 3 is the motor on the belly of the belt
    //Maybe doing the 4 corners (F B L R) would be better? Less ambiguous since in normal operaton
    //the belt cant activate more than 2 motors at once. 
    analogWrite(MOTORS[0], 255);
    if (fastmode) delay(50);
    else delay(150);
    analogWrite(MOTORS[0], 0);
    delay(300);
  }  
}
void rgbflash(int pin, int num) {
  for (int i=0; i<num; i++) {
    analogWrite(pin, 255);
    delay(100);
    analogWrite(pin, 0);
    delay(100);
  }
}

void getOutputPinsFromAngle(int angle, struct MotorOutput *motorvals) {
  angle = constrain(angle, 0, 360);
  motorpin_index = angle/MOTOR_ANGLE;
//  if (motorpin_index < 0) motorpin_index = NUM_MOTORS - 1; //the frick is going on here?? I assume at some point previously we were subtracting from motorpin_index? dont think we need this anymore.
  else if (motorpin_index == NUM_MOTORS) motorpin_index = 0;
  motorvals->motor1_pin = MOTORS[motorpin_index];
  if (motorpin_index == NUM_MOTORS-1) motorvals->motor2_pin = MOTORS[0];
  else motorvals->motor2_pin = MOTORS[motorpin_index+1];
}

struct MotorOutput interpFromAngle(int angle, struct MotorOutput *motorvals){ 
  float ratio = 1.0 - ((float)(angle%MOTOR_ANGLE)/(float)MOTOR_ANGLE);
  motorvals->motor1_pwm = (int)(ratio * MOTOR_PWM_MAX);
  motorvals->motor2_pwm = MOTOR_PWM_MAX - motorvals->motor1_pwm;
}

void zeroPWMvals() { for (int i=0; i<NUM_MOTORS; i++) analogWrite(MOTORS[i], 0); }
void writeToMotors(struct MotorOutput *motorvals) {
  zeroPWMvals(); 
  //In testing I've found the motors don't start working until the PWM value reaches about 60
  //Because we are writing a zero first we don't have to worry about 60 being too low and motors never switching off
  analogWrite(motorvals->motor1_pin, map(motorvals->motor1_pwm, 0, 255, MOTOR_PWM_MIN, MOTOR_PWM_MAX)); 
  analogWrite(motorvals->motor2_pin, map(motorvals->motor2_pwm, 0, 255, MOTOR_PWM_MIN, MOTOR_PWM_MAX)); 
  delay(PWM_DELAY);
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

void loop() {
  struct MotorOutput motor_outputs; //Should be destroyed and recreated each loop iteration, not leaking.
  
  // Map the pot output from 0-1023 to 0-360
  pot_value = analogRead(INTENSITY_ADJUST_POT_PIN);
  //MOTOR_PWM_MAX = map(pot_value, 0, 1023, 0, 255);  //TODO FIX ME IN HARDWARE!
  struct ID imudata = getIMUData();
  raw_compass_angle = getCompassHeading(&imudata);
  //smooth the value out
  if (ENABLE_FILTERING) compass_angle = getFilteredHeading(raw_compass_angle);
  else compass_angle = raw_compass_angle;
  //Get the output pins based on the angle
  getOutputPinsFromAngle(compass_angle, &motor_outputs);
  //Get the PWM values for those motors, interpolated between the two
  interpFromAngle(compass_angle, &motor_outputs);
  //We should now have everything we need to output the correct PWM value to the correct pin
  writeToMotors(&motor_outputs);
}
