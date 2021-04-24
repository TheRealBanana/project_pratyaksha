#include <ArduinoQueue.h>

const int NUM_MOTORS = 6;
int MOTOR_ANGLE = (360/NUM_MOTORS); //Number of degrees between motors when mapped to a circle
int MOTORS[NUM_MOTORS] = {5, 6, 7, 8, 9, 10}; //Using Mega now, normal PWM pins
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
bool ENABLE_FILTERING = true;
int buffer_size = 25;
//Arctan/tan alone didnt seem to work so I had to go with atan2 which requires cos and sin components
double sinheading = 0.0;
double cosheading = 0.0;
int angle = 0;
ArduinoQueue<double> sin_heading_buffer(buffer_size);
ArduinoQueue<double> cos_heading_buffer(buffer_size);
double sin_heading_buffer_rolling_sum = 0;
double cos_heading_buffer_rolling_sum = 0;

//Having trouble using normal averaging when the angle is around 0/360
//Taking the sin of the current angle will give us a smooth periodic value so there shouldnt be any swapping issues anywhere
//Then we take the arcsine of the average of the sine values to get our filtered heading.
int getFilteredHeading(int newheading) {
  sinheading = sin(degToRads(newheading)); 
  cosheading = cos(degToRads(newheading)); 
  if (!sin_heading_buffer.isFull() or !cos_heading_buffer.isFull()) {
    sin_heading_buffer.enqueue(sinheading);
    sin_heading_buffer_rolling_sum += sinheading;
    cos_heading_buffer.enqueue(sinheading);
    cos_heading_buffer_rolling_sum += sinheading;
    Serial.println(" - notfull returning last heading");
    return newheading;
  }
  // Now that we have a full buffer we take the average of all values and then take the arcsine of that value and return it
  // Only two things that change are removing the oldest value and adding a new value
  sin_heading_buffer_rolling_sum -= sin_heading_buffer.dequeue();
  sin_heading_buffer.enqueue(sinheading);
  sin_heading_buffer_rolling_sum += sinheading;
  cos_heading_buffer_rolling_sum -= cos_heading_buffer.dequeue();
  cos_heading_buffer.enqueue(cosheading);
  cos_heading_buffer_rolling_sum += cosheading;
  angle = radsToDeg(atan2(sin_heading_buffer_rolling_sum/buffer_size, cos_heading_buffer_rolling_sum/buffer_size));
  if (angle < 0) angle += 360; //shift the negative portion of our output to the right
  Serial.println(" - AVG HEADING: " + (String)angle);
  return angle; 
}
//Makes the math look cleaner and hopefully it doesnt drag performance down 
double degToRads(int angle_in_degrees) { return angle_in_degrees*PI/180; } 
int radsToDeg(double angle_in_rads) { return angle_in_rads * 180/PI; }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Loop through and set output mode on all MOTOR pins
  for (int i=0; i<6; i++) pinMode(MOTORS[i], OUTPUT);
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
int d = 10;

void loop() {
  struct MotorOutput motor_outputs; //Should be destroyed and recreated each loop iteration, not leaking.
  
  // Map the pot output from 0-1023 to 0-360
  pot_value = analogRead(POT_PIN);
  //d = map(pot_value, 0, 1023, 1, 50);
  //Serial.println("PWM: " + (String)MOTOR_PWM_MIN);
  Serial.print("Pot value: " + (String)pot_value);
  pot_angle = map(pot_value, 0, 1023, 0, 360);  
  //BACK AND FORTH
  //pot_angle = a;
  //if (f) a++;
  //else a--;
  //if (a == 300) f=false;
  //if (a == 0) f=true;
  //IN A CIRCLE TO THE RIGHT
  //pot_angle = a;
  //if (a == 360) a=0;
  //a++;
  //delay(d);
  
  //Serial.print("Pot angle: " + (String)pot_angle);
  //smooth the value out
  if (ENABLE_FILTERING) pot_angle = getFilteredHeading(pot_angle);
  //Get the output pins based on the angle
  getOutputPinsFromAngle(pot_angle, &motor_outputs);
  //Get the PWM values for those motors, interpolated between the two
  interpFromAngle(pot_angle, &motor_outputs);
  //We should now have everything we need to output the correct PWM value to the correct pin
  //writeToMotors(&motor_outputs);
}
