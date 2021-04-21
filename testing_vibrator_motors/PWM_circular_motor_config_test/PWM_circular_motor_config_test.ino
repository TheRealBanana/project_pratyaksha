#include <ArduinoQueue.h>

const int NUM_MOTORS = 6;
int MOTOR_ANGLE = (360/NUM_MOTORS); //Number of degrees between motors when mapped to a circle
int MOTORS[NUM_MOTORS] = {3, 5, 6, 9, 10, 11}; //All 6 PWM pins on the Uno/Nano/Mini
int MOTOR_PWM_MIN = 80; //Min value before the motor starts to move. Its actually 60 but you dont feel much until 70.
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
ArduinoQueue<int> heading_buffer(buffer_size);
int heading_buffer_rolling_sum = 0;

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
  Serial.println(" - AVG HEADING: " + (String)(heading_buffer_rolling_sum/buffer_size));
  return heading_buffer_rolling_sum/buffer_size; 
}

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

void loop() {
  struct MotorOutput motor_outputs; //Should be destroyed and recreated each loop iteration, not leaking.
  
  // Map the pot output from 0-1023 to 0-360
  pot_value = analogRead(POT_PIN);
  //Serial.print("Pot value: " + (String)pot_value);
  pot_angle = map(pot_value, 0, 1023, 0, 360);  
  //pot_angle = a;
  //if (f) a++;
  //else a--;
  //if (a == 300) f=false;
  //if (a == 0) f=true;
  //Serial.print("Pot angle: " + (String)pot_angle);
  //smooth the value out
  if (ENABLE_FILTERING) pot_angle = getFilteredHeading(pot_angle);
  //Get the output pins based on the angle
  getOutputPinsFromAngle(pot_angle, &motor_outputs);
  //Get the PWM values for those motors, interpolated between the two
  interpFromAngle(pot_angle, &motor_outputs);
  //We should now have everything we need to output the correct PWM value to the correct pin
  writeToMotors(&motor_outputs);
}
