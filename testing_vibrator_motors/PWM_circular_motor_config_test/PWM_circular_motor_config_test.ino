#include <ArduinoQueue.h>

const int NUM_MOTORS = 6;
int MOTOR_ANGLE = (360/NUM_MOTORS); //Number of degrees between motors when mapped to a circle
int MOTORS[NUM_MOTORS] = {3, 5, 6, 9, 10, 11}; //All 6 PWM pins on the Uno/Nano/Mini
int POT_PIN = A0; //Dont have to set to input pinmode to do analogRead()
int PWM_MAX = 255; //Max range of our PWM output
int PWM_DELAY = 10; //Trying to get rid of weird flickering now

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
//Take n samples per second and take an average over the last m samples (moving average)
int buffer_size = 255;
ArduinoQueue<int> heading_buffer(buffer_size);

//All the queue libs I could fine didn't have all the features I needed
class HeadingBuffer {
  public:
    HeadingBuffer(int size);
    void add(int val);
    int take();
    int peak(int index);
    int getSum();
    int getAverage();
    int isFull();
    int size();
  private:
    //variables like the array, size, pointer, etc..
    //dont be afraid to do the naive implementation as long as it works
    //who cares if there is a better solution. Get it working!
  
};

int getFilteredHeading(int newheading) {
  if (!heading_buffer.isFull()) {
    heading_buffer.enqueue(newheading);
    return newheading;
  }
  //do maths
  //Add the new heading and take the average of all values
  
  
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
  motorvals->motor1_pwm = (int)(ratio * PWM_MAX);
  motorvals->motor2_pwm = PWM_MAX - motorvals->motor1_pwm;
  //Serial.println("  |  Motor PWM vals: " + (String)motorvals->motor1_pwm + " - " + (String)motorvals->motor2_pwm);
}

void zeroPWMvals() { for (int i=0; i<NUM_MOTORS; i++) analogWrite(MOTORS[i], 0); }
void writeToMotors(struct MotorOutput *motorvals) {
  zeroPWMvals(); 
  analogWrite(motorvals->motor1_pin, motorvals->motor1_pwm); 
  analogWrite(motorvals->motor2_pin, motorvals->motor2_pwm); 
  delay(PWM_DELAY);
}

void loop() {
  struct MotorOutput motor_outputs; //Should be destroyed and recreated each loop iteration, not leaking.
  
  // Map the pot output from 0-1023 to 0-360
  pot_value = analogRead(POT_PIN);
  //Serial.print("Pot value: " + (String)pot_value);
  pot_angle = map(pot_value, 0, 1023, 0, 360);
  //Serial.print("Pot angle: " + (String)pot_angle);
  //Get the output pins based on the angle
  getOutputPinsFromAngle(pot_angle, &motor_outputs);
  //Get the PWM values for those motors, interpolated between the two
  interpFromAngle(pot_angle, &motor_outputs);
  //We should now have everything we need to output the correct PWM value to the correct pin
  writeToMotors(&motor_outputs);
}
