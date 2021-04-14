int MOTOR_PIN = 6; //Can also do PWM on this pin
int MOTOR_PIN_2 = 5;
int POT_PIN = A0; //Dont have to set to input to do analogRead()

struct motors {
  int m1 = 0;
  int m2 = 0;
};

int pot_value = 0;
float pot_percent = 0.0f;
int pwm_value = 0;
struct motors mvals;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(POT_PIN, LOW);
}

struct motors interpolate(float ratio, int value){ 
  struct motors outvals;
  outvals.m1 = (int)(ratio * (float)value);
  outvals.m2 = value - outvals.m1;
  return outvals;
}

void loop() {
  
  // Figure out our delay based on the 1k potentiometer
  pot_value = analogRead(POT_PIN);
  mvals = interpolate((float)pot_value/1023.0, 255);
  Serial.print(mvals.m1);
  Serial.print(" | ");
  Serial.println(mvals.m2);
  //Serial.println(pot_value);
  //pwm_value = map(pot_value, 0, 1023, 40, 255);
  //Serial.println(pwm_value);
  analogWrite(MOTOR_PIN, mvals.m1);
  analogWrite(MOTOR_PIN, mvals.m2);
}
