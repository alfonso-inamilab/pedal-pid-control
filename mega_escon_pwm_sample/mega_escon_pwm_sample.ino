// This program is an example of how to move the pedal in an open loop using velocity 
// The motor is MAXON and the encoder box is ESCON 50/5
// The pedal should move foward or backwards the given VEL value and DIR value

//CONNECTIONS
// Arduino MEGA PIN 44 to ESCON D1 pin
// Arduino MEGA PIN 3  to ESCON D2 pin
// Arduino MEGA PIN 4  to ESCON D3 pin
// Arduino GND  PIN    to ESCON D5 pin

#define VEL_PIN 44  //MEGA pin for the velocity (phisical PWM pin)
#define ENABLED_PIN 3  //MEGA pin for enabled
#define DIRECTION_PIN 4  //MEGA pin for direction

int VEL = 76;  //motor velocity value (PWM value from 0 to 255) 
int DIR = LOW;  //rotation direction (LOW pedal foward, HIGH pedal backwards)

void setup() {
  // Set connections pins for output
  pinMode(VEL_PIN, OUTPUT);        //velocity control
  pinMode(ENABLED_PIN, OUTPUT);   //enabled pin
  pinMode(DIRECTION_PIN, OUTPUT); //rotation pin 

  digitalWrite(ENABLED_PIN, HIGH);  // enabled motor movement
}

void loop() {
  analogWrite(VEL_PIN, VEL);  //send the PWM to the epos
  digitalWrite(DIRECTION_PIN, DIR);    // rotation direction (LOW pedal foward, HIGH pedal backwards)
}
