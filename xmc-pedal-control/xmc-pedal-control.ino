#include <TLE5012-ino.hpp>
#include <PID_v1.h>

// This version implements a very basic PID for the ESCON and Flat Maxxon motor. 
// The library was taken from here: https://playground.arduino.cc/Code/PIDLibrary/
// And I follow this example here:  https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/

// TODO Tune the parameters using the Ziegler–Nichols method: 
// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

#define PWM1_A 11  //pin for the velocity
#define PWM1_B 12  //pin for the rotation direction
#define PWM1_C 13  //pin for the ON/OFF switch 

Tle5012Ino Tle5012Sensor = Tle5012Ino();
errorTypes checkError = NO_ERROR;
int CALIB_CYCLES = 10; //Number of initial cycles to average the calib value
int cycles_count =0;  //cycles counter for to calculate the calib value

// PID controller variables 
double Setpoint, Input, Output;             //goal,input and output
double consKp=1, consKi=0.05, consKd=0.25;  //tunning parameters 

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);  //Specify the links and initial tuning parameters

void setup() {
  delay(2000);
  Serial.begin(9600);
  while (!Serial) {};
  checkError = Tle5012Sensor.begin();
  Serial.print("checkError: ");
  Serial.println(checkError,HEX);

  pinMode(PWM1_A, OUTPUT);
  pinMode(PWM1_B, OUTPUT);
  pinMode(PWM1_C, OUTPUT);

  Setpoint = -60;  //PID goal angle
  Tle5012Sensor.getAngleValue(Input);  //Inital angle 
  myPID.SetMode(AUTOMATIC);  //turn the PID on
  myPID.SetTunings(consKp, consKi, consKd);   //set the PID tunning parameters
  Serial.println("Init done");
}

int rotDirection = 255;  
void loop() {
    Tle5012Sensor.getAngleValue(Input);  //getting angle from the sensor 
  
    Serial.print("°C\tangle:");  Serial.print(Output);   // FOR DEBUG ONLY 
    Serial.println("");                                 // FOR DEBUG ONLY 

    myPID.Compute();
    if (Output > 0){
      rotDirection = 255; 
    }else 
      rotDirection = 0;

    analogWrite(PWM1_A, Output);  //send the PWM to the epos
    analogWrite(PWM1_B, rotDirection);  // clockwise   
    analogWrite(PWM1_C, 255);    // Turn on the motor all the time
}
