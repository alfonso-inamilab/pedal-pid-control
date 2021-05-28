#include <TLE5012-ino.hpp>
//#include <PID_v1.h>

#define PWM1_A 11
#define PWM1_B 12
#define PWM1_C 13

Tle5012Ino Tle5012Sensor = Tle5012Ino();
errorTypes checkError = NO_ERROR;
int CALIB_CYCLES = 10; //Number of initial cycles to average the calib value
int cycles_count =0;  //cycles counter for to calculate the calib value

// This version just displays the magnetic angle sensor from the XMC to the serial terminal 
// No motor control has been implemented yet.

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
  
  delay(1000);
  Serial.println("Init done");
}


double angle = 0.0;
double avg = 0.0;
double out = 0.0;
double OFFSET = 100.0;
void loop() {
  Tle5012Sensor.getAngleValue(angle);  //getting angle from the sensor 
  
  // This block is used to calibrate the initial position of the pedal
  if (cycles_count < CALIB_CYCLES){
      avg = avg + angle;
      cycles_count++;
  }else if (cycles_count == CALIB_CYCLES){
      avg = avg / CALIB_CYCLES;
      cycles_count++;
  }else if (cycles_count > CALIB_CYCLES){
    angle = angle - avg;  //applying the calibration value, to get the real angle

    Serial.print("°C\tangle:");  Serial.print(angle);
    Serial.print("°C\tavg:");  Serial.print(avg);
    Serial.println("");
    analogWrite(PWM1_A, angle);  //this doesnt work 
    analogWrite(PWM1_B, 255);
    analogWrite(PWM1_C, 0);
  }
}
