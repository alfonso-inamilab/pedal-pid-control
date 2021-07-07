// This version implements a very basic PID for the ESCON contoler and the Flat Maxxon motor. 
// The library was taken from here: https://playground.arduino.cc/Code/PIDLibrary/
// And I followed this example here:  https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/

// TODO: Tune the parameters using the Ziegler–Nichols method: 
// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

// The magnetic angle encoder used for the task is AS5600. 
// the sample code and the connections are taken from here:
// https://curiousscientist.tech/blog/as5600-magnetic-position-encoder 

// CONNECTIONS (magnetometer)
// Arduino MEGA pin 21 SCL to magnetometer SCL pin
// Arduino MEGA pin 20 SDA to magnetometer SDA pin
// Arduino GND pin to magnetometer GND pin 
// Arduino MEGA 5V output to magnetometer VCC
// CONNECTIONS (ESCON controller) 
// Arduino MEGA PIN 44 to ESCON D1 pin
// Arduino MEGA PIN 3  to ESCON D2 pin
// Arduino MEGA PIN 4  to ESCON D3 pin
// Arduino GND  PIN    to ESCON D5 pin

// PROGRAM 
// This a PID controler for the motorized pedal.
// TODO, calibrate the the PID Kp, KI and Kd values (consKp, consKi, consKd)

#include <PID_v1.h> //PID controller library
#include <Wire.h>  //This is for i2C

#define PWM_PIN 44      //MEGA pin for the velocity (physical PWM on MEGA)
#define ENABLE_PIN  3  //MEGA pin for the rotation direction
#define ROTATION_PIN 4  //MEGA pin for the ON/OFF switch 

// PID controller variables 
double Setpoint, Input, Output;             //goal, input and output
double consKp=1.5, consKi=0.0, consKd=0.0;  // PID tunning parameters 
int rotation = 0;   //it can be only HIGH or LOW
float angle=0;     //vales the angle value 
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);  //Specify the links and initial PID tuning parameters

//Internal variables of the checkMagnetPresence() and ReadRawAngle() functions
int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle;   //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
int magnetStatus = 0; //value of the status register (MD, ML, MH)

void checkMagnetPresence()
  {  
    //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
  
    while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
    {
      magnetStatus = 0; //reset reading
  
      Wire.beginTransmission(0x36); //connect to the sensor
      Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
      Wire.endTransmission(); //end transmission
      Wire.requestFrom(0x36, 1); //request from the sensor
  
      while(Wire.available() == 0); //wait until it becomes available 
      magnetStatus = Wire.read(); //Reading the data after the request
  
      //Serial.print("Magnet status: ");
      //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
    }      
    
    //Status register output: 0 0 MD ML MH 0 0 0  
    //MH: Too strong magnet - 100111 - DEC: 39 
    //ML: Too weak magnet - 10111 - DEC: 23     
    //MD: OK magnet - 110111 - DEC: 55
  
    //Serial.println("Magnet found!");
    //delay(1000);  
  }

float ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle 
  return degAngle;
}

void setup() {
  delay(1000);
  Serial.begin(115200);  // Setup serial comunication 
  while (!Serial) {};
  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock

  //checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  
  Setpoint = 250;  //set PID goal angle
  angle = ReadRawAngle();  //get inital angle 
  Input = angle;  //copy angle value to PID 
  
  myPID.SetMode(AUTOMATIC);  //turn the PID on
  myPID.SetTunings(consKp, consKi, consKd);   //set the PID tunning parameters

  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ROTATION_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);  // Turn on the motor all the time
  Serial.println("Init done");
}

double prevOutput;
void loop() {
  angle = ReadRawAngle();
  Input = angle;  

  prevOutput = Output;
  myPID.Compute();
  analogWrite(PWM_PIN, Output);  //send the PWM Output value to the EPOS
  
  //Control the rotation direction of the pedal using the Output value
  //TODO: test this code block 
  if (prevOutput < Output) {
    rotation = HIGH;
  }else{
    rotation = LOW;
  }
  digitalWrite(ROTATION_PIN, rotation);    // HIGH goes back, LOW goes foward 

  //DEBUG PID output and rotation in Serial monitor
  Serial.print("PID Input:\t"); Serial.print(Input); Serial.print("PID Output:\t"); Serial.print(Output); Serial.print("Rotation:\t"); Serial.print(rotation); 
  Serial.println("");
  
  //Serial.print("°C\t angle:");  Serial.print(Input);   // FOR DEBUG ONLY 
  //Serial.print("°C\t output:");  Serial.print(Output);   // FOR DEBUG ONLY 
  //Serial.println("" );
//  if (Serial.available() > 0) {
//    char inChar = Serial.read();
//    if (inChar == 'a'){
//      Output = Output - 1;
//      analogWrite(PWM_PIN, Output);  //send the PWM to the epos
//      delay(1000);
//      Input = ReadRawAngle();  //getting angle from the sensor 
//      delay(1000);
//      
//      Serial.print(Input, DEC); Serial.print(","); Serial.print(Output,DEC);
//      Serial.println("");    
//    }
//  }
  
}
