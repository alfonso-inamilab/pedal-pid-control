// The magnetic angle sensor used for the task is AS5600. 
// the sample code and the connections are taken from here:
// https://curiousscientist.tech/blog/as5600-magnetic-position-encoder 

// CONNECTIONS 
// Arduino MEGA pin 21 SCL to magnetometer SCL pin
// Arduino MEGA pin 20 SDA to magnetometer SDA pin
// Arduino GND pin to magnetometer GND pin 
// Arduino MEGA 5V output to magnetometer VCC

// PROGRAM 
// The output will be printed on the Serial monitor
// The output will be the angle measured by the pedal movement
// just move the pedal and see the change in the Serial monitor

#include <Wire.h>  //This is for i2C comms with the sensor

// Internal variables for the checkMagnetPresence() and ReadRawAngle functions
int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle;   //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
int magnetStatus = 0; //value of the status register (MD, ML, MH)

// Program variables
float angle; // saves the angle value on every loop

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

  //OPTIONAL
  //checkMagnetPresence(); //check the magnet presence (blocks until a magnet is found)

  Serial.println("Init done");
}

void loop() {
  angle = ReadRawAngle();  //getting angle from the sensor 
  Serial.print("°C\t angle:");  Serial.print(angle);  //print angle in Serial port
  Serial.println("");
}
