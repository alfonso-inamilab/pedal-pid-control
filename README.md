# Pedal Arduino control 
Samples to control the Performance Pedal with Arduino and AS5600 magnetic encoder

- mega-angle-reader.ino - Sample that prints the magnetic encoder (AS5600) data into Serial
- mega_escon_pwm_sample - Sample velocity control for MAXON motor with ESCON and Arduino  
- mega_pedal_control - PID controller sample for MAXON, ESCON and AS5600

## Hardware setup 

### **Accelerometer setup**
The magnetic encoder (AS5600) is located on the side of the pedal using a 3D printed stand. 

The 3D model of this stand can be found in 'models' folder.

The recomended magnet for this task is: https://www.amazon.co.jp/-/en/gp/product/B08XNFY3Y2/ref=ox_sc_act_title_1?smid=A1X8LI5R2IUBMO&psc=1

The magnet should be placed around 1 mm directly over the sensor dice (chip).

### **Connections with Arduino**  

**Conexions with ESCON motor controller**
| Arduino Mega | ESCON |
| ------------- | ------------- |
| PIN 44 (PWM)  | ESCON D1 (Vel. control) PWM  |
| PIN 3   | ESCON D2 (motor enable) high or low |
| PIN 4   | ESCON D3 (rotation direction) high or low  |
| GND   | ESCON D5  |

**Conexions with magnetic encoder**
| Arduino Mega | Magnetic Encoder |
| ------------- | ------------- |
| PIN 21 (SCL)   | SCL  |
| PIN 20 (SDA)   | SDA  |
| GND   | GND  |
| 5V   | VCC  |

## External links 
The PID controller libraries are taken from here: https://playground.arduino.cc/Code/PIDLibrary/
and this sample was followed: https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/

The magnetic angle encoder used for the task is AS5600. 
the sample code and the connections are taken from here:
https://curiousscientist.tech/blog/as5600-magnetic-position-encoder 

## TODO task 
Calibrate the paramters Kp, Ki and Kd of the PID controller using Ziegler–Nichols method:
- https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
- https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

