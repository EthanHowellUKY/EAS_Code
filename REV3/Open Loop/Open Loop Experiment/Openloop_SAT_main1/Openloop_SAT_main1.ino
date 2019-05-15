#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"


char incomingByte;
DFRobotVL53L0X sensor;
unsigned long period = 30000; 
double dist[3]={0.0,0.0,0.0};
unsigned int i = 1;
double vel = 0.0;
double velocity = 0.0; 
double delta_pos = 0.0;
File myFile;

void setup()  
{ 
  analogReadResolution(10);
  analogWriteResolution(10);
  pinMode(9, OUTPUT);
  Serial.begin(115200);
  //delay(20000);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  

  Serial.print("Initializing SD card...");
  
    // see if the card is present and can be initialized:
    if (!SD.begin(4)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    Serial.println("card initialized.");

  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(Continuous, High);
  //Laser rangefinder begins to work
  sensor.start();
  myFile = SD.open("sat1.csv", FILE_WRITE);
  myFile.print("Distance");
  myFile.print(",");
  myFile.println("Velocity"); 
  while(Serial.available()==0){}
  incomingByte = Serial.read();
  if(incomingByte == 'A')
  {
  Serial.println(incomingByte);
  }

} 

void loop()  
{   
  while(millis() < period) 
  {
    if(myFile)
    {
    S.startSinusoid1(20);
    delay(87);
//    //Serial.print("Voltage value: ");
//    //Serial.println(S.return_voltage());
    dist[i] = (sensor.getDistance()/10)+20;
   // i++;
    vel = velocity_func(dist);
//    //Serial.print("Distance: ");
    Serial.println(dist[i]);
    myFile.print(dist[i]);
    myFile.print(",");
    i++;
    Serial.print("Velocity: ");
    Serial.println(vel);
    myFile.println(vel);
//    myFile.print(",");
//
//    
    if (i == 3)
    {
      dist[0] = dist[i-1];
      i = 1;
    }
//    
//    //delay(1000);
    S.stopSinusoid();
    }
  }
    myFile.close();
    exit(0);
} 



double velocity_func(double dist[2])
{
  delta_pos = dist[i] - dist[i - 1];
  velocity = delta_pos/0.016;
  return velocity;
}


  