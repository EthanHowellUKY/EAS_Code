/*
   Small Satellite Position Control Software.
   Filename: olsat2.ino
   Author: Ajin Sunny
   Last Modified by: Ethan Howell


   Written for Thesis: Single-Degree-of-Freedom Experiments Demonstrating
                       Electromagnetic Formation Flying for Small Satellite Swarms Using
                       Piecewise-Sinusoidal Controls
   Version: 2.0
   Date: 02-25-2019
   Last Updated: 03-01-2020

*/

//HEADER FILES
#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SPI.h>
#include "SdFat.h"
#include "Arduino.h"
#include "math.h"
#include <VL53L0X.h>
#include <Wire.h>

/*===========================================================*/
    // --------------------------------------------------- //
   //                 * MISC VARIABLES *                  //
  // --------------------------------------------------- //
/*===========================================================*/
unsigned long period = 50000;
unsigned int startime = 0;
unsigned int endtime = 0;
unsigned int exp_start;
const float c = 8.5;
double k2a = 28.5;
float kr = 1;
float kv = 1;
double desired_dist = 0.350;
double Amplitude = 0.00;
unsigned int i = 1;
unsigned int j = 1;
unsigned int k = 1;
double a = 0.97;
/*===========================================================*/
    // --------------------------------------------------- //
   //                 * VL53L0X DEFINITIONS *             //
  // --------------------------------------------------- //
/*===========================================================*/
#define SHT 7

VL53L0X sens;
#define SADDR 32

struct nSat {
  double dist = 0.0;
  double vel = 0.0;
  double velsat = 0.0;
  double A_v = 0.0;
  double A_d = 0.0;
  double vel_hist[9] = {0.0};
  double dist_hist[8] = {0.0};
  double dist_time[8] = {0.0};
  double velocity_final[4] = {0.0};
  int i = 1;
  int j = 1;
  double beta;
  /*=========================================================*/
      // ------------------------------------------------- //
     //          * RELATIVE DISTANCE AVERAGING *          //
    // ------------------------------------------------- //
  /*=========================================================*/
  double sensordistRead(VL53L0X* sens) {
    double sum = 0;
    double final_relative_dist = 0;
  
    //sens->rangingTest(meas, false);
  
    for(int kk=0; kk<90; ++kk) {
      sum += sens->readReg16Bit(sens->RESULT_RANGE_STATUS + 10); // Range in meters + offset
    }
    final_relative_dist = ((sum/1000)+0.210612583)/2000.00;
    
    return final_relative_dist;
  }
  /*=========================================================*/
      // ------------------------------------------------- //
     //             * VELOCITY ESTIMATION *               //
    // ------------------------------------------------- //
  /*=========================================================*/
  void velocity_func(VL53L0X* sens) {
    dist_hist[i] = sensordistRead(sens);
    dist = dist_hist[i];
    dist_time[i] = (double)millis() / 1000;
  
    vel_hist[j] = (dist_hist[i] - dist_hist[i-1]) / (dist_time[i] - dist_time[i-1]);
  
    if(vel_hist[j] > 0.50 | vel_hist[j] < -0.50) {
      vel_hist[j] = vel_hist[j-1];
    }
    velocity_final[i] = a * velocity_final[i-1] + (1-a) * vel_hist[j];
    vel = velocity_final[i];
    if(abs(velocity_final[i]) <= 0.01) {
      velsat = 0;
    }
    else {
      velsat = velocity_final[i];
    }
  
    if (i == 3) {
      dist_hist[0] = dist_hist[i];    //shifts the array back to the 0th element of the array.
      dist_time[0] = dist_time[i];
      velocity_final[0] = velocity_final[i];
      i = 0;                // sets the counter back to the first position.
      j = 1;
    }
    i++;
    j++;
  }
  /*=========================================================*/
      // ------------------------------------------------- //
     //              * FEEDBACK ALGORITHM *               //
    // ------------------------------------------------- //
  /*=========================================================*/
  void feedback_algorithm() {
    beta = (tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel));
    if(beta > 0) {
      Amplitude = k2a * pow(dist,2) * (pow(abs(beta),0.5));
    }
    else {
      Amplitude = -1 * k2a * pow(dist,2) * (pow(abs(beta),0.5));
    }
  
    if(Amplitude > 3.50) {
      A_v = 3.50;
    }
    else if(Amplitude < -3.50) {
      A_v = -3.50;
    }
    else {
      A_v = Amplitude;
    }
    
    A_d = (A_v*490)/2.75;
  }
};

// Top level struct containing data for N Satellites
struct satdat {
  double ms;
  nSat sat1;
} sat;
/*===========================================================*/
    // --------------------------------------------------- //
   //                 * SD DEFINITIONS *                  //
  // --------------------------------------------------- //
/*===========================================================*/
SdFat sd;                         //
SdFile file;                      //
const uint8_t SD_CS_PIN = 4;      //
char filename[20] = "sat2.csv";   //
/*===========================================================*/
    // --------------------------------------------------- //
   //           * RELATIVE DISTANCE AVERAGING *           //
  // --------------------------------------------------- //
/*===========================================================*/
/*
double sensordistRead(VL53L0X* sens) {
  double sum = 0;
  double final_relative_dist = 0;

  //sens->rangingTest(meas, false);

  for(int kk=0; kk<90; ++kk) {
    sum += sens->readReg16Bit(sens->RESULT_RANGE_STATUS + 10); // Range in meters + offset
  }
  final_relative_dist = ((sum/1000)+0.210612583)/2000.00;
  return final_relative_dist;
}
*/
/*===========================================================*/
    // --------------------------------------------------- //
   //              * VELOCITY ESTIMATION *                //
  // --------------------------------------------------- //
/*===========================================================*/
/*
void velocity_func(nSat &sat_t, VL53L0X* sens) {
  sat_t.dist_hist[sat_t.i] = sensordistRead(sens);
  sat_t.dist = sat_t.dist_hist[sat_t.i];
  sat_t.dist_time[sat_t.i] = (double)millis()/1000;

  sat_t.vel_hist[sat_t.j] = (sat_t.dist_hist[sat_t.i]-sat_t.dist_hist[sat_t.i-1])/(sat_t.dist_time[sat_t.i]-sat_t.dist_time[sat_t.i-1]);

  if(sat_t.vel_hist[sat_t.j] > 0.50 | sat_t.vel_hist[sat_t.j] < -0.50) {
    sat_t.vel_hist[sat_t.j]=sat_t.vel_hist[sat_t.j-1];
  }
  sat_t.velocity_final[sat_t.i] = a*sat_t.velocity_final[sat_t.i-1] + (1-a)*sat_t.vel_hist[sat_t.j];
  sat_t.vel = sat_t.velocity_final[sat_t.i];
  if(abs(sat_t.velocity_final[sat_t.i]) <= 0.01) {
    sat_t.velsat = 0;
  }
  else {
    sat_t.velsat = sat_t.velocity_final[sat_t.i];
  }

  if (sat_t.i == 3) {
    sat_t.dist_hist[0]=sat_t.dist_hist[sat_t.i];    //shifts the array back to the 0th element of the array.
    sat_t.dist_time[0] = sat_t.dist_time[sat_t.i];
    sat_t.velocity_final[0] = sat_t.velocity_final[sat_t.i];
    sat_t.i = 0;                // sets the counter back to the first position.
    sat_t.j = 1;
  }
  sat_t.i++;
  sat_t.j++;
}
*/
/*===========================================================*/
    // --------------------------------------------------- //
   //               * FEEDBACK ALGORITHM *                //
  // --------------------------------------------------- //
/*===========================================================*/
/*
void feedback_algorithm(nSat &sat_t) {
  if((tanh(kr * (sat_t.dist - desired_dist)) + c*tanh(kv * sat_t.vel)) > 0) {
    Amplitude = k2a * pow(sat_t.dist,2) * (pow(abs(tanh(kr * (sat_t.dist - desired_dist)) + c*tanh(kv * sat_t.vel)),0.5));
  }
  else {
    Amplitude = -1 * k2a * pow(sat_t.dist,2) * (pow(abs(tanh(kr * (sat_t.dist - desired_dist)) + c*tanh(kv * sat_t.vel)),0.5));
  }

  if(Amplitude > 3.50) {
    sat_t.A_v = 3.50;
  }
  else if(Amplitude < -3.50) {
    sat_t.A_v = -3.50;
  }
  else {
    sat_t.A_v = Amplitude;
  }
  sat_t.A_d = (sat_t.A_v*490)/2.75;
}
*/
/*===========================================================*/
    // --------------------------------------------------- //
   //                * WRITE FILE HEADER *                //
  // --------------------------------------------------- //
/*===========================================================*/
void writeHeader(Print* pr) {
  pr->print(F("millis"));
  pr->write(',');
  pr->print("dist1");
  pr->write(',');
  pr->print("vel1");
  pr->write(',');
  pr->print("velsat1");
  pr->write(',');
  pr->print("ad1");
  pr->write(',');
  pr->print("av1");
  pr->println();
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                * WRITE DATA TO FILE *               //
  // --------------------------------------------------- //
/*===========================================================*/
void printData(Print* pr, satdat* sat) {
  pr->print((millis()-exp_start)/1000.00,8);
  pr->write(',');
  pr->print(sat->sat1.dist,8);
  pr->write(',');
  pr->print(sat->sat1.vel,8);
  pr->write(',');
  pr->print(sat->sat1.velsat,8);
  pr->write(',');
  pr->print(sat->sat1.A_d,8);
  pr->write(',');
  pr->print(sat->sat1.A_v,8);
  pr->println();
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                  * INIT VL53L0X *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void initLIDAR() {
  //
  pinMode(SHT, OUTPUT);

  Wire.begin(); //

  pinMode(SHT, INPUT); //
  delay(10);
  sens.setAddress(SADDR); //
  
  //
  if(!sens.init()) {
    Serial.println("Satellite 2, Sensor Failed");
    while(1){}
  }

  //
  sens.setTimeout(500);

  //
  sens.startContinuous(0);
  /*
  i2c = &Wire;
  pinMode(SHT, OUTPUT);
  digitalWrite(SHT, LOW);

  delay(10);

  digitalWrite(SHT, HIGH);
  if(!sens.begin(SADDR, DBG, i2c)) {
    Serial.println("Failed to boot SAT2 VL53L0X");
    while(1);
  }
  */
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                  * INIT SD FILE *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void initSD() {

  // Initialize SD pin to open comms between DUE and the SD Shield
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  // Try to initialize SD card at 50MHz clk
  // if failed wait because SD may not yet be inserted
  Serial.println("Initializing SD card...");
  while (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    Serial.println("Card failed, or not present...");
    delay(10000);
  }

  // Open the SD file with read/write permissions
  while(!file.open(filename, O_RDWR | O_CREAT | O_TRUNC)){
    delay(10);
  }

  // Write the header of the csv file
  writeHeader(&file);

  // Relay to user that initilization is complete
  Serial.println("card initialized.");
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                       * SETUP *                     //
  // --------------------------------------------------- //
/*===========================================================*/
void setup() {

  analogReadResolution(10);
  analogWriteResolution(10);

  Serial.begin(115200);
  while (!Serial) {
    SysCall::yield();
  }
  delay(400);
  
  initLIDAR();  // INITIALIZE SENSOR(s)         //
  initSD();     // INITIALIZE SD CARD AND FILE  //

  // WAIT FOR USER INPUT TO BEGIN EXPERIMENT //
  Serial.println("Satellite 2 ready, awaiting input.");
  while(Serial.available() == 0) {}
  while(Serial.read() != 'A') {}
  Serial.write("Starting..");
  //S.setAmplitude(-1*S.returnAmplitude()); // Uncomment for closed loop attraction
  // BEGIN EXPERIMENT //
  exp_start = millis();
  while((millis()-exp_start) < period) {
    S.startSinusoid1(10);
    if(file.isOpen()) {
      Serial.print("Start: ");
      startime = millis();
      Serial.print((startime-exp_start)/1000.00);
      Serial.print('\t');

      //velocity_func(sat.sat1, &sens, &meas);
      sat.sat1.velocity_func(&sens);

      printData(&file, &sat);

      endtime = millis();

      Serial.print("End: ");
      Serial.print((endtime-exp_start)/1000.00);
      Serial.print('\t');
      Serial.print("Diff1: ");
      Serial.println((endtime-startime)/1000.00);
      if((endtime-startime) < 100) {
        delay(100-(endtime-startime));
        //feedback_algorithm(sat.sat1);
        sat.sat1.feedback_algorithm();
        unsigned int endtime2 = millis();
      }
      else {
        //feedback_algorithm(sat.sat1);
        sat.sat1.feedback_algorithm();
      }
    }
    S.stopSinusoid();
  }
  file.close();
  Serial.println("Finished");
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                     * MAIN LOOP *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void loop() {}
