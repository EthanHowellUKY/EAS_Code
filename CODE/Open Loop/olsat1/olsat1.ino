/*
   Small Satellite Position Control Software.
   Filename: olsat1.ino
   Author: Ajin Sunny
   Last Modified by: Ethan Howell


   Written for Thesis: Single-Degree-of-Freedom Experiments Demonstrating
                       Electromagnetic Formation Flying for Small Satellite Swarms Using
                       Piecewise-Sinusoidal Controls
   Version: 2.1
   Date Added: 02-25-2019
   Last Updated: 04-06-2020

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
unsigned long period = 10000;  // Experiment runtime in millis
unsigned int startime = 0;     // Begin time of data collection
unsigned int endtime = 0;      // End time of data collection
unsigned int exp_start;        // Start time of experiment
const double c = 8.5;          // Constant in feedback,  revisit
double ka = 28.5;              // Gain, not sure,        revisit
double kr = 1;                 // Gain, not sure,        revisit
double kv = 1;                 // Gain, not sure,        revisit
double desired_dist = 0.350;   // Desired relative distance, (m)
double a = 0.85;               // Filtering constant,    revisit
const int numSamples = 40;     // Number of distance samples
const int updateFreq = 10;     // Update frequency of feedback
const int sinFreq = 10;        // Desired frequency of sine wave

// Caclulate the sampling time based on the user defined params
double sampleTime = (1000.0/updateFreq) / (numSamples);
/*===========================================================*/
    // --------------------------------------------------- //
   //                 * VL53L0X DEFINITIONS *             //
  // --------------------------------------------------- //
/*===========================================================*/
#define SHT 7     // Shutoff pin to VL53L0X sensor

VL53L0X sens;     // Pololu VL53L0X sensor object
#define SADDR 32  // New I2C sensor address to R/W

struct nSat {
  /*
   *      Structure containing all data necessary to control 1
   *      satellite. All data is local to that satellite for
   *      optimality.
   */
  double dist = 0.0;             // Measured relative distance
  double dt = 0.0;
  double vel = 0.0;              // Approx relative velocity
  double velsat = 0.0;           // Velocity w/saturation
  double A_v = 0.0;              // Voltage Amplitude
  double A_d = 0.0;              // Digital Amplitude
  double vel_hist[2] = {0.0};    // Cache of velocity measurement
  double velocity_final[2] = {0.0};
  double dist_hist[2] = {0.0};   // Cache of distance measurement
  double dist_time[2] = {0.0};   // Recorded read times;
  int jj = 2;                    // Iterator to determine idx
  int idx;                       // idx used to access Cache
  double beta;                   // Feedback variable,   revisit
  /*=========================================================*/
      // ------------------------------------------------- //
     //        * LINEAR FIT OF RELATIVE DISTANCE *        //
    // ------------------------------------------------- //
  /*=========================================================*/
  float fitData(float meas) {
    /*
     *      Curve fit collected position data to the desired
     *      scale. This equation was generated using a linear
     *      fit in Excel.
     *
     *      Inputs
     *      -------
     *        float meas
     *          Averaged distance measurement taken from
     *          Pololu VL53L0X. Units in mm.
     *
     *      Returns
     *      -------
     *        float fitData()
     *          Converted distance value to referenced values
     *          between 20-150mm. Units in m.
     */
    return (0.6427 * meas + 5.7245) / 1000.00 + 0.2;
  }
  /*=========================================================*/
      // ------------------------------------------------- //
     //          * RELATIVE DISTANCE AVERAGING *          //
    // ------------------------------------------------- //
  /*=========================================================*/
  float sensorDistRead(VL53L0X* sens) {
    /*
     *      Read raw data from the VL53L0X sensor, and average
     *      over a given number of samples.
     *
     *      Inputs
     *      -------
     *        VL53L0X* sens
     *          Pointer to the Pololu sensor object
     *
     *      Returns
     *      -------
     *        float final_relative_dist
     *          The averaged relative distance between this
     *          satellite and the next. Units are in mm
     */
    float sum = 0.0;
    float final_relative_dist = 0.0;
    float avg_dist = 0.0;

    for(int kk=0; kk<numSamples; ++kk) {
      sum += sens->readReg16Bit(sens->RESULT_RANGE_STATUS + 10); // Range in meters + offset
      delay(sampleTime);
    }
    avg_dist = sum/numSamples;
    final_relative_dist = fitData(avg_dist);

    return final_relative_dist;
  }
  /*=========================================================*/
      // ------------------------------------------------- //
     //             * VELOCITY ESTIMATION *               //
    // ------------------------------------------------- //
  /*=========================================================*/
  void velocity_func(VL53L0X* sens) {
    /*
     *      A finite difference method which uses the average
     *      distance over a given number of samples to
     *      approximate the relative velocity between satellites.
     *
     *      Inputs
     *      -------
     *        VL53L0X* sens
     *          Pointer to the Pololu sensor object
     *
     *      Returns
     *      -------
     *        float velsat
     *          The estimated relative velocity between this
     *          satellite and the next. Units are in m/s
     */

    double dist_avg = 0.0;
    double dtavg = 0.0;
    dist_avg = sensorDistRead(sens);
    dtavg = (double)millis()/1000.00;

    if(dist_avg >= 0.9) {
      dist_avg = dist;
    }

    idx = jj % 2;

    vel_hist[idx] = (dist_avg - dist) / (dtavg - dt);
    // Saturate if moving too fast
    if(abs(vel_hist[idx]) > 0.5) {
      vel_hist[idx] = vel_hist[!idx];
    }

    // Uncomment if filtering is needed
    velocity_final[idx] = a * velocity_final[!idx] + (1-a) * vel_hist[idx];

    // If velocity is small enough, saturate it to smooth control
    if( abs(velocity_final[idx]) <= 0.001 ) {
      velsat = 0;
    }
    else {
      velsat = velocity_final[idx];
    }

    vel = velocity_final[idx]; // Store the new velocity
    ++jj; // Increment so that our modulus indexing will continue
    dist = dist_avg;
    dt = dtavg;
  }
  /*=========================================================*/
      // ------------------------------------------------- //
     //              * FEEDBACK ALGORITHM *               //
    // ------------------------------------------------- //
  /*=========================================================*/
  void feedback_algorithm() {
    /*
     *      Satellite feedback control algorithm. This algorithm
     *      computes the amplitude of the sine wave necessary to
     *      minimize the error between our current estimated
     *      relative distance, and desired relative distance.
     *
     *      Inputs
     *      -------
     *        None
     *
     *      Returns
     *      -------
     *        float A_v
     *          Amplitude in volts that we are sending to the
     *          Copley Controls amplifier. This amplifier then
     *          converts the voltage amplitude to a current
     *          amplitude proportional to some scaling factor.
     *          Units are in Volts.
     *
     *        float A_d
     *          Digital Amplitude value to be passed into the
     *          sine wave generation function. Value is
     *          proportional to the voltage amplitude by some
     *          constant.
     */
    float Amplitude;
    beta = (tanh(kr * (dist - desired_dist)) + c*tanh(kv * velsat));
    if(beta > 0) {
      Amplitude = ka * pow(dist,2) * (pow(abs(beta),0.5));
    }
    else {
      Amplitude = -1 * ka * pow(dist,2) * (pow(abs(beta),0.5));
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

// Top level struct containing data for N Satellites.
// Define as many nested structures as there are neighbor sats.
struct satdat {
  float ms;
  nSat sat1;
} sat;
/*===========================================================*/
    // --------------------------------------------------- //
   //                 * SD DEFINITIONS *                  //
  // --------------------------------------------------- //
/*===========================================================*/
SdFat sd;                         // FileSystem object of SD
SdFile file;                      // SD file object to R/W
const uint8_t SD_CS_PIN = 4;      // SD shield access pin
char filename[20] = "sat1.csv";   // Filename to save data
/*===========================================================*/
    // --------------------------------------------------- //
   //                * WRITE FILE HEADER *                //
  // --------------------------------------------------- //
/*===========================================================*/
void writeHeader(Print* pr) {
  /*
   *      Write the csv file header
   *
   *      Inputs
   *      -------
   *        Print* pr
   *          Pointer to printable object, i.e. Serial, SD, etc.
   *
   *      Returns
   *      -------
   *        None
   */
  pr->print(F("Time"));
  pr->print(',');
  pr->print("Distance");
  pr->print(',');
  pr->print("Velocity");
  pr->print(',');
  pr->print("SaturatedVelocity");
  pr->print(',');
  pr->print("Amplitude");
  pr->print(',');
  pr->print("AmplitudeDigital");
  pr->println();
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                * WRITE DATA TO FILE *               //
  // --------------------------------------------------- //
/*===========================================================*/
void printData(Print* pr, satdat* sat) {
  /*
   *      Write measured data to the csv file
   *
   *      Inputs
   *      -------
   *        Print* pr
   *          Pointer to printable object, i.e. Serial, SD, etc.
   *
   *        satdat* sat
   *          Pointer to the structure containing individual
   *          satellite data.
   *
   *      Returns
   *      -------
   *        None
   */
  pr->print((millis()-exp_start)/1000.00,8);  // Current Time
  pr->print(',');
  pr->print(sat->sat1.dist,8);    // Distance
  pr->print(',');
  pr->print(sat->sat1.vel,8);     // Velocity
  pr->print(',');
  pr->print(sat->sat1.velsat,8);  // Saturated Velocity
  pr->print(',');
  pr->print(sat->sat1.A_v,8);     // Voltage Amplitude
  pr->print(',');
  pr->print(sat->sat1.A_d,8);     // Digital Amplitude
  pr->println();
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                  * INIT VL53L0X *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void initLIDAR() {
  /*
   *      Initialize the LIDAR sensor. Cycle the shutoff pin
   *      so that we can write a new address (redundant if only
   *      one I2C sensor is present).
   *
   *      Inputs
   *      -------
   *        None
   *
   *      Returns
   *      -------
   *        None
   */

  // Write low signal to shutoff pin to de-activate sensor
  pinMode(SHT, OUTPUT);

  Wire.begin(); // Initialize I2C bus

  // Write high signal to shutoff pin to re-activate sensor
  pinMode(SHT, INPUT);
  delay(10);

  // Set sensor address to user defined location
  sens.setAddress(SADDR);

  // Initialize sensor, alert user if init fails
  if(!sens.init()) {
    Serial.println("Satellite 1, Sensor Failed");
    while(1){}
  }

  // Set the timeout limit to 500 millis.
  sens.setTimeout(500);

  // Start continuous mode, take measurements as fast as possible
  sens.startContinuous(0);
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                  * INIT SD FILE *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void initSD() {
  /*
   *      Initialize the SD card and SD file to read and write
   *      data to SD.
   *
   *      Inputs
   *      -------
   *        None
   *
   *      Returns
   *      -------
   *        None
   */

   /*
    *   TODO: Need to add functionality to check if file is
    *         already present. If so, take appropriate measures.
    */

  // Initialize SD pin to open comms between DUE and SD Shield
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
  if(sd.exists(filename)) {
    sd.remove(filename);
  }
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
  /*
   *      Run the experiment. Since this experiment is a
   *      "one-off" run, this is implemented in the setup with
   *      a timer. Previous versions of this code implemented an
   *      "exit()" method. This method of ending the code is not
   *      recommended as there is no default OS to defer to on
   *      the Arduino.
   *
   *      Experiment begins by initializing the necessary
   *      external components (LIDAR, SD, etc.). User is then
   *      prompted to begin the experiment via key input over
   *      XBEE Series1 RF module.
   *
   *      Once begun, the experiment will collect measurements
   *      and write data to SD file until the runtime of the
   *      experiment is reached. At this time data is saved,
   *      and SD file closed.
   *
   *      Inputs
   *      -------
   *        None
   *
   *      Returns
   *      -------
   *        None
   */

  // Set the analog read and write resolution.
  // Default is 10-bits, but Due has 12-bit capability.
  // Likely don't need these here
  analogReadResolution(10);
  analogWriteResolution(10);

  // Open serial port at 115200 baud rate
  Serial.begin(115200);
  while (!Serial) {
    SysCall::yield();
  }
  delay(400);

  initLIDAR();  // INITIALIZE SENSOR(s)         //
  initSD();     // INITIALIZE SD CARD AND FILE  //

  // WAIT FOR USER INPUT TO BEGIN EXPERIMENT //
  Serial.println("Satellite 1 ready, awaiting input.");
  while(Serial.available() == 0) {}
  while(Serial.read() != 'A') {}
  Serial.println("Starting..");

  // Initial Distance and timestamp
  sat.sat1.dist = sat.sat1.sensorDistRead(&sens);
  sat.sat1.dt = (double)millis()/1000.00;
  sat.sat1.dist_hist[1] = sat.sat1.dist;
  sat.sat1.dist_time[1] = sat.sat1.dt;
  delay(100);

  // BEGIN EXPERIMENT //
  exp_start = millis();
  while((millis()-exp_start) < period) {

    // Generate a sine wave at 10Hz
    // O.L. Exp -> Defualt Amplitude
    // C.L. Exp -> Updated Amplitude
    S.startSinusoid1(10);

    // Verify that the SD file is open and capable to
    // write new data
    if(file.isOpen()) {

      // Get measurement/write cycle starttime & output to user
      Serial.print("Start: ");
      startime = millis();
      Serial.print((startime-exp_start)/1000.00);
      Serial.print('\t');

      // Call the velocity function to update the current
      // relative velocity of the satellites
      sat.sat1.velocity_func(&sens);

      // Write the newly collected data to the SD file
      printData(&file, &sat);

      // Get measurement/write cycle endtime & output to user
      endtime = millis();
      Serial.print("End: ");
      Serial.print((endtime-exp_start)/1000.00);
      Serial.print('\t');
      Serial.print("Diff: ");
      Serial.println((endtime-startime)/1000.00);

      // If meas/write finished within update period, wait until
      // the end of the period.
      if((endtime-startime) < 100) {
        delay(100-(endtime-startime));
      }

      // Implement feedback to determine necessary amplitude
      // of sinewave over the next period.
      sat.sat1.feedback_algorithm();
    }

    // Stop the sine wave so that we can restart
    // O.L. Exp -> Same Amplitude
    // C.L. Exp -> Updated Amplitude
    S.stopSinusoid();
  }

  // Close the SD file and save data for later analysis
  file.close();

  // Relay to user that experiment is complete
  Serial.println("Finished");
}
/*===========================================================*/
    // --------------------------------------------------- //
   //                     * MAIN LOOP *                   //
  // --------------------------------------------------- //
/*===========================================================*/
void loop() {}
