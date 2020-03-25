/*
    Small Satellite Position Control Software
    Filename:       EAS.h
    Author:         Ethan Howell
    Date Created:   02/17/2020

    Version:        0.0.1
*/
#ifndef EAS_H
#define EAS_H

#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SPI.h>
#include "SdFat.h"
#include "Arduino.h"
#include "math.h"
#include <VL53L0X.h>
#include <Wire.h>

#define DFLT_NUM_SATS 5

struct Measurement_Data_t {
    float distance = 0.0;
    float velocity = 0.0;
    float sat_velocity = 0.0;
    float volt_amplitude = 0.0;
    float digi_amplitude = 0.0;
};

class nSAT {

private:
    int shtPins[DFLT_NUM_SATS] = {0};
    int addr[DFLT_NUM_SATS] = {0};
    int numSat = DFLT_NUM_SATS;

    int numSamples;
    int updateFreq;
    int numAvgs = 2;
    float sampleTime;

    float K[3] = {0};
    float filtConst = 0.97;

    Measurement_Data_t sat_t;

    VL53L0X sens;
    VL53L0X *sensors[DFLT_NUM_SATS] = { &sens };

    void temp();

public:
    void setAddress(int _xsht[], int _addr[], int _numSats);

    void setSampling(int _samp, int _freq);

    void setGains(int _gains[3]);

    void setFilterGain(float a);

    void linDataFit(float meas);

    float getDistance(VL53L0X* sens);

    void getVelocity();

    void getFeedback();

};

class SAT {

private:
    SdFat sd;
    SdFile file;
    const uint8_t SD_CS_PIN = 4;

    void temp();

public:
    void getFeedback(nSAT *sats[], int numSats);

    void initSD(char fName[20]);

    void initLIDAR();

    void initialize(char fName[20], nSAT *sats[], int numSats);

    void writeHeader();

    void writeData();

};

/*
class iSat {

private:
    SdFat sd;
    SdFile file;

    static uint32_t startMicros;
    const uint8_t SD_CS_PIN = 4;      //
    const uint8_t S1ADDR = 32;
    const uint8_t S2ADDR = 35;
    const int XSHT1 = 6;
    const int XSHT2 = 7;
    float dist = 0.0;             // Measured relative distance
    float d1 = 0.0;
    float d2 = 0.0;
    float dt1 = 0.0;
    float dt2 = 0.0;
    float dt = 0.0;
    float dtavg = 0.0;
    float vel = 0.0;              // Approx relative velocity
    float velsat = 0.0;           // Velocity w/saturation
    float A[2] = {0.0};           // Voltage/Digital Amplitude
    float vel_hist[2] = {0.0};    // Cache of velocity measurement
    float velocity_final[2] = {0.0};
    float dist_hist[2] = {0.0};   // Cache of distance measurement
    float dist_time[2] = {0.0};   // Recorded read times;
    int jj = 2;                   // Iterator to determine idx
    int idx;                      // idx used to access Cache
    float beta;                   // Feedback variable,   revisit

    float gainKa = 28.5;
    float gainKr = 1.0;
    float gainKv = 1.0;
    float filterWt = 0.97;
    int numSamples = 50;
    int numAvgs = 2;
    int updateFreq = 10;

    struct selfID {
        uint8_t* SADDR;
        uint8_t XADDR;
        int* SHT;
    };

    float distance();

    void velocity();

    void feedback();

    // Initialize the LiDAR Sensor(s) at their provided ids
    void initLIDAR();

    // Initialize communication with the host XBee (or neighbors?)
    void initComms();

    // Initialize the sd file to prepare usage
    void initSD();

    // Write data to SD
    void printData(Print* pr);

    // Write header of SD file
    void printHeader(Print* pr);

public:

    // Initialize hardware components
    void initialize(char fName[10]);

    // Get the Feedback Amplitude for Closed loop control
    void getFeedback();

    // Write data to the SD card
    void writeFile();

    // Transmit to another Satellites
    void talk(uint8_t n_addr);

    // Receive from another Satellite
    void listen(uint8_t n_addr);

    // Set control gains
    void setGains(float _gains[3]);

    // Set the number of samples taken per averaging period
    void setNumSamples(int _numSamp);

    // Set the Freq. in Hz of which the signal is updated
    void setUpdateFreq(int _freq);

};

class nSat {

public:
    nSat(uint8_t xbee_addr);
    void changeAddr(uint8_t newAddr);
    void getAddr();

private:
    uint8_t XADDR;
};
*/

#endif
