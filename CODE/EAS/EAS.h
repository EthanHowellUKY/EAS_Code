/*
    Small Satellite Position Control Software
    Filename:       EAS.h
    Author:         Ethan Howell
    Date Created:   02/17/2020

    Version:        0.0.1
*/
#ifndef EAS_H
#define EAS_H

#include "Arduino.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include "SdFat.h"

#ifdef MULTIPLEX
VL53L0X sens[1]
numLidar = 1
#else
VL53L0X sens[2]
numLidar = 2
#endif

class iSat {

public:

    // Initialize hardware components
    void initialize(char fName[10]);

    // Get the relative position between Satellites
    void getPosition();

    // Get the relative velocity between Satellites
    void getVelocity();

    // Write data to the SD card
    void writeFile();

    // Transmit to another Satellites
    void talk(uint8_t n_addr);

    // Receive from another Satellite
    void listen(uint8_t n_addr);

private:
    SdFat sd;
    SdFile file;

    static uint32_t startMicros;
    const uint8_t SD_CS_PIN = 4;      //
    const uint8_t S1ADDR = 32;
    const uint8_t S2ADDR = 35;
    const int XSHT1 = 6;
    const int XSHT2 = 7;
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
    const double alpha = 0.97;
    double beta;
    const double c = 8.5;
    const double ka = 28.5;
    const float kr = 1;
    const float kv = 1;

    struct selfID {
        uint8_t* SADDR;
        uint8_t XADDR;
        int* SHT;
    };

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
};

class nSat /*: counter<nSat>*/ {

public:
    nSat(uint8_t xbee_addr);
    void changeAddr(uint8_t newAddr);
    void getAddr();

private:
    uint8_t XADDR;
};

#endif
