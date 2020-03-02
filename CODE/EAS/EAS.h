
#ifndef EAS_H
#define EAS_H

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"

boolean DBG = false;
TwoWire* i2c;

/*
template <typename T>
    struct counter {
        counter() {
            objects_created++;
            objects_alive++;
        }
        counter(const counter&) {
            objects_created++;
            abjects_alive++;
        }
    protected:
        virtual ~counter() {
            --objects_alive;
        }
        static int objects_created;
        static int objects_alive;
    };
    template <typename T> int counter<T>::objects_created(0);
    template <typename T> int counter<T>::objects_alive(0);
*/

class iSat {

public:
    // Init Sensor with LiDAR I2C address and onboard XBEE MAC address
    iSat(uint8_t* sensAddrs, uint8_t xbAddr, int* shtPins);

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

    // Structure to store the necessary satellite data
    struct nSat_t {
        double dist = 0.0;
        double vel = 0.0;
        double velSat = 0.0;
        double Av = 0.0;
        double Ad = 0.0;
    };

private:
    static uint32_t startMicros;

    struct selfID {
        uint8_t* SADDR;
        uint8_t XADDR;
        int* SHT;
    };

    // Initialize the LiDAR Sensor(s) at their provided ids
    void initLIDAR();

    // Initialize communication with the host XBee (or neighbors?)
    void initComms();

    // Convert the written binary file to CSV
    void binaryToCsv();

    // Create the binary file to save data
    void createBinFile();

    // Procedure to collect and write data to the SD
    void logData();

    // Collect data to be written to the SD
    void recordBinFile();

    // Rename the SD file if already exists
    void renameBinFile();

    // Write data to SD
    void printData(Print* pr, SatDat* data);

    // Sensor data collection
    void acquireData(SatDat* data);

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
