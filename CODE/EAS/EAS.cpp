
#include "EAS.h"

/*============================================================================*/
   //------------------------------------------------------------------------//
  //                         * Neighbor Satellite *                         //
 //------------------------------------------------------------------------//
/*============================================================================*/

void nSAT::setAddress(int _xsht[], int _addr[], int _numSats) {
    for(int ii = 0; ii < numSats; ++ii) {
        shtPins[ii] = _xsht[ii];
        addr[ii] = _addr[ii];
    }
}

void nSAT::setSampling(int _samp, int _freq) {
    numSamples = _samp;
    updateFreq = _freq;

    sampleTime = (1000.0/updateFreq) / (numAvgs * numSamples);
}

void nSAT::setGains(int _gains[3]) {
    K = gains;
}

void nSAT::linDataFit(float meas) {
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

void nSAT::setFilterGain(float a) {
    filtConst = a;
}

float nSAT::getDistance() {
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
        // Range in meters + offset
        sum += sens->readReg16Bit(sens->RESULT_RANGE_STATUS + 10);
        delay(sampleTime - 10);
    }
    avg_dist = sum/numSamples;
    final_relative_dist = fitData(avg_dist);

    return final_relative_dist;
}

void nSAT::getVelocity() {
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

}

void nSAT::getFeedback() {
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

}

/*============================================================================*/
   //------------------------------------------------------------------------//
  //                           * Self Satellite *                           //
 //------------------------------------------------------------------------//
/*============================================================================*/

void SAT::writeHeader() {
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

}

void SAT::writeData() {
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

}

void SAT::initSD(char fName[20]) {
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
    while(!file.open(fName, O_RDWR | O_CREAT | O_TRUNC)){
        delay(10);
    }

    // Write the header of the csv file
    writeHeader(&file);

    // Relay to user that initilization is complete
    Serial.println("card initialized.");

}

void SAT::initLIDAR() {
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

}

void SAT::initialize(char fName[20], nSAT *sats[], int numSats) {
    initLIDAR();
    initSD();
}

void SAT::getFeedback(nSAT *sats[], int numSats) {

}
