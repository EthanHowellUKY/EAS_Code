
#include "EAS.h"

/*============================================================================*/
   //------------------------------------------------------------------------//
  //                         * Neighbor Satellite *                        //
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

void nSAT::getVelocity(VL53L0X* sens) {
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
     *        float sat_velocity
     *          The estimated relative velocity between this
     *          satellite and the next. Units are in m/s
     */
     float vel_avg = 0.0;
     float dist_avg = 0.0;
     int curIdx;
     for(int ii=2; ii<numSamples+2; ++ii) {
       curIdx = ii % 2;
       dist_hist[curIdx] = fitData(sens->readReg16Bit(sens->RESULT_RANGE_STATUS + 10));
       dist_time[curIdx] = (float)millis()/1000.00;

       dist_avg += dist_hist[curIdx];
       vel_avg += (dist_hist[curIdx] - dist_hist[!curIdx])
                 / (dist_time[curIdx] - dist_time[!curIdx]);
       delay(sampleTime);
     }

     idx = jj % 2; // Use the modulus as index so we don't
                   // continually have to reset arrays
     vel_hist[idx] = vel_avg / numSamples;
     distance = dist_avg / numSamples;

     // Saturate if moving too fast
     if(abs(vel_hist[idx]) > 0.5) {
       vel_hist[idx] = vel_hist[!idx];
     }

     // Uncomment if filtering is needed
     //velocity_final[idx] = a * velocity_final[!idx] + (1-a) * vel_hist[idx];
     velocity_final[idx] = vel_hist[idx];

     // If velocity is small enough, saturate it to smooth control
     if( abs(velocity_final[idx]) <= 0.001 ) {
       sat_velocity = 0;
     }
     else {
       sat_velocity = velocity_final[idx];
     }

     velocity = velocity_final[idx]; // Store the new velocity
     ++jj; // Increment so that our modulus indexing will continue
     //dist = dist_avg;
     //dt = dtavg;

     return sat_velocity;
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

     //***********************//
     //comment says return A_v and A_d
     //can't return two values
     //***********************//
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
       pr->print(F("Time"));
       pr->print(',');
       //pr->print("dt1");
       //pr->print(',');
       //pr->print("dt2");
       //pr->print(',');
       pr->print("Distance");
       pr->print(',');
       //pr->print("d1");
       //pr->print(',');
       //pr->print("d2");
       //pr->print(',');
       pr->print("Velocity");
       pr->print(',');
       pr->print("SaturatedVelocity");
       pr->print(',');
       pr->print("Amplitude");
       pr->print(',');
       pr->print("AmplitudeDigital");
       pr->println();
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
     pr->print((millis()-exp_start)/1000.00,8);  // Current Time
     pr->print(',');
     //pr->print(sat->sat1.dt1,8);
     //pr->print(',');
     //pr->print(sat->sat1.dt2,8);
     //pr->print(',');
     pr->print(sat->sat1.dist,8);    // Distance
     pr->print(',');
     //pr->print(sat->sat1.d1,8);
     //pr->print(',');
     //pr->print(sat->sat1.d2,8);
     //pr->print(',');
     pr->print(sat->sat1.vel,8);     // Velocity
     pr->print(',');
     pr->print(sat->sat1.sat_velocity,8);  // Saturated Velocity
     pr->print(',');
     pr->print(sat->sat1.A_v,8);     // Voltage Amplitude
     pr->print(',');
     pr->print(sat->sat1.A_d,8);     // Digital Amplitude
     pr->println();
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
