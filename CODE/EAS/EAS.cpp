
#include "EAS.h"

  //--------------------------//
 //    * Self Satellite *    //
//--------------------------//

void iSat::iSat(uint8_t* sensAddrs, uint8_t xbAddr, int* shtPins) {
    selfID ID;
    ID.SADDR = sensAddrs;
    ID.XADDR = xbAddr;
    ID.SHT = shtPins;
}

void iSat::initLIDAR() {
    i2c = &Wire;

}

void iSat::getPosition() {

}

void iSat::getVelocity() {

}

void iSat::writeFile() {

}

void iSat::initComms() {

}

void iSat::talk() {

}

void iSat::listen() {

}

void iSat:: binaryToCsv() {

}

void iSat::createBinFile() {

}

void iSat::logData() {

}

void iSat::recordBinFile() {

}

void iSat::renameBinFile() {

}

void iSat::printData(Print* pr, SatDat* data) {

}

void iSat::acquireData(SatDat* data) {

}

void iSat::printHeader(Print* pr) {

}

  //--------------------------//
 //  * Neighbor Satellite *  //
//--------------------------//

void nsat::changeAddr(uint8_t newAddr) {
    XADDR = newAddr;
}

uint8_t nSat::getAddr() {
    return XADDR;
}
