
#include "EAS.h"

/*==============================*/
   //--------------------------//
  //    * Self Satellite *    //
 //--------------------------//
/*==============================*/

void iSat::initialize(char fName[10]);

void iSat::getPosition() {

}

void iSat::getVelocity() {

}

void iSat::writeFile() {

}

void iSat::talk() {

}

void iSat::listen() {

}

void iSat::initLIDAR() {

}

void iSat::initComms() {

}

void iSat::initSD() {

}

void iSat::printData(Print* pr) {

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
