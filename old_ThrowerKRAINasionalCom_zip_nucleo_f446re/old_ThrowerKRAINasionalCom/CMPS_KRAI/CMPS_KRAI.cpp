/**
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

/**
 * Includes
 */
#include "CMPS_KRAI.h"

CMPS_KRAI::CMPS_KRAI(PinName sda, PinName scl, int address) {
    i2c = new I2C(sda, scl);
    //CMPS11 maksimum 100kHz CMPS12 maksimum 400kHz
    i2c->frequency(100000);
    i2cAddress = address;

}

char CMPS_KRAI::readSoftwareRevision(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char registerContents = 0;

    //First, send the number of register we wish to read,
    //in this case, command register, number 0.
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Now, read one byte, which will be the contents of the command register.
    i2c->read(i2cAddress, &registerContents, 1);
    
    return registerContents; 
}

int CMPS_KRAI::readBearing(void){
    char registerNumber = COMPASS_BEARING_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    int bearing = ((int)registerContents[0] << 8) | ((int)registerContents[1]);
    
    return bearing;   
}
void CMPS_KRAI::calibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF0;
    char calibrate_data2 = 0xF5;
    char calibrate_data3 = 0xF7;
    //kirim data 1
    Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
    //kirim data 2 delay 25ms
    Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data2, 1);
    //kirim data 3 delay 25ms
    Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data3, 1);
}

void CMPS_KRAI::stopCalibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF8;
    //kirim data 1
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
}