/**
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

#ifndef CMPS_DAGOZ_H
#define CMPS_DAGOZ_H

/**
 * Includes
 */
#include "mbed.h"

#define CMPS_DEFAULT_I2C_ADDRESS    0xC0

// Register Addresses
#define SOFTWARE_REVISION_REG       0x0
#define COMPASS_BEARING_WORD_REG    0x2
#define COMPASS_BEARING_BOSCH_REG   0x1A
#define COMPASS_CALIBRATION_STATUS  0x1E

// Inisiasi I2C komunikasi dari kompas
class CMPS_DAGOZ {

    I2C* i2c;
    int  i2cAddress;

public:

    /**
     * Constructor.
     *
     * @param sda mbed pin to use for I2C SDA
     * @param scl mbed pin to use for I2C SCL
     * @param address I2C address of this device.
     */
    CMPS_DAGOZ(PinName sda, PinName scl, int address);

    /**
     * Membaca revisi dari kompas
     */
    char readSoftwareRevision(void);

    /**
     * Membaca nilai dari kompas
     * Range nilai 0 - 3599
     */
    int readBearing(void);

    /**
     * Membaca nilai heading IMU hitungan Bosch
     * Range nilai 0 - 5759
     */
    int readBearingBosch(void);

    /**
     * Memeriksa kondisi kalibrasi CMPS12
     * Range nilai 0 - 3
     */
    int getCalibrationStatus(void);

};

#endif /* CMPS_DAGOZ_H */
