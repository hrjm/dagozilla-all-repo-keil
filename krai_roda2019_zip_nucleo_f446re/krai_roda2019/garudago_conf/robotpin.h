/*Garudago ITB 2019 Attributes*/

#ifndef ROBOTPIN_H
#define ROBOTPIN_H

#include "mbed.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include "Motor.h"
#include "encoderHAL.h"
#include <garudago_hardware_interface/JoystickPS3.h>

#define PIN_OPTO_3      PB_9                            //Extension shagai      1 -> Memendek
#define PIN_OPTO_4      PB_8//PB_6                            //Penaik gerege         1 -> Naik
#define PIN_OPTO_5      PB_4//LED Mati                  //Pelontar shagai       1 -> Panjang
#define PIN_OPTO_7      PC_8                            //Pencapit gerege       1 -> Buka
#define PIN_OPTO_9      PC_5 //PB_8//PA_9  //PC_5                         //Pembelok gerege       1 -> Lurus ///////////opto1
#define PIN_OPTO_11     PB_2                            //Pengambil shagai      1 -> Tutup

//Serial pin for UART Arduino
#define PIN_TX              PA_0
#define PIN_RX              PA_1

//Serial Initiate
//Serial pc(USBTX, USBRX, 115200);
joysticknucleo stick(PIN_TX, PIN_RX);

//Compass
CMPS12_KRAI compass_sensor(PC_9,PA_8, 0xC0);

//Internal Encoder From Motor using interrupt
encoderKRAI enc_right_top(PC_10, PC_13, 537.6, encoderKRAI::X4_ENCODING);
encoderKRAI enc_left_top(PC_12,  PC_11,  537.6, encoderKRAI::X4_ENCODING);
encoderKRAI enc_left_back(PC_3, PC_2, 537.6, encoderKRAI::X4_ENCODING);
encoderKRAI enc_right_back(PC_15, PC_14, 537.6, encoderKRAI::X4_ENCODING);
encoderKRAI enc_shagai(PC_6, PC_7, 538, encoderKRAI::X4_ENCODING);

//input capture encoder
// encoderHAL enc2(TIM4);
// encoderHAL enc3(TIM2);
// encoderHAL enc1(TIM3);

Motor motor_right_top(PA_11,PA_12,PB_12);
Motor motor_left_top(PA_7,PA_5,  PA_6);
Motor motor_left_back(PB_0,PC_1, PC_0);
Motor motor_right_back(PB_1,PB_14, PB_15);
Motor motor_shagai(PA_15, PA_13, PA_14 );

//Pneumatic
DigitalOut  pneumatic_extension (PIN_OPTO_3);
DigitalOut  pneumatic_penaik    (PIN_OPTO_4);
DigitalOut  pneumatic_pelontar  (PIN_OPTO_5);
DigitalOut  pneumatic_pencapit  (PIN_OPTO_7);
DigitalOut  pneumatic_pembelok  (PIN_OPTO_9);
DigitalOut  pneumatic_pengambil (PIN_OPTO_11);

#endif