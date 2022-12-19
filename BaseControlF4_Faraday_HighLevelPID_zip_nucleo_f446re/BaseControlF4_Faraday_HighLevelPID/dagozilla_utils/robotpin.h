#ifndef ROBOTPIN_H
#define ROBOTPIN_H

#include "CMPS_DAGOZ.h"
#include "EncoderDAGOZ.h"
#include "MotorDAGOZ.h"
#include "EncoderMotor.h"

/*****************************
        Pin Declaration
 *****************************/
//Encoder pin
EncoderDAGOZ locomotionEncL(TIM1);          //Locomotion Right Encoder
EncoderDAGOZ locomotionEncR(TIM3);          //Locomotion Left Encoder
EncoderDAGOZ locomotionEncB(TIM2);          //Locomotion Back Encoder

EncoderMotor intEncL(PD_2, PC_8, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor intEncR(PC_4, PB_13, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor intEncB(PA_11, PB_12, 537.6, EncoderMotor::X4_ENCODING);

//Motor pin buat Board Sistem next ver.
MotorDagoz locomotionMotorL(PH_1, PB_6);    //Locomotion Left Motor
MotorDagoz locomotionMotorR(PA_12, PB_7);   //Locomotion Right Motor
MotorDagoz locomotionMotorB(PA_10, PB_8);   //Locomotion Back Motor

//Serial pin
Serial pc(USBTX, USBRX, 115200);            //Serial debug

//Compass CMPS12 pin
CMPS_DAGOZ compass(PC_12, PB_10, 0xC0);       //Compass I2C Communication SDA SCL
DigitalOut compassLed(PB_0);                //CMPS error indicator

//LED Pin 
//DigitalOut ledRed(PA_4);                    //Red LED
//DigitalOut ledGreen(PC_10);                  //Green LED
//DigitalOut ledBlue(PA_1);                   //Blue LED

//Stepper Pin
//DigitalOut stepperVerDir(PC_13);                //Direction pin for stepper
//DigitalOut stepperVerStep(PC_8);               //Step pin for stepper, di board saat ini masih PC_4
//DigitalOut stepperHorDir(PC_9);                //Direction pin for stepper
//DigitalOut stepperHorStep(PC_5);               //Step pin for stepper, di board saat ini masih PC_4

#endif