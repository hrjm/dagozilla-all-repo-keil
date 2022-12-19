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
EncoderDAGOZ dribblerEncR(TIM4);            //Dribbler Right Encoder
EncoderDAGOZ dribblerEncL(TIM8);            //Dribbler Left Encoder

EncoderMotor intEncL(PG_8, PE_0, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor intEncB(PD_10, PG_14, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor intEncR(PF_2, PF_1, 537.6, EncoderMotor::X4_ENCODING);
//Motor pin buat Board Sistem next ver.
MotorDagoz locomotionMotorL(PF_12, PF_8);   //Locomotion Right Motor
MotorDagoz locomotionMotorR(PG_1, PF_9);    //Locomotion Left Motor
MotorDagoz locomotionMotorB(PF_11, PF_7);   //Locomotion Back Motor

MotorDagoz dribblerMotorL(PF_15, PE_6);     //Dribbler Right Motor
MotorDagoz dribblerMotorR(PE_15, PE_5);     //Dribbler Left Motor
//Serial pin
Serial pc(USBTX, USBRX, 115200);          //Serial debug 
//Compass CMPS12 pin
CMPS_DAGOZ compass(PB_9, PB_8, 0xC0);       //Compass I2C Communication SDA SCL
DigitalOut compassLed(PC_5);                //CMPS error indicator
//Potensio Pin
AnalogIn dribblerPotR(PF_6);                //Potensio for Left Dribbler, di board saat ini masih PC_2
AnalogIn dribblerPotL(PC_3);                //Potensio for Right Dribbler
AnalogIn infraRed(PF_5);                    //Potensio for Kicker, di board saat ini masih PD_4
//LED Pin 
DigitalOut ledRed(PG_4);                    //Red LED
DigitalOut ledGreen(PG_5);                  //Green LED
DigitalOut ledBlue(PG_6);                   //Blue LED
//Kicker Pin            
PwmOut kicker(PB_15);                       //Kicker pwm effort, di board saat ini masih PC_8
//Stepper Pin
DigitalOut stepperDir(PA_10);               //Direction pin for stepper
DigitalOut stepperStep(PF_10);              //Step pin for stepper, di board saat ini masih PC_4

#endif