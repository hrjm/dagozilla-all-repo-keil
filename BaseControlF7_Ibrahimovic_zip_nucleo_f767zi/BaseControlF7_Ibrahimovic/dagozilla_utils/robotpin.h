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
EncoderDAGOZ dribblerEncL(TIM4);
EncoderDAGOZ dribblerEncR(TIM8);

EncoderMotor intEncFL(PG_8, PE_0, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncFR(PD_10, PG_14, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncBL(PF_2, PF_1, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncBR(PB_7, PC_13, 364.0, EncoderMotor::X4_ENCODING);

//Motor pin buat Board Sistem next ver.
MotorDagoz locomotionMotorFL(LOCOMOTION_PWM_PERIOD_US, PF_11, PF_7);   //Locomotion Front Left Motor
MotorDagoz locomotionMotorFR(LOCOMOTION_PWM_PERIOD_US, PF_12, PF_8);    //Locomotion Front Right Motor
MotorDagoz locomotionMotorBL(LOCOMOTION_PWM_PERIOD_US, PG_1, PF_9);   //Locomotion Back Left Motor
MotorDagoz locomotionMotorBR(LOCOMOTION_PWM_PERIOD_US, PC_2, PA_15);  //Locomotion Back Right Motor

MotorDagoz dribblerMotorR(DRIBBLER_PWM_PERIOD_US, PF_15, PE_6);     //Dribbler Right Motor
MotorDagoz dribblerMotorL(DRIBBLER_PWM_PERIOD_US, PE_15, PE_5);     //Dribbler Left Motor

//Serial pin
Serial pc(USBTX, USBRX, 115200);          //Serial debug 

//Compass CMPS12 pin
CMPS_DAGOZ compass(PB_9, PB_8, 0xC0);       //Compass I2C Communication SDA SCL
DigitalOut compassLed(PC_5);                //CMPS error indicator

//Potensio Pin  
AnalogIn dribblerPotL(PF_6);                //Potensio for Left Dribbler, di board saat ini masih PC_2
AnalogIn dribblerPotR(PC_3);                //Potensio for Right Dribbler
AnalogIn infraRed(PF_5);                    //Potensio for Kicker, di board saat ini masih PD_4

//LED Pin 
DigitalOut ledRed(PG_4);                    //Red LED
DigitalOut ledGreen(PG_5);                  //Green LED
DigitalOut ledBlue(PG_6);                   //Blue LED

//Kicker Pin            
PwmOut kicker(PB_15);                       //Kicker pwm effort, di board saat ini masih PC_8

//Servo Pin
PwmOut kickerServo(PC_7);

#endif