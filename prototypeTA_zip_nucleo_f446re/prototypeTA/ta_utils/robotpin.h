#ifndef ROBOTPIN_H
#define ROBOTPIN_H

#include "mbed.h"
#include "EncoderDAGOZ.h"

/*****************************
        Pin Declaration
 *****************************/
//Encoder pin
EncoderDAGOZ locomotionEncL(TIM2);          //Locomotion Left Encoder
EncoderDAGOZ locomotionEncR(TIM1);          //Locomotion Right Encoder
EncoderDAGOZ locomotionEncB(TIM3);          //Locomotion Back Encoder
//Motor pin pwm fwd rev
Motor locomotionMotorB(PB_6, PA_6, PA_7);   //Locomotion Left Motor
Motor locomotionMotorL(PB_8, PC_7, PA_5);  //Locomotion Right Motor
Motor locomotionMotorR(PB_9, PA_10, PB_10);   //Locomotion Back Motor
//Serial pin
Serial pc(USBTX, USBRX, 115200);          //Serial debug 
DigitalOut buzzer1 (PC_1);
DigitalOut buzzer2 (PC_2);

#endif