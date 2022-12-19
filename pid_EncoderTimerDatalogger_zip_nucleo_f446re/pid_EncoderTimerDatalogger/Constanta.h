#ifndef CONSTANTA_H
#define CONSTANTA_H

#include "mbed.h"
/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double LOCOMOTIONWHEEL = 0.05;    //Metres
const double DRIBBLERWHEEL = 0.035;     //Metres
// Locomotion Components
const float KVFF = 1.0;                 //Velocity Feedforward for Locomotion
const float KAFF = 1.0;                 //Acceleration Feedforward for Locomotion
const float ACCT = 2.0;                 //Acceleration for robot Translation
const float ACCR = 2.0;                 //Acceleration for robot rotation
// Right Dribbler Components
const float RK1FB = 1.0;                //Right Dribbler Pot to Omega feedback 
const float RK2FB = 1.0;                //Right Dribbler Error to pwm feedback
const float RKVFF = 1.0;                //Right Dribbler velocity feedforward
const float RKXFF = 1.0;                //Right Dribbler x velocity feedforward
const float RKYFF = 1.0;                //Right Dribbler y velocity feedforward
const float RKTFF = 1.0;                //Right Dribbler w velocity feedforward
const float RTARGET = 1.0;              //Right Dribbler potensio target
// Left Dribbler Components
const float LK1FB = 1.0;                //Left Dribbler Pot to Omega feedback 
const float LK2FB = 1.0;                //Left Dribbler Error to pwm feedback
const float LKVFF = 1.0;                //Left Dribbler velocity feedforward
const float LKXFF = 1.0;                //Left Dribbler x velocity feedforward
const float LKYFF = 1.0;                //Left Dribbler y velocity feedforward
const float LKTFF = 1.0;                //Left Dribbler w velocity feedforward
const float LTARGET = 1.0;              //Left Dribbler potensio target
/*****************************
        Global Variable
 *****************************/
//Compass Global Varible
float compassValue = 0;                 //Compass value
//Odometry global variable
double locomotionRVel = 0;
double locomotionLVel = 0;
double locomotionBVel = 0;
double locomotionRRot = 0;
double locomotionLRot = 0;
double locomotionBRot = 0;
double locomotionRVtarget = 0;
double locomotionLVtarget = 0;
double locomotionBVtarget = 0;
//Dribbler global variabel 
double dribblerRVel = 0;
double dribblerLVel = 0;
double dribblerRRot = 0;
double dribblerLRot = 0;    
double dribblerRVtarget = 0;
double dribblerLVtarget = 0;
//Kicker global variable
double kickPowerTarget = 0;
float distance = 0;

#endif