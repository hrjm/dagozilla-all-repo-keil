#ifndef CONFIG_H
#define CONFIG_H

#include "mbed.h"

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const float PI = 3.1415;                //PI constanta
const float RADTODEG = 57.2957;         //Radian to Deg const
const float LOCOMOTIONWHEEL = 0.024;    //R in metres
const float PID_TS = 0.01;
const float MAIN_TS = 10;
const float WHEELS_PPR = 196.0;
//right PID constants
const float right_kp = 0.3;
const float right_ki = 0.7;
const float right_kd = 0.0;
const float right_N = 0.0;
const float right_FF = 0.0;
//left PID constants
const float left_kp = 1.0;
const float left_ki = 0.0;
const float left_kd = 0.0;
const float left_N = 0.0;
const float left_FF = 0.0;
//back PID constants
const float back_kp = 1.0;
const float back_ki = 0.0;
const float back_kd = 0.0;
const float back_N = 0.0;
const float back_FF = 0.0;

#endif