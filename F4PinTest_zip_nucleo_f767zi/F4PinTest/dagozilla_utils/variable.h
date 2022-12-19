#ifndef VARIABLE_H
#define VARIABLE_H

#include "mbed.h"
#include <ros/time.h>

/*****************************
        Global Variable
 *****************************/
//Compass Global Varible
float compass_value = 0;                 //Compass value

//Odometry global variable
double locomotion_R_vel = 0;
double locomotion_L_vel = 0;
double locomotion_B_vel = 0;
double locomotion_R_rot = 0;
double locomotion_L_rot = 0;
double locomotion_B_rot = 0;

//For initiate encoder value
float cur_locomotion_R = 0;
float cur_locomotion_L = 0;
float cur_locomotion_B = 0;

//pwm value
double locomotion_R_target_rate = 0;
double locomotion_L_target_rate = 0;
double locomotion_B_target_rate = 0;

//extended variable
unsigned long last_timer;
float rotInR, rotInL, rotInB;
Timer t;

//Odometry
float theta_com;
float theta;
float theta_prev;

float vx;
float vy;
float vw;

#endif