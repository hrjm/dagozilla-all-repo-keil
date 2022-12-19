#ifndef CONSTANTA_H
#define CONSTANTA_H

#include "mbed.h"

/*****************************
        Global Variable
 *****************************/
//Odometry global variable
double locomotion_R_vel = 0;
double locomotion_L_vel = 0;
double locomotion_B_vel = 0;
double locomotion_R_rot = 0;
double locomotion_L_rot = 0;
double locomotion_B_rot = 0;
double locomotion_R_vtarget = 0;
double locomotion_L_vtarget = 0;
double locomotion_B_vtarget = 0;

//For initiate encoder value
float cur_locomotion_R = 0;
float cur_locomotion_L = 0;
float cur_locomotion_B = 0;
//pwm value
double locomotion_R_target_rate = 0;
double locomotion_L_target_rate = 0;
double locomotion_B_target_rate = 0;
bool buzzer_state = false;


//Odometry
float theta_com;
float theta;
float theta_prev;
float x;
float x_prev;
float y;
float y_prev;

float vx;
float vy;
float w;

#endif