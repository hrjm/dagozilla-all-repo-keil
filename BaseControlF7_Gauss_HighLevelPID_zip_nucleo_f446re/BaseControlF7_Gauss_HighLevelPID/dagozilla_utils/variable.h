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
//Dribbler global variabel 
double dribbler_R_vel = 0;
double dribbler_L_vel = 0;
double dribbler_R_rot = 0;
double dribbler_L_rot = 0;    
double dribbler_R_vtarget = 0;
double dribbler_L_vtarget = 0;
//Kicker global variable
double kick_power_target = 0;
float distance = 0;

//For initiate encoder value
float cur_locomotion_R = 0;
float cur_locomotion_L = 0;
float cur_locomotion_B = 0;
float cur_dribbler_R = 0;
float cur_dribbler_L = 0;
//Potensio value
float cur_pot_L = 0;
float cur_pot_R = 0;
//pwm value
double locomotion_R_target_rate = 0;
double locomotion_L_target_rate = 0;
double locomotion_B_target_rate = 0;
double dribbler_L_target_rate = 0;
double dribbler_R_target_rate = 0;
//extended variable
bool dribbler_state;
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