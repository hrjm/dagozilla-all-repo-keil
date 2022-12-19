#ifndef VARIABLE_H
#define VARIABLE_H

#include "mbed.h"
#include <ros/time.h>

/*****************************
        Global Variable
 *****************************/

//Kicker global variable
double kick_power_target = 0;
bool kicker_shoot_mode = 0;
float range = 0.0011;
float position = 0.03;
float ball_distance = 0;

//For initiate encoder value
float cur_locomotion_L = 0;
float cur_locomotion_R = 0;
float cur_locomotion_B = 0;
float cur_dribbler_L = 0;
float cur_dribbler_R = 0;

//Potentio value
float cur_pot_L = 0;
float cur_pot_R = 0;

//pwm value
double locomotion_FL_target_rate = 0;
double locomotion_FR_target_rate = 0;
double locomotion_BL_target_rate = 0;
double locomotion_BR_target_rate = 0;
double dribbler_L_target_rate = 0;
double dribbler_R_target_rate = 0;

//extended variable
unsigned long last_timer;
int32_t rotInFL = 0;
int32_t rotInFR = 0;
int32_t rotInBL = 0;
int32_t rotInBR = 0;

Timer t;
uint64_t time_last_kick = 0;
uint64_t kicker_ready_time = 3000;

//Odometry
float theta_com;
float theta_result= 0;
float theta_prev;

#endif