#ifndef VARIABLE_H
#define VARIABLE_H

#include "mbed.h"
#include <ros/time.h>

/*****************************
        Global Variable
 *****************************/
// PID Controller Config
enum Mode
{
        P,
        PIX,
        PD,
        PID
};
int PIDModeFR;
int PIDModeFL;
int PIDModeBR;
int PIDModeBL;
float cp = 0.02;
float kpFR = 0;
float kpFL = 0;
float kpBR = 0;
float kpBL = 0;
float tiFR = 0;
float tiFL = 0;
float tiBR = 0;
float tiBL = 0;
float tdFR = 0;
float tdFL = 0;
float tdBR = 0;
float tdBL = 0;
float ffFR = 0;
float ffFL = 0;
float ffBR = 0;
float ffBL = 0;
float fcFR = 0;
float fcFL = 0;
float fcBR = 0;
float fcBL = 0;

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

// Target Velocity & Feedback Velocity
double locomotion_FL_target_vel = 0;
double locomotion_FR_target_vel = 0;
double locomotion_BL_target_vel = 0;
double locomotion_BR_target_vel = 0;

double locomotion_FL_vel = 0;
double locomotion_FR_vel = 0;
double locomotion_BL_vel = 0;
double locomotion_BR_vel = 0;

//extended variable
unsigned long last_timer;
int32_t rotInFL = 0;
int32_t rotInFR = 0;
int32_t rotInBL = 0;
int32_t rotInBR = 0;

Timer t;

//Odometry
float theta_com;
float theta_result = 0;
float theta_prev;

#endif