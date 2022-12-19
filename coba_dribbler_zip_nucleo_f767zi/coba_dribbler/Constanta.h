#ifndef CONSTANTA_H
#define CONSTANTA_H

#include "mbed.h"
/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double RADTODEG = 57.2957795131;  //Radian to Deg const
const double LOCOMOTIONWHEEL = 0.03;    //Metres
const double DRIBBLERWHEEL = 0.035;     //Metres
// Right Dribbler Components
const float RK1FB = 0.1;                //Right Dribbler Pot to Omega feedback 
const float RK2FB = 0.35;                //Right Dribbler Error to pwm feedback

const float RKVFF = 0.087;                //Right Dribbler velocity feedforward
const float RKTFF = 0.5;                //Right Dribbler w velocity feedforward

const float RTARGET = 9.0;              //Right Dribbler potensio target
// Left Dribbler Components
const float LK1FB = 0.1;                //Left Dribbler Pot to Omega feedback 
const float LK2FB = 0.35;                //Left Dribbler Error to pwm feedback

const float LKVFF = 0.087;                //Left Dribbler velocity feedforward
const float LKTFF = 0.5;                //Left Dribbler w velocity feedforward

const float LTARGET = 9.0;              //Left Dribbler potensio target
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
double locomotion_R_vtarget = 0;
double locomotion_L_vtarget = 0;
double locomotion_B_vtarget = 0;
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
//feedforward transition
double target_vel_L = 0;
double target_vel_R = 0;
double prev_target_vel_L = 0;
double prev_target_vel_R = 0;
//pwm value
double locomotion_R_target_rate = 0;
double locomotion_L_target_rate = 0;
double locomotion_B_target_rate = 0;
double dribbler_L_target_rate = 0;
double dribbler_R_target_rate = 0;

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