#ifndef CONFIG_H
#define CONFIG_H

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             // Pi constant
const double RADTODEG = 57.2957795131;  // Radian to Deg conversion factor
const double LOCOMOTIONWHEEL = 0.030;   // in Meters

// High level PID calculation enable
int high_level_pid = 0;

// Default PID configuration
// Global PID config
float Ts = 0.02;

// Right locomotion motor PID config
float Kp_R = 0.5;
float Ki_R = 0.3;
float Kd_R = 0.0;
float alpha_R = 0.0;
float FF_R = 0.0;

// Left locomotion motor PID config
float Kp_L = 0.5;
float Ki_L = 0.3;
float Kd_L = 0.0;
float alpha_L = 0.0;
float FF_L = 0.0;

// Right locomotion motor PID config
float Kp_B = 0.5;
float Ki_B = 0.3;
float Kd_B = 0.0;
float alpha_B = 0.0;
float FF_B = 0.0;

#endif