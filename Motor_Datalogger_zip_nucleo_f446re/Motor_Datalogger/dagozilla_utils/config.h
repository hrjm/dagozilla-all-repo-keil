#ifndef CONFIG_H
#define CONFIG_H

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double RADTODEG = 57.2957795131;  //Radian to Deg const
const double LOCOMOTIONWHEEL = 0.05;    //Metres
const double DRIBBLERWHEEL = 0.029;     //Metres
const double PPR_ENC = 4096.0;

const double KP_DL = 1.07;    //Kp for left dribbler motor
const double KI_DL = 24.6;//;    //Ki for left dribbler motor
const double KD_DL = 0.0;    //Kd for left dribbler motor
const double N_DL = 0.0;     //LPF coefficient for left dribbler motor     
const double FF_DL = 0.2;    //Feedforward gain for left dribbler motor

const double KP_DR = 0.906;    //Kp for right dribbler motor
const double KI_DR = 25;    //Ki for right dribbler motor
const double KD_DR = 0.0;    //Kd for right dribbler motor
const double N_DR = 0.0;     //LPF coefficient for right dribbler motor     
const double FF_DR = 0.2;    //Feedforward gain for right dribbler motor

const double TS = 0.01;

#endif