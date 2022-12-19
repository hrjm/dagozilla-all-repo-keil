#ifndef CONFIG_H
#define CONFIG_H

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double RADTODEG = 57.2957795131;  //Radian to Deg const
const double LOCOMOTIONWHEEL = 0.03;    //Metres
const double DRIBBLERWHEEL = 0.0325;     //Metres
const double tolerance = 0.3;
// Right Dribbler Components
float right_from_potensio = 2.0;//0.925;
float right_from_vy = 2.0;
float right_from_vx = 0;
float right_from_rot = 0.416; 
const float right_pot_target = 0.7;              //Right Dribbler potensio target
// Left Dribbler Components
float left_from_potensio = 2.0;//0.925;
float left_from_vy = 2.0;
float left_from_vx = 0;
float left_from_rot = 0.416; 
const float left_pot_target = 0.7;              //Left Dribbler potensio target

// PID Constants
const double KP_R = 0.5;    //Kp for right Locomotion motor
const double KI_R = 0.3;    //Ki for right Locomotion motor
const double KD_R = 0.0;    //Kd for right Locomotion motor
const double N_R = 0.0;     //LPF coefficient for right Locomotion motor     
const double FF_R = 0.0;    //Feedforward gain for right Locomotion motor

const double KP_B = 0.5;    //Kp for back Locomotion motor
const double KI_B = 0.3;    //Ki for back Locomotion motor
const double KD_B = 0.0;    //Kd for back Locomotion motor
const double N_B = 0.0;     //LPF coefficient for back Locomotion motor     
const double FF_B = 0.0;    //Feedforward gain for back Locomotion motor

const double KP_L = 0.5;    //Kp for left Locomotion motor
const double KI_L = 0.3;    //Ki for left Locomotion motor
const double KD_L = 0.0;    //Kd for left Locomotion motor
const double N_L = 0.0;     //LPF coefficient for left Locomotion motor     
const double FF_L = 0.0;    //Feedforward gain for left Locomotion motor

const double KP_DL = 0.3;    //Kp for left dribbler motor
const double KI_DL = 0.3;    //Ki for left dribbler motor
const double KD_DL = 0.0;    //Kd for left dribbler motor
const double N_DL = 0.0;     //LPF coefficient for left dribbler motor     
const double FF_DL = 0.2;    //Feedforward gain for left dribbler motor

const double KP_DR = 0.3;    //Kp for right dribbler motor
const double KI_DR = 0.3;    //Ki for right dribbler motor
const double KD_DR = 0.0;    //Kd for right dribbler motor
const double N_DR = 0.0;     //LPF coefficient for right dribbler motor     
const double FF_DR = 0.2;    //Feedforward gain for right dribbler motor

const double TS = 0.02;

#endif