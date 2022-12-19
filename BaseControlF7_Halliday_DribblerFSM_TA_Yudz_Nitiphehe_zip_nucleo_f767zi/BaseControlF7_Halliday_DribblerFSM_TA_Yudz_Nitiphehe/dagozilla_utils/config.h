#ifndef CONFIG_H
#define CONFIG_H

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double RADTODEG = 57.2957795131;  //Radian to Deg const
const double LOCOMOTIONWHEEL = 0.03;    //Metres
const double DRIBBLERWHEEL = 0.028;     //Metres
const double tolerance = 0.5;
// Right Dribbler Components
float right_from_potensio = -0.5;//0.925;
float right_from_vy = 0;
float right_from_vx = 0;
float right_from_rot = -0.2; 
const float right_pot_target = 1.7;              //Right Dribbler potensio target
// Left Dribbler Components
float left_from_potensio = 0.5;//0.925;
float left_from_vy = 0;
float left_from_vx = 0;
float left_from_rot = -0.2;
const float left_pot_target = 1.7;              //Left Dribbler potensio target

// PID Constants
const double KP_R = 0.342;    //Kp for right Locomotion motor
const double KI_R = 7.17;    //Ki for right Locomotion motor
const double KD_R = 0.0;    //Kd for right Locomotion motor
const double N_R = 0.0;     //LPF coefficient for right Locomotion motor     
const double FF_R = 0.0;    //Feedforward gain for right Locomotion motor

const double KP_B = 0.319;    //Kp for back Locomotion motor
const double KI_B = 7.05;    //Ki for back Locomotion motor
const double KD_B = 0.0;    //Kd for back Locomotion motor
const double N_B = 0.0;     //LPF coefficient for back Locomotion motor     
const double FF_B = 0.0;    //Feedforward gain for back Locomotion motor

const double KP_L = 0.318;    //Kp for left Locomotion motor
const double KI_L = 7.54;    //Ki for left Locomotion motor
const double KD_L = 0.0;    //Kd for left Locomotion motor
const double N_L = 0.0;     //LPF coefficient for left Locomotion motor     
const double FF_L = 0.0;    //Feedforward gain for left Locomotion motor

const double KP_DL_1 = 0.363;//0.209;//0.209;//1.203;//0.209;//0.503;//0.962;//1.07;    //Kp for left dribbler motor
const double KI_DL_1 = 6.723;//3.97;//3.97;//18.51;//3.97;//9.85;//19.2;//24.6;//10;//;    //Ki for left dribbler motor
const double KD_DL_1 = 0.0;    //Kd for left dribbler motor
const double N_DL_1 = 0.0;     //LPF coefficient for left dribbler motor     
const double FF_DL_1 = 0.023;    //Feedforward gain for left dribbler motor

const double KP_DR_1 = 0.395;//0.175;//0.175;//0.903;//0.175;//0.421;//0.83;//0.906;    //Kp for right dribbler motor
const double KI_DR_1 = 6.155;//3.82;//3.82;//19.14;//3.82;//9.43;//18.3;//25;//25;    //Ki for right dribbler motor
const double KD_DR_1 = 0.0;    //Kd for right dribbler motor
const double N_DR_1 = 0.0;     //LPF coefficient for right dribbler motor     
const double FF_DR_1 = 0.023;    //Feedforward gain for right dribbler motor

//const double KP_DL_2 = 0.209;//1.07;    //Kp for left dribbler motor
//const double KI_DL_2 = 3.97;//10;//;    //Ki for left dribbler motor
//const double KD_DL_2 = 0.0;    //Kd for left dribbler motor
//const double N_DL_2= 0.0;     //LPF coefficient for left dribbler motor     
//const double FF_DL_2 = 0.2;    //Feedforward gain for left dribbler motor
//
//const double KP_DR_2 = 0.175;//0.906;    //Kp for right dribbler motor
//const double KI_DR_2 = 3.82;//25;    //Ki for right dribbler motor
//const double KD_DR_2 = 0.0;    //Kd for right dribbler motor
//const double N_DR_2 = 0.0;     //LPF coefficient for right dribbler motor     
//const double FF_DR_2 = 0.2;    //Feedforward gain for right dribbler motor


const double TS = 0.01;

#endif