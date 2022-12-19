/*Garudago ITB 2019 Attributes*/

#ifndef  CONFIG_H
#define CONFIG_H

const float  PI = 3.141592;
const float  RAD_TO_DEG = 57.295779;
const float  WHEEL_RAD = 0.075;

const float  motor_distance_from_center = 0.365;
const float  encoder_distance_from_center = 0.139;
const float  position_tolerance = 0.55; //0.5//awalnya 0.05
const float  orientation_tolerance = 0.55; //0.05

const float  max_velocity_magnitude = 3.0;
const float  max_rotation_magnitude = 3.0;
const float  cutoff_velocity_magnitude = 1; //1//awalnya 0.4
const float  cutoff_rotation_magnitude = 2.2; //2//awalnya 1.6

const float  max_linear_acceleration_magnitude = 3;
const float  max_angular_acceleration_magnitude = 3.0;
const float  max_linear_deceleration_magnitude = 3;
const float  max_angular_deceleration_magnitude = 3.0;

const float  linear_acceleration_feedforward_factor = 1.7;
const float  angular_acceleration_feedforward_factor = 1.7;
const float  linear_deceleration_feedforward_factor = 1.7;
const float  angular_deceleration_feedforward_factor = 1.7;

const float right_top_kp = 1.0;
const float right_top_ki = 1.0;
const float right_top_kd = 0.0;
const float right_top_N  = 0.0;
const float right_top_TS = 0.02;
const float right_top_FF = 0.0;

const float left_top_kp = right_top_kp;//0.5275; //0.3412
const float left_top_ki = right_top_ki;//0.09134; //0.06226 //0.0855
const float left_top_kd = 0.0;
const float left_top_N  = 0.0;
const float left_top_TS = 0.02;
const float left_top_FF = 0.0;

const float right_back_kp = right_top_kp;//0.4375; //0.454
const float right_back_ki = right_top_ki;//0.08134; //0.0715
const float right_back_kd = 0.0;
const float right_back_N  = 0.0;
const float right_back_TS = 0.02;
const float right_back_FF = 0.0;

const float left_back_kp = right_top_kp;//0.3624
const float left_back_ki = right_top_ki;//0.06817
const float left_back_kd = 0.0;
const float left_back_N  = 0.0;
const float left_back_TS = 0.02;
const float left_back_FF = 0.0;

//mode R2 
const float kp = 1.5;//0.5624;
const float ki = 1;//0.05817;
const float kp2 = 1.7;
const float ki2 = 1.1;

const float manual_linear_velocity = 3;
const float manual_linear_velocityR2 = 1;

#endif