/*Garudago ITB 2019 Attributes*/

#ifndef  VARIABLE_H
#define VARIABLE_H

#include "mbed.h"

//robot static body variable
//[0] : x position
//[1] : y position
//[2] : orientation
float *robot_geometry;
float theta_from_compass;

float theta_target = 0.0; 
float theta_shagai = 0.0;
float count_reset_arm = 0.0;

//robot kinetic variable
//[0] left_top
//[1] right_top
//[3] left_back
//[4] right_back
float *wheels_target_velocity;
float left_top_vel;
float right_top_vel;
float left_back_vel;
float right_back_vel;

float linear_velocity;
float rotation_velocity;
float manual_alpha;

int state_condition;

bool brake_state = 0;
bool brake_clear = 0;
bool up;
bool belok=1;
//Timer Variable
Timer timer;
unsigned long prev_compass_timer;
unsigned long prev_motor_timer;
unsigned long prev_pneumatic_timer;
unsigned long prev_stick_timer;
unsigned long prev_stick_symbol_timer;

/***********************************************************************/
//Struct for setpoint mapping
typedef struct map {
    int n;
    float x_pos[10];
    float y_pos[10];
    float theta_pos[10];
    float x_offset[10];
    float y_offset[10];
} mapping;

const mapping map_NULL =    {  0,
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}};

const mapping red_field =   {  0,
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}};

const mapping blue_field =   {  0,
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}};

mapping map_data = map_NULL;

#endif