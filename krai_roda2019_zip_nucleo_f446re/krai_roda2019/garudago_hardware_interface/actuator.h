/*
 * Author    : Garudago KRAI ITB 2019
 * Developer : Calmantara Sumpono Putra 
 * Version   : 1.0.0 
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

class Base{
    public:
        Base();
        //procedure to initiate all of constants

        float *update_state(float wheel_right_position,
                           float wheel_left_position,
                           float wheel_back_position);
        //procedure to get current position of robot

        float *compute_action(float x_robot,
                              float y_robot,
                              float theta, 
                              float x_target,
                              float y_target,
                              float theta_target);
        //function to get set point value for each motor

        float compute_velocity(float x_robot,
                               float y_robot,
                               float x_target,
                               float y_target);
        //function to compute linear velocity of robot

        float compute_rotation(float theta,
                               float theta_target);
        //function to compute rotation of robot

        float *manual_movement(float theta,
                              float theta_target,
                              float linear,
                              float alpha);
        //procedure for manual movement robot
        
        float _compute_alpha(float x_robot,
                             float y_robot,
                             float theta,
                             float x_target,
                             float y_target);
        //function to compute robot movement orientation

        float *_base_position_from_wheels(float wheel_right,
                                          float wheel_left,
                                          float wheel_back);
        //funstion to transform robot position from three wheels

        float *_base_velocity_from_wheels(float wheel_right_top,
                                          float wheel_left_top,
                                          float wheel_right_back,
                                          float wheel_left_back);
        //function to transform robot velocity from four wheels

        float *_wheels_from_base(float linear,
                                 float rotation,
                                 float alpha);   
        //funtion to compute set point for each motor   
  
    private:
        //motor and encoder distance from center
        float _motor_distance_from_center;
        float _encoder_distance_from_center;
        //limitation and tolerance
        float _position_tolerance;
        float _orientation_tolerance;
        float _max_velocity_magnitude;
        float _max_rotation_magnitude;
        float _cutoff_velocity_magnitude;
        float _cutoff_rotation_magnitude;
        //limitation 
        float _max_linear_acceleration_magnitude;
        float _max_linear_deceleration_magnitude;
        float _max_angular_acceleration_magnitude;
        float _max_angular_deceleration_magnitude;
        //feedforward constant
        float _linear_acceleration_feedforward_factor;
        float _angular_acceleration_feedforward_factor;
        float _linear_deceleration_feedforward_factor;
        float _angular_deceleration_feedforward_factor;
        
        float x_actual, y_actual, theta_actual;
        float _last_position, _last_orientation;
        //compute rotation variable
        float _feedforward_rotation_command;
        float _prev_feedforward_rotation_command;
        float _rotation_from_s1;    //for acceleration command
        float _rotation_from_s2;    //for deceleration command
        float _target_rotation_command;
        float _prev_target_rotation_command;
        //compute velocity variable
        float _feedforward_linear_command;
        float _prev_feedforward_linear_command;
        float _linear_from_s1;    //for acceleration command
        float _linear_from_s2;    //for deceleration command
        float _target_linear_command;
        float _prev_target_linear_command;
};


#endif