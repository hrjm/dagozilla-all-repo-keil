/*
 * Author    : Garudago KRAI ITB 2019
 * Developer : Calmantara Sumpono Putra 
 * Version   : 1.0.0 
 */

#include "mbed.h"
#include "actuator.h"
#include <garudago_conf/config.h>

Base::Base(){
    //motor and encoder distance from center
    _motor_distance_from_center =  motor_distance_from_center;
    _encoder_distance_from_center = encoder_distance_from_center;
    //limitation and tolerance
    _position_tolerance = position_tolerance;
    _orientation_tolerance = orientation_tolerance;
    _max_velocity_magnitude = max_velocity_magnitude;
    _max_rotation_magnitude = max_rotation_magnitude;
    _cutoff_velocity_magnitude = cutoff_velocity_magnitude;
    _cutoff_rotation_magnitude = cutoff_rotation_magnitude;
    //limitation 
    _max_linear_acceleration_magnitude = max_linear_acceleration_magnitude;
    _max_linear_deceleration_magnitude = max_linear_deceleration_magnitude;
    _max_angular_acceleration_magnitude = max_angular_acceleration_magnitude;
    _max_angular_deceleration_magnitude = max_angular_deceleration_magnitude;
    //feedforward constant
    _linear_acceleration_feedforward_factor = linear_acceleration_feedforward_factor;
    _angular_acceleration_feedforward_factor = angular_acceleration_feedforward_factor;
    _linear_deceleration_feedforward_factor = linear_deceleration_feedforward_factor;
    _angular_deceleration_feedforward_factor = angular_deceleration_feedforward_factor;
}

float *Base::update_state(float wheel_right_position,
                        float wheel_left_position,
                        float wheel_back_position){
//procedure to get current position of robot
    float *_get_geometry; 
    static float _geometry[3];
    _get_geometry = _base_position_from_wheels(wheel_right_position,
                                                wheel_left_position,
                                                wheel_back_position);
    _geometry[0] = _get_geometry[0];
    _geometry[1] = _get_geometry[1];
    _geometry[2] = _get_geometry[2];
    return _geometry;
}

float *Base::compute_action(float x_robot,
                            float y_robot,
                            float theta, 
                            float x_target,
                            float y_target,
                            float theta_target){
//function to get set point value for each motor
    float _linear_velocity;
    float _rotation;
    float _alpha;
    float *_get_wheels;
    static float _wheels[3];

    _linear_velocity = compute_velocity(x_robot, 
                                        y_robot,
                                        x_target, 
                                        y_target);

    _rotation = compute_rotation(theta, theta_target);

    _alpha = _compute_alpha(x_robot, 
                            y_target,
                            theta,
                            x_target,
                            y_target);

    _get_wheels = _wheels_from_base(_linear_velocity,
                                    _rotation,
                                    _alpha);

    _wheels[0] = _get_wheels[0];
    _wheels[1] = _get_wheels[1];
    _wheels[2] = _get_wheels[2];
    _prev_feedforward_rotation_command = _rotation;
    _prev_feedforward_linear_command = _linear_velocity;

    return _wheels;
}

float *Base::manual_movement(float theta, 
                            float theta_target,
                            float linear,
                            float alpha){
    float _rotation;
    float *_get_wheels;
    static float _wheels[4];

    _rotation = compute_rotation(theta, theta_target);
    _get_wheels = _wheels_from_base(linear, _rotation, alpha);

    _wheels[0] = _get_wheels[0];
    _wheels[1] = _get_wheels[1];
    _wheels[2] = _get_wheels[2];
    _wheels[3] = _get_wheels[3];

    return _wheels;
}

float Base::compute_velocity(float x_robot,
                            float y_robot,
                            float x_target,
                            float y_target){
//function to compute linear velocity of auto mode robot
    return 0;
}

float Base::compute_rotation(float theta,
                             float theta_target){
//function to compute rotation of auto mode robot
    if(fabs(theta_target - theta)<=orientation_tolerance &&
       (fabs(_prev_feedforward_rotation_command) < _cutoff_rotation_magnitude)){
        _feedforward_rotation_command = 0;
        _last_orientation = theta;
    } 
    
    else{
        //compute velocity from origin and target position
        if(theta_target > theta){
            _rotation_from_s1 = sqrt(2 * _max_angular_acceleration_magnitude * 
                                fabs(theta - _last_orientation));
            _rotation_from_s2 = sqrt(2 * _max_angular_deceleration_magnitude * 
                                fabs(theta_target - theta));
        } else{
            _rotation_from_s1 = -sqrt(2 * _max_angular_acceleration_magnitude * 
                                fabs(theta - _last_orientation));
            _rotation_from_s2 = -sqrt(2 * _max_angular_deceleration_magnitude * 
                                fabs(theta_target - theta));
        }

        //acceleration or deceleration
        _prev_target_rotation_command = _target_rotation_command;
        if(_rotation_from_s1 <= _rotation_from_s2){ 
            _target_rotation_command = _rotation_from_s1;
        }
        else if(_rotation_from_s2 < _rotation_from_s1){
            _target_rotation_command = _rotation_from_s2;
        }

        //feedforward command
        float _delta_rotation_command = fabs(_target_rotation_command) - 
                                        fabs(_prev_target_rotation_command);
        if(_delta_rotation_command > 0){
            _feedforward_rotation_command = _prev_target_rotation_command + 
                (angular_acceleration_feedforward_factor * _delta_rotation_command);
        } else{
            _feedforward_rotation_command = _prev_target_rotation_command + 
                (angular_deceleration_feedforward_factor * _delta_rotation_command);
        }

        //limitation for command
        if(fabs(_feedforward_rotation_command) > max_angular_acceleration_magnitude){
            _feedforward_rotation_command = max_angular_acceleration_magnitude;
        }

        //condition to make correct sign
        if(theta_target < theta){
            _feedforward_rotation_command = -_feedforward_rotation_command;
        }
    }
    return _feedforward_rotation_command;   
}

float Base::_compute_alpha(float x_robot,
                        float y_robot,
                        float theta,
                        float x_target,
                        float y_target){
//function to compute robot movement orientation
    float _alpha = atan((y_target-y_robot)/(x_target-x_robot)) - theta;
    
    if(x_target < x_robot)  return _alpha + PI;
    else    return _alpha;
}

float *Base::_base_position_from_wheels(float wheel_right,
                                        float wheel_left,
                                        float wheel_back){
//funstion to transform robot position from three wheels
    static float _geometry[3];
    float _y = sqrt(3.0) / 3.0 * (wheel_right - wheel_left);
    float _x = (1.0 / 3.0 * (wheel_left + wheel_right)) - (2.0 / 3.0 * wheel_back);
    float _theta = (1.0 / (3.0 * _encoder_distance_from_center)) * \
                    (wheel_right + wheel_back + wheel_left);

    x_actual += cos(_theta)*_x - sin(_theta)*_y;
    y_actual += sin(_theta)*_x + cos(_theta)*_y;
    theta_actual += _theta;
    
    _geometry[0] = x_actual;
    _geometry[1] = y_actual;
    _geometry[2] = theta_actual;
    return _geometry;
}

float *Base::_base_velocity_from_wheels(float wheel_right_top,
                                        float wheel_left_top,
                                        float wheel_right_back,
                                        float wheel_left_back){

    return 0;
}

float *Base::_wheels_from_base(float linear,
                            float rotation,
                            float alpha){
//funtion to compute set point for each motor

    static float _motor[4];
    _motor[0] = (-linear*cos(alpha)*1.4142/2) - (linear*sin(alpha)*1.4142/2) + \
               (rotation*_motor_distance_from_center);   //for left top motor
    _motor[1] = (-linear*cos(alpha)*1.4142/2) + (linear*sin(alpha)*1.4142/2) + \
               (rotation*_motor_distance_from_center);   //for right top motor
    _motor[2] = (linear*cos(alpha)*1.4142/2) - (linear*sin(alpha)*1.4142/2) + \
               (rotation*_motor_distance_from_center);   //for left back motor
    _motor[3] = (linear*cos(alpha)*1.4142/2) + (linear*sin(alpha)*1.4142/2) + \
               (rotation*_motor_distance_from_center);   //for right back motor
    
    return _motor;
}