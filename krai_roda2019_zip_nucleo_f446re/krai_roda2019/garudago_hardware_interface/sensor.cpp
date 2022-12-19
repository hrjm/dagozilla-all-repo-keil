/*
 * Author    : Garudago KRAI ITB 2019
 * Developer : Calmantara Sumpono Putra 
 * Version   : 1.0.0 
 */

#include "mbed.h"
#include "sensor.h"
#include <garudago_conf/config.h>


Compass::Compass(){
    // Constructor
}

void Compass::compass_reset(float _value){
    _offset_compass_value = _value;
}

void Compass::compass_update(float _value){
    _theta_origin = _value;
    _theta_offset = _value - _offset_compass_value;
}

float Compass::compass_value(){
    float theta_transformed;

    if(_theta_offset > 180.0 && _theta_offset <= 360.0)
        theta_transformed = (_theta_origin - 360.0 - _offset_compass_value)/-RAD_TO_DEG;
    else if(_theta_offset < -180.0 && _theta_offset >= -360.0)
        theta_transformed = (_theta_origin  + 360.0 - _offset_compass_value)/-RAD_TO_DEG;
    else
        theta_transformed = (_theta_origin - _offset_compass_value)/-RAD_TO_DEG;  

    return theta_transformed; 
}
