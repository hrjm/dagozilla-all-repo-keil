/*
 * Author    : Garudago KRAI ITB 2019
 * Developer : Calmantara Sumpono Putra 
 * Version   : 1.0.0 
 */

#ifndef SENSOR_H
#define SENSOR_H

class Compass{
    public:
        Compass();
        void compass_reset(float _value);
        //procedure that update offset value of compass

        void compass_update(float _value);
        //procedure that update compass value

        float compass_value();
        //procedure that return compass value after transform
    private:
        float _offset_compass_value;
        float _theta_origin;
        float _theta_offset;

};

#endif