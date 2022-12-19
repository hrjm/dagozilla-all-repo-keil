#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID
{
public :
        typedef enum Mode{
            PI_MODE,
            PID_MODE
        }Mode;

        PID(double p , double i , double d , double _N , double _Ts, double FF, Mode _mode) ;
        double createpwm( double setpoint , double feedback ) ;
    
private :
        double Kp ;
        double Kd ;
        double Ki ;
        double N ;
        double Ts ;
        double a0;
        double a1;
        double a2;
        double b0;
        double b1;
        double b2;
        double ku1;
        double ku2;
        double ke0;
        double ke1;
        double ke2;
        double e2;
        double e1;
        double e0;
        double u2;
        double u1;
        double u0; 
        double FF;
        double prev_setpoint;
        Mode mode;
};
#endif

    