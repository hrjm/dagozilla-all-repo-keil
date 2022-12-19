#ifndef PID_H
#define PID_H

#include "mbed.h"

// Discrete PID Algorithm
class PID {
public :
        typedef enum Mode{
            PI_MODE,
            PID_MODE
        }Mode;

        PID(double p , double i , double d , double _N , double _Ts, double FF, Mode _mode) ;
        double createpwm( double setpoint , double feedback ) ;
        void setTunings( double _Kp, double _Ki, double _Kd, double _N, double _FF );
    
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
        double setpoint_prev;
        Mode mode;
};

// 'Continuous' PID Algorithm
// the implemented algorithm is an IP-D controller
class pid_analog {
        
public  :
        // alpha range: 0 <= alpha <= 1
        pid_analog( double Kp, double Ki, double Kd, double alpha, 
                double Ts, double FF );
        
        double createpwm( double setpoint, double feedback);
        
        void setTunings( double Kp, double Ki, double Kd, double alpha,
                double FF );


private :
        
        // Attributes
        double _Kp;
        double _Ki;
        double _Kd;
        double _alpha;
        double _Ts;
        double _FF;
        
        // Helper variables 
        double setpoint_current;
        double setpoint_prev;
        double error_current;
        double integral_term;
        double feedback_prev;

        // Filter variables
        double filtered_feedback_current;
        double filtered_feedback_prev;

        double max_output;
        double min_output;
};

#endif
