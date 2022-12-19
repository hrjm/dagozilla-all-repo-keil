#include "PID.h"

// Discrete PID Algorithm
PID :: PID(double p , double i , double d , double _N , double _Ts, double _FF, Mode _mode)
{
    Kp = p ; Kd = d ; Ki = i ; N = _N ; Ts = _Ts ;
    FF = _FF;
    mode = _mode;
    
    if(mode == PID_MODE){
        a0 = (1+N*Ts);
        a1 = -(2 + N*Ts);
        a2 = 1;
        b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
        b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
        b2 = Kp + Kd*N;

    }
    else if(mode == PI_MODE){
        a0 = 1;
        a1 = -1;
        a2 = 0;
        b0 = Kp + Ki*Ts;
        b1 = -Kp;
        b2 = 0;
    }

    ku1 = a1/a0;
    ku2 = a2/a0;
    ke0 = b0/a0;
    ke1 = b1/a0;
    ke2 = b2/a0;
}

double PID::createpwm( double setpoint , double feedback )
{
    e2 = e1 ;
    e1 = e0 ;
    u2 = u1 ;
    u1 = u0 ;
    e0 = setpoint-feedback;
    u0 = - (ku1 * u1 )  - ( ku2*u2 )  + ke0*e0 + ke1*e1 + ke2*e2;
    setpoint_prev = setpoint;
    
    if (u0 >= 1)
    {
        u0 = 1 ;
    }
    else if (u0 <= -1)
    {
        u0 = -1;
    }
        
    double feedforward_output = u0 + FF*setpoint;
    
    if (feedforward_output >= 1)
    {
        feedforward_output = 1 ;
    }
    else if (feedforward_output <= -1)
    {
        feedforward_output = -1;
    }

    
    return feedforward_output ;   
}

void PID::setTunings( double _Kp, double _Ki, double _Kd, double _N, double _FF ) {
    
    Kp = _Kp;
    Ki = _Ki * Ts;
    Kd = _Kd / Ts;
    N = _N;
    FF = _FF;

}


// 'Continuous' PID Algorithm
pid_analog::pid_analog( double Kp, double Ki, double Kd, double alpha, double Ts, double FF ) {

    _alpha = alpha;
    _Ts = Ts;
    _FF = FF;

    _Kp = Kp;
    _Ki = Ki * _Ts;
    _Kd = Kd / _Ts;

    setpoint_prev = 0;
    setpoint_current = 0;
    error_current = 0;
    integral_term = 0;
    feedback_prev = 0;

    filtered_feedback_prev = 0;

    min_output = -1.0;
    max_output = 1.0;
}


double pid_analog::createpwm( double setpoint, double feedback ) {

    double output;
    double feedback_diff, setpoint_diff;
    double derivative_term;

    // Check for setpoint changes
    if (setpoint != setpoint_current) {
        setpoint_prev = setpoint_current;
        setpoint_current = setpoint;
    }

    // Error variables calculation
    error_current = setpoint - feedback;
    feedback_diff = feedback - feedback_prev;
    setpoint_diff = setpoint_current - setpoint_prev;

    integral_term += _Ki * error_current;
    
    // Exponential Filter
    filtered_feedback_current = _alpha * feedback_diff + (1 - _alpha) * filtered_feedback_prev;
    filtered_feedback_prev = filtered_feedback_current;

    derivative_term = -_Kd * filtered_feedback_current;

    // Integral windup mitigation
    if ( integral_term > max_output ) integral_term = max_output;
    else if ( integral_term < min_output ) integral_term = min_output;

    // Output calculation
    output = _Kp * error_current + integral_term + derivative_term + _FF * setpoint;

    // Update for the next loop
    feedback_prev = feedback;
    setpoint_prev = setpoint;

    // Output windup mitigation
    if ( output > max_output ) output = max_output;
    else if ( output < min_output ) output = min_output;
    
    return output;
}

void pid_analog::setTunings( double Kp, double Ki, double Kd, double alpha, double FF ) {
    
    _Kp = Kp;
    _Ki = Ki * _Ts;
    _Kd = Kd / _Ts;
    _alpha = alpha;
    _FF = FF;

}