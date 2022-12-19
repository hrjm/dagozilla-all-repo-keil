#include "PID.h"


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
    u0 = - (ku1 * u1 )  - ( ku2*u2 )  + ke0*e0 + ke1*e1 + ke2*e2 + FF*setpoint;

    if (u0 >= 1)
    {
        u0 = 1 ;
    }
    else if (u0 <= -1)
    {
        u0 = -1;
    }
    return u0 ;   
}