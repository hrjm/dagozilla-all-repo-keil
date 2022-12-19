#include "MotorDAGOZ.h"



MotorDagoz :: MotorDagoz ( PinName direction, PinName pwmpin) : _direction(direction), _pwmpin(pwmpin)
{
    _pwmpin.period_us(3000);  
}

void MotorDagoz::setpwm(float inputpwm )
{
    if ( inputpwm >= 0 )
    {
        _direction = 1;
        _pwmpin.write(inputpwm);
    }
    else
    {
        _direction = 0;
        _pwmpin.write(-1*inputpwm);
    }   
}
