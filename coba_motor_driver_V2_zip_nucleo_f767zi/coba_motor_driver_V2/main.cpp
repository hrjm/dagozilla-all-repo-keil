#include "mbed.h"
#include "motor.h"

//format : pin digital (buat nentuin arah), PWM
motor motor1 (PH_1, PB_6);
motor motor3 (PA_12, PB_7);
motor motor2 (PA_10, PB_8);

//motor pengL (PE_15, PE_5);
//motor pengR (PF_15, PE_6);

// Serial
Serial pc(USBTX,USBRX,115200);
 
int main() 
{
   
    while(1) 
    { 
        motor1.setpwm(0.3);
        wait(3);
        motor1.setpwm(-0.3);
        wait(3);
        motor1.setpwm(0);
        wait(1);
        motor2.setpwm(0.3);
        wait(3);
        motor2.setpwm(-0.3);
        wait(3);
        motor2.setpwm(0);
        wait(1);
        motor3.setpwm(0.3);
        wait(3);
        motor3.setpwm(-0.3);
        wait(3);
        motor3.setpwm(0);
        wait(1);
//
//        pengL.setpwm(-0.3);
//        pengR.setpwm(0.3);
        
    }
}
