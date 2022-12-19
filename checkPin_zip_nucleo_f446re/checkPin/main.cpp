#include "mbed.h"
 
DigitalOut right_switch1(PC_8);
DigitalOut right_switch2(PC_6);
DigitalOut right_switch3(PA_2);
DigitalOut right_switch4(PC_0);
DigitalOut right_switch5(PC_1);
DigitalOut right_switch6(PC_15);
DigitalOut right_switch7(PC_3);
DigitalOut right_switch8(PC_2);
DigitalOut right_switch9(PC_14);
AnalogOut pwm_right(PB_6);
//AnalogOut pwm_left(PB_14);
//AnalogOut pwm_up(PB_15);
 
int main() {
    uint16_t sample;
    while(1) {
        right_switch1 = 1;
        right_switch2 = 1;
        right_switch3 = 1;
        right_switch4 = 1;
        right_switch5 = 1;
        right_switch6 = 1;
        right_switch7 = 1;
        right_switch8 = 1;
        right_switch9 = 1;
        sample = (uint16_t) (0.5);
        pwm_right.write(0.5);
//        pwm_left.write(0.5);
//        pwm_up.write(0.5);        
        
    }
}