#include "Servo.h"
#include<stdio.h>
#include"mbed.h"

Serial pc(USBTX, USBRX, 115200);

float range = 0.0011;
float position = 0;
float degrees;
Servo kickerServo(PC_7);

int main() {
    kickerServo.calibrate(0.05,0);
    kickerServo.position(position);
    position = 0.05;
    while (1) {
        kickerServo.calibrate(range, 0.0);
        kickerServo = position;
        pc.printf("%f\n", position);
        if (position >= 0.21) {
            position = 0.05;
        }
        else {
            position += 0.3;
        }
        wait(2.0);
        
    }
    }
    