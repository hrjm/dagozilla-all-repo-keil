#include "mbed.h"
#include "rtos.h"
#include <config.h>
#include <robotpin.h>
#include <variable.h>

// Thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

// Primitive function
void mainProcess();
void getCompass();
void saveCompassCalibration();

/*****************************
        Main Function
 *****************************/

int main()
{   
    
    thread1.start(mainProcess);
    thread2.start(getCompass);
    
    button.rise(&saveCompassCalibration);

    while (true) {
        //Idle 'thread'
    }
}

void mainProcess(){
    
    while(1){
        int calibrationStatus = compass.getCalibrationStatus();
        
        printf("Calibration Status: %d\n", calibrationStatus);
        printf("Current Angle     : %.5f\n", theta);
        printf("********************\n");
        
        Thread::wait(20);
    }
}

void getCompass(){
    float theta0 = compass.readBearing()/10.0;
    Thread::wait(1000);
    theta0 = compass.readBearing()/10.0;
    compassLed = 0;
    while(1){
        float theta_temp = (compass.readBearing()/10.0) - theta0;
        
        if(theta_temp > 180.0 && theta_temp <= 360.0)
            theta_com = (theta_temp - 360.0)/-RADTODEG;    
        else if(theta_temp < -180.0 && theta_temp >= -360.0)
            theta_com = (theta_temp + 360.0)/-RADTODEG;
        else
            theta_com = theta_temp/-RADTODEG;
        
        theta = theta_com;

        if(theta != 0) compassLed = 1;
        else compassLed = 0;

        Thread::wait(60);
        }
}


void saveCompassCalibration() {
    compass.saveCalibrationProfile();
}
