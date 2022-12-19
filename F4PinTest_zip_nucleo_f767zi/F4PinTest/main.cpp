#include "mbed.h"
#include "rtos.h"
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dagozilla_msgs/HardwareCommandMsg.h>
#include <dagozilla_msgs/HardwareStateMsg.h>

//thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//primitive function
void mainProcess();
void getCompass();
void moveLocomotion();
void moveDribbler();

/*****************************
        Main Function
 *****************************/

int main()
{
    //init ros advertise and subscribe
    t.start();
    
    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    
    while(1){
        
        //read encoder        
        rotInB = intEncR.getRevolutions();
        rotInR = intEncL.getRevolutions();
        rotInL = intEncB.getRevolutions();
        
        // Correct encoder unit
        locomotion_R_rot = rotInR * 2 * PI * 0.05; // in m
        locomotion_L_rot = rotInL * 2 * PI * 0.05; // in m
        locomotion_B_rot = rotInB * 2 * PI * 0.05; // in m
        
        // Calculate acutal velocity
        locomotion_R_vel = locomotion_R_rot / 0.02; // in m/s
        locomotion_L_vel = locomotion_L_rot / 0.02; // in m/s
        locomotion_B_vel = locomotion_B_rot / 0.02; // in m/s
        
        moveLocomotion();
        pwm_right = 0.5;
        pwm_left = 0.5;
        pwm_up = 0.5;
        
        if(t - last_timer >=1000){
            locomotion_R_target_rate =0;
            locomotion_B_target_rate =0;
            locomotion_L_target_rate =0;
        }
        
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

        if(fabs(theta) >= 10.0/RADTODEG) compassLed = 1;
        else compassLed = 0;
        
        pc.printf("%.3f\n", theta);
        
        Thread::wait(60);
    }
}

void moveLocomotion(){ 
    //Motor pwm    
    locomotionMotorR.setpwm(locomotion_R_target_rate);
    locomotionMotorL.setpwm(locomotion_L_target_rate);
    locomotionMotorB.setpwm(locomotion_B_target_rate);
}
