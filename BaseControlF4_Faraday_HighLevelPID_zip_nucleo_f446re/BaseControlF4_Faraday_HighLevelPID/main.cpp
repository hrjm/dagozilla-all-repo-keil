#include "mbed.h"
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dagozilla_msgs/HardwareCommandMsg.h>
#include <dagozilla_msgs/HardwareStateMsg.h>

/*****************************
        ROS node handle 
 *****************************/
ros::NodeHandle nh;

/******************************
    Publisher-Subscriber
******************************/
dagozilla_msgs::HardwareStateMsg stateMsg;
ros::Publisher statePub("/hardware/state", &stateMsg);

void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg);
ros::Subscriber<dagozilla_msgs::HardwareCommandMsg> commandSub("/hardware/command", &commandCallback);

//thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//primitive function
void mainProcess();
void getCompass();
void moveLocomotion();
void moveDribbler();
void publishMessage();

/*****************************
        Main Function
 *****************************/

int main()
{
    //init ros advertise and subscribe
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(statePub);
    nh.subscribe(commandSub);
    t.start();
    
    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    
    while(1){
        nh.spinOnce();
        
        //read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1)/PPR_ENC;
        cur_locomotion_B = -locomotionEncB.GetCounter(1)/PPR_ENC;
        cur_locomotion_L = -locomotionEncL.GetCounter(1)/PPR_ENC;
        
        rotInB = intEncR.getRevolutions();
        rotInR = intEncL.getRevolutions();
        rotInL = intEncB.getRevolutions();
        
        // Correct encoder unit
        locomotion_R_rot = cur_locomotion_R * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_L_rot = cur_locomotion_L * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_B_rot = cur_locomotion_B * LOCOMOTIONWHEEL * 2 * PI; // in m
        
        // Calculate acutal velocity
        locomotion_R_vel = (rotInR*2*PI*0.05) / 0.02;
        locomotion_L_vel = (rotInL*2*PI*0.05) / 0.02;
        locomotion_B_vel = (rotInB*2*PI*0.05) / 0.02;
        
        moveLocomotion();
        publishMessage();
        nh.spinOnce();        
        
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
        Thread::wait(60);
    }
}

void moveLocomotion(){ 
    //Motor pwm    
    locomotionMotorR.setpwm(locomotion_R_target_rate);
    locomotionMotorL.setpwm(locomotion_L_target_rate);
    locomotionMotorB.setpwm(locomotion_B_target_rate);
}

void publishMessage(){
    // Publish position data
    stateMsg.base_right_wheel_position = locomotion_R_rot;    // in m
    stateMsg.base_back_wheel_position = locomotion_B_rot;     // in m
    stateMsg.base_left_wheel_position = locomotion_L_rot;     // in m
    // Publish velocity data
    stateMsg.base_right_wheel_velocity = locomotion_R_vel;    // in m/s
    stateMsg.base_back_wheel_velocity = locomotion_B_vel;     // in m/s
    stateMsg.base_left_wheel_velocity = locomotion_L_vel;     // in m/s
    // Publish dribbler data
    stateMsg.dribbler_left_wheel_velocity = 0.0;
    stateMsg.dribbler_right_wheel_velocity = 0.0;
    stateMsg.dribbler_right_potentio_value = 0.0;
    stateMsg.dribbler_left_potentio_value = 0.0;
    // Publish distance data
    stateMsg.dribbler_ir_distance = 0.0;                 // in analog value
    stateMsg.compass_angle = theta;
    statePub.publish(&stateMsg);
}

void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
    locomotion_R_target_rate = commandMsg.base_right_wheel_target_rate;
    locomotion_B_target_rate = commandMsg.base_back_wheel_target_rate;
    locomotion_L_target_rate = commandMsg.base_left_wheel_target_rate;
    
    last_timer = t;
}