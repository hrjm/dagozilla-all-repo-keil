#include "mbed.h"
#include "rtos.h"
#include <config.h>
#include <robotpin.h>
#include <variable.h>
#include <ros.h>
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

// Thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

// Primitive function
void mainProcess();
void getCompass();
void moveLocomotion();
void moveDribbler();
void publishMessage();
void loadRobotConfig();

/*****************************
        Main Function
 *****************************/

int main()
{
    // Init ROS advertise and subscribe
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    nh.advertise(statePub);
    nh.subscribe(commandSub);

    // Wait until Nucleo is connected to PC
    while(!nh.connected()){
        nh.spinOnce();
    }

    loadRobotConfig();

    t.start();
    
    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //Idle 'thread'
    }
}

void mainProcess(){
    
    while(1){
        nh.spinOnce();
        
        // Read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1)/4096.0;
        cur_locomotion_B = -locomotionEncB.GetCounter(1)/4096.0;
        cur_locomotion_L = -locomotionEncL.GetCounter(1)/4096.0;
        
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
            locomotion_R_vtarget = 0;
            locomotion_B_vtarget = 0;
            locomotion_L_vtarget = 0;
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

        if(theta != 0) compassLed = 1;
        else compassLed = 0;

        Thread::wait(60);
        }
}

void moveLocomotion(){
    double locomotion_R_PWM, locomotion_L_PWM, locomotion_B_PWM;

    if (!high_level_pid) {
        locomotion_R_PWM = locomotionMotorRController.createpwm(locomotion_R_target_rate, locomotion_R_vel);
        locomotion_L_PWM = locomotionMotorLController.createpwm(locomotion_L_target_rate, locomotion_L_vel);
        locomotion_B_PWM = locomotionMotorBController.createpwm(locomotion_B_target_rate, locomotion_B_vel);
    }
    else {
        locomotion_R_PWM = locomotion_R_target_rate;
        locomotion_L_PWM = locomotion_L_target_rate;
        locomotion_B_PWM = locomotion_B_target_rate;
    }

    // Motor PWM
    locomotionMotorR.setpwm(locomotion_R_PWM);
    locomotionMotorL.setpwm(locomotion_L_PWM);
    locomotionMotorB.setpwm(locomotion_B_PWM);
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
    // Publish orientation data
    stateMsg.compass_angle = theta;
    statePub.publish(&stateMsg);
}

void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
    locomotion_R_target_rate = commandMsg.base_right_wheel_target_rate;
    locomotion_B_target_rate = commandMsg.base_back_wheel_target_rate;
    locomotion_L_target_rate = commandMsg.base_left_wheel_target_rate;
    
    last_timer = t;
}

void loadRobotConfig(){
    if(!nh.getParam("/use_pid", &high_level_pid));

    if(!high_level_pid){
        
        // Load PID config
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/right_wheel/proportional", &Kp_R));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/right_wheel/integrator", &Ki_R));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/right_wheel/derivative", &Kd_R));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/right_wheel/filter_coefficient", &alpha_R));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/right_wheel/feedforward", &FF_R));

        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/back_wheel/proportional", &Kp_B));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/back_wheel/integrator", &Ki_B));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/back_wheel/derivative", &Kd_B));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/back_wheel/filter_coefficient", &alpha_B));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/back_wheel/feedforward", &FF_B));

        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/left_wheel/proportional", &Kp_L));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/left_wheel/integrator", &Ki_L));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/left_wheel/derivative", &Kd_L));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/left_wheel/filter_coefficient", &alpha_L));
        if(!nh.getParam("/dagozilla_base_control_test/hardware_profile/faraday/base/left_wheel/feedforward", &FF_L));

        locomotionMotorRController.setTunings(Kp_R, Ki_R, Kd_R, alpha_R, FF_R);
        locomotionMotorLController.setTunings(Kp_L, Ki_L, Kd_L, alpha_L, FF_L);
        locomotionMotorBController.setTunings(Kp_B, Ki_B, Kd_B, alpha_B, FF_B);
    }
    
}
