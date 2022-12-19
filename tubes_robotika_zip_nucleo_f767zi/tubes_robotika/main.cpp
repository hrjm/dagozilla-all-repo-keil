#include "mbed.h"
#include <Motor/Motor.h>
#include <tubes_utils/robotpin.h>
#include <tubes_utils/config.h>
#include <tubes_utils/variable.h>
#include <ros.h>
#include <tubes_msgs/HardwareCommandMsg.h>
#include <tubes_msgs/HardwareStateMsg.h>

/*****************************
        ROS node handle 
 *****************************/
 ros::NodeHandle nh;

/******************************
  Publisher-Subscriber
******************************/
tubes_msgs::HardwareStateMsg stateMsg;
ros::Publisher statePub("/hardware/state", &stateMsg);

void commandCallback(const tubes_msgs::HardwareCommandMsg& commandMsg);
ros::Subscriber<tubes_msgs::HardwareCommandMsg> commandSub("/hardware/command", &commandCallback);

//thread for RTOS
Thread thread1;
Thread thread2;

//primitive function
void mainProcess();
void moveLocomotion();
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

    thread1.start(mainProcess);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    Thread::wait(1000);

    while(1){
        nh.spinOnce();
        //read encoder
        cur_locomotion_R = locomotionEncR.GetCounter(1)/WHEELS_PPR;
        cur_locomotion_L = locomotionEncL.GetCounter(1)/WHEELS_PPR;
        cur_locomotion_B = locomotionEncB.GetCounter(1)/WHEELS_PPR;
        // Correct encoder unit
        locomotion_R_rot = cur_locomotion_R * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_L_rot = cur_locomotion_L * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_B_rot = cur_locomotion_B * LOCOMOTIONWHEEL * 2 * PI; // in m
        // Calculate acutal velocity
        locomotion_R_vel = locomotion_R_rot/0.01;
        locomotion_L_vel = locomotion_L_rot/0.01;
        locomotion_B_vel = locomotion_B_rot/0.01;
        
        moveLocomotion();
        publishMessage();
        
        if(buzzer_state){
            buzzer1 = 0;
            buzzer2 = 0;
        } 
        else{
            buzzer1 = 1;
            buzzer2 = 1;
        }
        
        nh.spinOnce();
    
        Thread::wait(MAIN_TS);
    }
}

void moveLocomotion(){
    //Motor pwm    
    locomotionMotorR.speed(locomotion_R_target_rate);
    locomotionMotorL.speed(locomotion_L_target_rate);
    locomotionMotorB.speed(locomotion_B_target_rate);
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
    statePub.publish(&stateMsg);
}

void commandCallback(const tubes_msgs::HardwareCommandMsg& commandMsg) {
     locomotion_R_target_rate = commandMsg.base_right_wheel_target_rate;
     locomotion_B_target_rate = commandMsg.base_back_wheel_target_rate;
     locomotion_L_target_rate = commandMsg.base_left_wheel_target_rate;
     buzzer_state = commandMsg.buzzer_state;
}