#include "mbed.h"
//#include <PIDDagoz/PID.h>
#include <Motor/Motor.h>
#include <ta_utils/robotpin.h>
#include <ta_utils/config.h>
#include <ta_utils/variable.h>
#include <ros.h>
#include <ta_msgs/HardwareCommandMsg.h>
#include <ta_msgs/HardwareStateMsg.h>

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

////PID global object PID, P I D N TS FF MODE
//PID locomotionPidR(right_kp, right_ki, right_kd, right_N, PID_TS, right_FF, PID::PI_MODE);  //Right locomotion
//PID locomotionPidL(left_kp, left_ki, left_kd, left_N, PID_TS, left_FF, PID::PI_MODE);       //Left locomotion
//PID locomotionPidB(back_kp, back_ki, back_kd, back_N, PID_TS, back_FF, PID::PI_MODE);       //Back locomotion

//thread for RTOS
Thread thread1;
// Thread therad2;

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
        } else{
            buzzer1 = 1;
            buzzer2 = 1;
        }
        
        nh.spinOnce();   
        Thread::wait(MAIN_TS);
    }
}

void moveLocomotion(){
    // Calculate motor pwm
//    locomotion_R_target_rate = locomotionPidR.createpwm(locomotion_R_vtarget, locomotion_R_vel);
//    locomotion_L_target_rate = locomotionPidL.createpwm(locomotion_L_vtarget, locomotion_L_vel);
//    locomotion_B_target_rate = locomotionPidB.createpwm(locomotion_B_vtarget, locomotion_B_vel);
    
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

void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
     locomotion_R_target_rate = commandMsg.base_right_wheel_target_rate;
     locomotion_B_target_rate = commandMsg.base_back_wheel_target_rate;
     locomotion_L_target_rate = commandMsg.base_left_wheel_target_rate;
     buzzer_state = commandMsg.buzzer_state;
}