#include "mbed.h"
#include "PID.h"
#include "millis.h"
// #include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
// #include <dagozilla_msgs/HardwareCommandMsg.h>
// #include <dagozilla_msgs/HardwareStateMsg.h>

/*****************************
        ROS node handle 
 *****************************/
// ros::NodeHandle nh;

// /******************************
//   Publisher-Subscriber
// ******************************/
// dagozilla_msgs::HardwareStateMsg stateMsg;
// ros::Publisher statePub("/hardware/state", &stateMsg);

// void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg);
// ros::Subscriber<dagozilla_msgs::HardwareCommandMsg> commandSub("/hardware/command", &commandCallback);

//PID global object PID, P I D N TS FF MODE
PID locomotionPidR(KP_R, KI_R, KD_R, N_R, TS, FF_R, PID::PI_MODE);  //Right locomotion PID
PID locomotionPidB(KP_B, KI_B, KD_B, N_B, TS, FF_B, PID::PI_MODE);  //Back locomotion 
PID locomotionPidL(KP_L, KI_L, KD_L, N_L, TS, FF_L, PID::PI_MODE);  //Left locomotion PID
PID dribblerPidR(KP_DR, KI_DR, KD_DR, N_DR, TS, FF_DR, PID::PI_MODE);  //Right dribbler PID
PID dribblerPidL(KP_DL, KI_DL, KD_DL, N_DL, TS, FF_DL, PID::PI_MODE);  //Left dribbler PID

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
long int prev_idle = 0;
long int prev_main = 0;
long int prev_compass = 0;
int main()
{
//init ros advertise and subscribe
    // nh.initNode();
    // nh.advertise(statePub);
    // nh.subscribe(commandSub);
    t.start();
    startMillis();
    kicker.period(0.01f);
    kicker = 1;   //deactivate kicker, active LOW

    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //do nothing
        pc.printf("IDLE: %d\n", (millis()));
        prev_idle = millis();
    }
}

void mainProcess(){
    float cur_pot_L0 = ((float)dribblerPotL.read()*10.0f);
    float cur_pot_R0 = ((float)dribblerPotR.read()*10.0f);
    Thread::wait(1000);
    
    while(1){
        cur_pot_L = -((float)dribblerPotL.read()*10.0f-cur_pot_L0);
        cur_pot_R = ((float)dribblerPotR.read()*10.0f-cur_pot_R0);
        // nh.spinOnce();
        
        // //ball distance from IR
        // distance = infraRed.read();
        
        //read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1)/4000.0;
        cur_locomotion_L = -locomotionEncL.GetCounter(1)/4000.0;
        cur_locomotion_B = -locomotionEncB.GetCounter(1)/4000.0;
        
        rotInB = intEncR.getRevolutions();
        rotInR = intEncL.getRevolutions();
        rotInL = intEncB.getRevolutions();
        
        cur_dribbler_R = -dribblerEncR.GetCounter(1)/537.6;
        cur_dribbler_L = dribblerEncL.GetCounter(1)/537.6;
        
        // Correct encoder unit
        locomotion_R_rot = cur_locomotion_R * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_L_rot = cur_locomotion_L * LOCOMOTIONWHEEL * 2 * PI; // in m
        locomotion_B_rot = cur_locomotion_B * LOCOMOTIONWHEEL * 2 * PI; // in m
        dribbler_R_rot = cur_dribbler_L * DRIBBLERWHEEL * 2 * PI; // in m
        dribbler_L_rot = cur_dribbler_R * DRIBBLERWHEEL * 2 * PI; // in m
        
        // Calculate acutal velocity
        locomotion_R_vel = (rotInR*2*PI*0.05) / 0.02;
        locomotion_L_vel = (rotInL*2*PI*0.05) / 0.02;
        locomotion_B_vel = (rotInB*2*PI*0.05) / 0.02;
        dribbler_R_vel = dribbler_R_rot / 0.02;
        dribbler_L_vel = dribbler_L_rot / 0.02;
        locomotion_B_vtarget = 1.0;

        moveLocomotion();
        moveDribbler();
        // kicker.write(1-kick_power_target);
        
        // publishMessage();
        // nh.spinOnce();
        // if (t - last_timer >=1000){
        //     locomotion_R_vtarget =0;
        //     locomotion_B_vtarget =0;
        //     locomotion_L_vtarget =0;
        //     kick_power_target =0;
        //     dribbler_state =0;
        // }
        pc.printf("MAIN: %d\n", (millis()));
        prev_main = millis();
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
            theta_com = ((compass.readBearing()/10.0) - 360.0 - theta0)/-RADTODEG;    
        else if(theta_temp < -180.0 && theta_temp >= -360.0)
            theta_com = ((compass.readBearing()/10.0) + 360.0 - theta0)/-RADTODEG;
        else
            theta_com = ((compass.readBearing()/10.0) - theta0)/-RADTODEG;
        
        theta = theta_com;

        if(fabs(theta) >= 10.0/RADTODEG) compassLed = 1;
        else compassLed = 0;
        pc.printf("COMPASS: %d\n", (millis()));
        prev_compass = millis();
        Thread::wait(50);
    }
}

void moveLocomotion(){
    // Calculate motor pwm
    locomotion_R_target_rate = locomotionPidR.createpwm(locomotion_R_vtarget, locomotion_R_vel);
    locomotion_L_target_rate = locomotionPidL.createpwm(locomotion_L_vtarget, locomotion_L_vel);
    locomotion_B_target_rate = locomotionPidB.createpwm(locomotion_B_vtarget, locomotion_B_vel);
    
    //Motor pwm    
    locomotionMotorR.setpwm(locomotion_R_target_rate);
    locomotionMotorL.setpwm(locomotion_L_target_rate);
    locomotionMotorB.setpwm(locomotion_B_target_rate);
}

void moveDribbler(){
//     if(dribbler_state){
//         float right_vtarget;
//         float left_vtarget;
    
// //        robot velocity
//         vy = (locomotion_R_vel*0.57731) - (locomotion_L_vel*0.57731);
//         vx = (2*locomotion_B_vel/3) - (locomotion_R_vel/3) - (locomotion_L_vel/3);
//         vw = (locomotion_R_vel + locomotion_L_vel + locomotion_B_vel)/(3*0.2);
// //        conditional for x
//         if(vx<0){
//             left_from_vx = 0.6;
//             right_from_vx = 0.8;
//         } else if(vx >= 0){
//             left_from_vx = 0.8;
//             right_from_vx = 0.6;
//         }
    
// //      conditional for vy
//         if(vy>0){
//             left_from_vy = -2;
//             right_from_vy= 2;
//         }
//         else {
//             left_from_vy = -2.5;
//             right_from_vy= -2.5;
//         }
        
//         //from potensio
//         float right_v1 = (right_pot_target - cur_pot_L) * right_from_potensio;
//         float left_v1 = (left_pot_target - cur_pot_R) * left_from_potensio;

//         //error tolerance
//         if(fabs(right_pot_target - cur_pot_L)<tolerance) right_v1 = 0;
//         if(fabs(left_pot_target - cur_pot_R)<tolerance) left_v1 = 0;
        
//         //condition that no ball in dribbler
//         if(fabs(cur_pot_L)<=0.2 && fabs(cur_pot_R)<=0.2){
//             right_vtarget = right_v1;
//             left_vtarget = left_v1;
//         } else{ //ball detected
//            //from robot velocity
//             right_vtarget = right_v1 + (right_from_vx*fabs(vx) + right_from_vy*vy + right_from_rot*fabs(vw));
//             left_vtarget = left_v1 + (left_from_vx*fabs(vx) + left_from_vy*vy + left_from_rot*fabs(vw));
//         }
//     //
//         //compute PID
//         float dribbler_R_target_rate = dribblerPidR.createpwm(right_vtarget, dribbler_R_vel);
//         float dribbler_L_target_rate = dribblerPidL.createpwm(left_vtarget, dribbler_L_vel);
//         //motor pwm
//         dribblerMotorR.setpwm(dribbler_R_target_rate);
//         dribblerMotorL.setpwm(dribbler_L_target_rate);

// //        dribblerMotorR.setpwm(0.3);
// //        dribblerMotorL.setpwm(0.3);
        
//         //print PWM
// //        pc.printf("vL = %.2f; vTarget_L = %.2f; LPWM = %.2f; vR = %.2f; vTarget_R = %.2f; RPWM = %.2f; L = %.2f; R = %.2f\n", dribbler_L_vel, left_vtarget, dribbler_L_target_rate, dribbler_R_vel, right_vtarget, dribbler_R_target_rate, cur_pot_L, cur_pot_R);
//     }
//     else{
//         dribblerMotorR.setpwm(0);
//         dribblerMotorL.setpwm(0);
//     }
}

// void publishMessage(){
//     // Publish position data
//     stateMsg.base_wheel_right_position = locomotion_R_rot;    // in m
//     stateMsg.base_wheel_back_position = locomotion_B_rot;     // in m
//     stateMsg.base_wheel_left_position = locomotion_L_rot;     // in m
//     // Publish velocity data
//     stateMsg.base_wheel_right_velocity = locomotion_R_vel;    // in m/s
//     stateMsg.base_wheel_back_velocity = locomotion_B_vel;     // in m/s
//     stateMsg.base_wheel_left_velocity = locomotion_L_vel;     // in m/s
//     // Publish distance data
//     stateMsg.dribbler_ir_distance = distance;               // in analog value
//     stateMsg.compass_angle = theta;
//     stateMsg.dribbler_active = dribbler_state;
//     statePub.publish(&stateMsg);
// }

// void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
//     locomotion_R_vtarget = commandMsg.base_wheel_right_velocity;
//     locomotion_B_vtarget = commandMsg.base_wheel_back_velocity;
//     locomotion_L_vtarget = commandMsg.base_wheel_left_velocity;
//     kick_power_target = commandMsg.kicker_effort;
//     dribbler_state = commandMsg.dribbler_active;
//     last_timer = t;
// }