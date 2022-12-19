#include "mbed.h"
#include "PID.h"
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

int main()
{
//init ros advertise and subscribe
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(statePub);
    nh.subscribe(commandSub);
    t.start();
    
    kicker.period(0.01f);
    kicker = 1;   //deactivate kicker, active LOW

    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    float cur_pot_L0 = ((float)dribblerPotL.read()*10.0f);
    float cur_pot_R0 = ((float)dribblerPotR.read()*10.0f);
    Thread::wait(1000);
    
    while(1){
        cur_pot_L = -((float)dribblerPotL.read()*10.0f-cur_pot_L0);
        cur_pot_R = ((float)dribblerPotR.read()*10.0f-cur_pot_R0);
        nh.spinOnce();
        
        //ball distance from IR
        distance = infraRed.read();
        
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

        moveLocomotion();
        moveDribbler();

        kicker.write(1-kick_power_target);
        
        publishMessage();
        nh.spinOnce();
        if (t - last_timer >=1000){
            locomotion_R_vtarget =0;
            locomotion_B_vtarget =0;
            locomotion_L_vtarget =0;
            kick_power_target =0;
            dribbler_state =0;
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
            theta_com = ((compass.readBearing()/10.0) - 360.0 - theta0)/-RADTODEG;    
        else if(theta_temp < -180.0 && theta_temp >= -360.0)
            theta_com = ((compass.readBearing()/10.0) + 360.0 - theta0)/-RADTODEG;
        else
            theta_com = ((compass.readBearing()/10.0) - theta0)/-RADTODEG;
        
        theta = theta_com;

        if(fabs(theta) >= 10.0/RADTODEG) compassLed = 1;
        else compassLed = 0;
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
    dribbler_R_target_rate = dribblerPidR.createpwm(dribbler_R_vtarget, dribbler_R_vel);
    dribbler_L_target_rate = dribblerPidL.createpwm(dribbler_L_vtarget, dribbler_L_vel);
    //motor pwm
    dribblerMotorR.setpwm(dribbler_R_target_rate);
    dribblerMotorL.setpwm(dribbler_L_target_rate);
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
    stateMsg.dribbler_left_wheel_velocity = dribbler_L_vel;   // in m/s
    stateMsg.dribbler_right_wheel_velocity = dribbler_R_vel;  // in m/s
    stateMsg.dribbler_right_potentio_value = cur_pot_R;             // in analog value
    stateMsg.dribbler_left_potentio_value = cur_pot_L;              // in analog value
    // Publish distance data
    stateMsg.dribbler_ir_distance = distance;                 // in analog value
    stateMsg.compass_angle = theta;
    statePub.publish(&stateMsg);
}

void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
    locomotion_R_vtarget = commandMsg.base_right_wheel_target_rate;
    locomotion_B_vtarget = commandMsg.base_back_wheel_target_rate;
    locomotion_L_vtarget = commandMsg.base_left_wheel_target_rate;
    dribbler_R_vtarget = commandMsg.dribbler_right_wheel_target_rate;
    dribbler_L_vtarget = commandMsg.dribbler_left_wheel_target_rate;
    kick_power_target = commandMsg.kicker_effort;
    last_timer = t;
}