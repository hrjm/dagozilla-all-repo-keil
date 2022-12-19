#include "mbed.h"
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dagozilla_msgs/HardwareCommand.h>
#include <dagozilla_msgs/HardwareState.h>

/*****************************
        ROS node handle 
 *****************************/
ros::NodeHandle nh;

/******************************
  Publisher-Subscriber
******************************/
dagozilla_msgs::HardwareState stateMsg;
ros::Publisher statePub("/nucleo/state/hardware", &stateMsg);

void commandCallback(const dagozilla_msgs::HardwareCommand& commandMsg);
ros::Subscriber<dagozilla_msgs::HardwareCommand> commandSub("/control/command/hardware", &commandCallback);

//thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//primitive function
void mainProcess();
void getCompass();
void moveLocomotion();
void publishMessage();
void moveDribbler();

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
//    thread2.start(getCompass);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    Thread::wait(1000);
    
    while(1){
        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;
                
        nh.spinOnce();
        
        //ball distance from IR
        distance = infraRed.read();
        
        //read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1);
        cur_locomotion_L = -locomotionEncL.GetCounter(1);
        
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
        
        cur_dribbler_L = -dribblerEncL.GetCounter(1);
        cur_dribbler_R = -dribblerEncR.GetCounter(1);
        
        moveLocomotion();
        moveDribbler();
        
        kicker.write(1-kick_power_target);
        
        publishMessage();
        nh.spinOnce();
        
        if (t - last_timer >=1000){
            
            locomotion_FL_target_rate = 0;
            locomotion_FR_target_rate = 0;
            locomotion_BL_target_rate = 0;
            locomotion_BR_target_rate = 0;
            
            dribbler_L_target_rate = 0;
            dribbler_R_target_rate = 0;
            
            kick_power_target = 1;
            kicker_shoot_mode = 0;
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
        Thread::wait(50);
    }
}

void moveLocomotion(){
    locomotionMotorFL.setpwm(locomotion_FL_target_rate);
    locomotionMotorFR.setpwm(locomotion_FR_target_rate);
    locomotionMotorBL.setpwm(locomotion_BL_target_rate);
    locomotionMotorBR.setpwm(locomotion_BR_target_rate);
}

void moveDribbler(){
    dribblerMotorL.setpwm(dribbler_L_target_rate);
    dribblerMotorR.setpwm(dribbler_R_target_rate);
}

void publishMessage(){

    // Publish position data
    stateMsg.base_motor_1_pulses = rotInFL;
    stateMsg.base_motor_2_pulses = rotInFR; 
    stateMsg.base_motor_3_pulses = rotInBL;
    stateMsg.base_motor_4_pulses = rotInBR;
    
    stateMsg.base_encoder_1_pulses = cur_locomotion_L;
    stateMsg.base_encoder_2_pulses = cur_locomotion_R;
    stateMsg.base_encoder_3_pulses = 0;
    
    stateMsg.dribbler_motor_l_pulses = cur_dribbler_L;
    stateMsg.dribbler_motor_r_pulses = cur_dribbler_R;
    
    stateMsg.dribbler_potentio_l_reading = cur_pot_L;
    stateMsg.dribbler_potentio_r_reading = cur_pot_R;
    
    stateMsg.ir_reading = distance;
    stateMsg.compass_reading = theta;
    
    statePub.publish(&stateMsg);
}

void commandCallback(const dagozilla_msgs::HardwareCommand& commandMsg) {
      
    locomotion_FL_target_rate = commandMsg.base_motor_1_pwm;
    locomotion_FR_target_rate = commandMsg.base_motor_2_pwm;
    locomotion_BL_target_rate = commandMsg.base_motor_3_pwm;
    locomotion_BR_target_rate = commandMsg.base_motor_4_pwm;
    
    dribbler_L_target_rate = commandMsg.dribbler_motor_l_pwm;
    dribbler_R_target_rate = commandMsg.dribbler_motor_r_pwm;
    
    kick_power_target = commandMsg.kicker_pwm;
    kicker_shoot_mode = commandMsg.kicker_mode;
    
    last_timer = t;
}
