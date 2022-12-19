#include "mbed.h"
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
//#include <dagozilla_msgs/HardwareCommandMsg.h>
//#include <dagozilla_msgs/HardwareStateMsg.h>

/*****************************
        ROS node handle 
 *****************************/
//ros::NodeHandle nh;

/******************************
  Publisher-Subscriber
******************************/
//dagozilla_msgs::HardwareStateMsg stateMsg;
//ros::Publisher statePub("/hardware/state", &stateMsg);
//
//void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg);
//ros::Subscriber<dagozilla_msgs::HardwareCommandMsg> commandSub("/hardware/command", &commandCallback);

//thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//primitive function
void mainProcess();
void getCompass();
//void moveLocomotion();
void moveDribbler();
//void publishMessage();

/*****************************
  Main Function
 *****************************/

int main()
{
    //init ros advertise and subscribe
    //nh.getHardware()->setBaud(115200);
//    nh.initNode();
//    nh.advertise(statePub);
//    nh.subscribe(commandSub);
//    t.start();
    
//    kicker.period(0.01f);
//    kicker = 1;   //deactivate kicker, active LOW

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
//        nh.spinOnce();
        
        //ball distance from IR
        distance = infraRed.read();
        
        //read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1)/PPR_ENC;
        cur_locomotion_L = -locomotionEncL.GetCounter(1)/PPR_ENC;
        cur_locomotion_B = -locomotionEncB.GetCounter(1)/PPR_ENC;
        
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

//        moveLocomotion();
//        moveDribbler();

//        kicker.write(1-kick_power_target);
        
//        publishMessage();
//        nh.spinOnce();
        //if (t - last_timer >=1000){
//            locomotion_R_target_rate =0;
//            locomotion_B_target_rate =0;
//            locomotion_L_target_rate =0;
//            kick_power_target =0;
//            dribbler_state =0;
//        }
        
//        Print Encoder Internal anad external
//        printf("intL = %.3f, intB = %.3f, intR = %.3f\n", rotInL, rotInB, rotInR);
//        printf("extL = %.3f, extB = %.3f, extR = %.3f\n", cur_locomotion_L, cur_locomotion_B, cur_locomotion_R);

//        print encoder dribbler & potentiometer
//          printf("pengL = %.3f, pengR = %.3f\n", cur_dribbler_L, cur_dribbler_R);
//          printf("potL = %.3f , potR = %.3f\n", cur_pot_L, cur_pot_R);

//       print infrared
            printf("Distance = %.2f\n",distance);
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
        //        print theta
//        printf("theta : %.3f\n", theta);
        Thread::wait(60);
        
    }
    
}

void moveLocomotion(){
    locomotionMotorR.setpwm(locomotion_R_target_rate);
    locomotionMotorL.setpwm(locomotion_L_target_rate);
    locomotionMotorB.setpwm(locomotion_B_target_rate);
}

void moveDribbler(){
    dribblerMotorR.setpwm(dribbler_R_target_rate);
    dribblerMotorL.setpwm(dribbler_L_target_rate);    
}

//void publishMessage(){
//    // Publish position data
//    stateMsg.base_right_wheel_position = locomotion_R_rot;    // in m
//    stateMsg.base_back_wheel_position = locomotion_B_rot;     // in m
//    stateMsg.base_left_wheel_position = locomotion_L_rot;     // in m
//    // Publish velocity data
//    stateMsg.base_right_wheel_velocity = locomotion_R_vel;    // in m/s
//    stateMsg.base_back_wheel_velocity = locomotion_B_vel;     // in m/s
//    stateMsg.base_left_wheel_velocity = locomotion_L_vel;     // in m/s
//    // Publish dribbler data
//    stateMsg.dribbler_left_wheel_velocity = dribbler_L_vel;   // in m/s
//    stateMsg.dribbler_right_wheel_velocity = dribbler_R_vel;  // in m/s
//    stateMsg.dribbler_right_potentio_value = cur_pot_R;             // in analog value
//    stateMsg.dribbler_left_potentio_value = cur_pot_L;              // in analog value
//    // Publish distance data
//    stateMsg.dribbler_ir_distance = distance;                 // in analog value
//    stateMsg.compass_angle = theta;
//    statePub.publish(&stateMsg);
//}
//
//void commandCallback(const dagozilla_msgs::HardwareCommandMsg& commandMsg) {
//    locomotion_R_target_rate = commandMsg.base_right_wheel_target_rate;
//    locomotion_B_target_rate = commandMsg.base_back_wheel_target_rate;
//    locomotion_L_target_rate = commandMsg.base_left_wheel_target_rate;
//    dribbler_R_target_rate = commandMsg.dribbler_right_wheel_target_rate;
//    dribbler_L_target_rate = commandMsg.dribbler_left_wheel_target_rate;
//    kick_power_target = commandMsg.kicker_effort;
//    last_timer = t;
//}