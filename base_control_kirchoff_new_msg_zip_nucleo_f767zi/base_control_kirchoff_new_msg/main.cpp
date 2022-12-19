#include "mbed.h"
#include "math.h"
#include <vector>
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dgz_msgs/StampedHardwareCommand.h>
#include <dgz_msgs/StampedHardwareState.h>
#include <std_srvs/SetBool.h>
#include "Clock.h"

/*****************************
        ROS node handle 
 *****************************/
ros::NodeHandle nh;
/******************************
  Publisher-Subscriber
******************************/
dgz_msgs::StampedHardwareState stateMsg;
// stateMsg.data.base_motors_pulse_delta_length = 4;
ros::Publisher statePub("/nucleo/state/hardware", &stateMsg);
ros::ServiceServer set_base_active_srv("/nucleo/command/set_base_active", &setBaseActive);
void commandCallback(const dgz_msgs::StampedHardwareCommand& commandMsg);
ros::Subscriber<dgz_msgs::StampedHardwareCommand> commandSub("/control/command/hardware", &commandCallback);

/******************************
         Service
******************************/
ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> client("control/state/notify_kick");

//thread for RTOS
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//primitive function
void mainProcess();
void getCompass();
void moveLocomotion();
void publishMessage();
void moveDribbler();
bool setBaseActive(std_srvs::SetBool::Request& req, std_srvs::SetBool:Response& res);

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
    nh.serviceClient(client);
    //client = nh.serviceClient<std_srvs::SetBool>("control/state/notify_kick") ;
    t.start();
    //kicker mode
    
    kickerServo.calibrate(range, 0.0); 
    kickerServo = position;
    //kicker setup
    kicker.period(0.02f);
    kicker = 1;   //deactivate kicker, active LOW

    thread1.start(mainProcess);
    thread2.start(getCompass);

    while (true) {
        //do nothing
    }
}

void mainProcess(){
    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    Thread::wait(1000);
    
    while(1){
        
        kick_power_target = 0;
        
        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;
                
        nh.spinOnce();
        
        //ball distance from IR
        ball_distance = infraRed.read();
        
//        read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1);
        cur_locomotion_L = -locomotionEncL.GetCounter(1);
        cur_locomotion_B = -locomotionEncB.GetCounter(1);
        
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
        
        //cur_dribbler_L = -dribblerEncL.GetCounter(1);
//        cur_dribbler_R = -dribblerEncR.GetCounter(1);
        
        moveLocomotion();
        moveDribbler();
        
        if (clock_ms()-time_last_kick > kicker_ready_time)
        {
            kicker.write(1-kick_power_target);
            time_last_kick = clock_ms();
            std_srvs::SetBool::Request req;
            std_srvs::SetBool::Response res;
            req.data = true;
            client.call(req, res);
        }
        else {
            kicker.write(1);
        }
        
        publishMessage();
        
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


void moveLocomotion(){
    locomotionMotorFL.setpwm(locomotion_FL_target_rate);
    locomotionMotorFR.setpwm(locomotion_FR_target_rate);
    locomotionMotorBL.setpwm(locomotion_BL_target_rate);
    locomotionMotorBR.setpwm(locomotion_BR_target_rate);
}

void moveDribbler(){
    dribblerMotorL.setpwm(-dribbler_L_target_rate);
    dribblerMotorR.setpwm(dribbler_R_target_rate);
}

void moveLever(){
    
    switch(kicker_shoot_mode) {
        case 0 : position = 0.055; break;
        case 1 : position = 0.1; break;
    }
    
    kickerServo.calibrate(range, 0.0); 
    kickerServo.write(position);
}


void publishMessage(){

    // Publish position data//
    stateMsg.data.base_motor_1_pulse_delta = rotInFL;
    stateMsg.data.base_motor_2_pulse_delta = rotInFR; 
    stateMsg.data.base_motor_3_pulse_delta = rotInBL;
    stateMsg.data.base_motor_4_pulse_delta = rotInBR;

    stateMsg.data.base_encoder_1_pulse_delta = cur_locomotion_L;
    stateMsg.data.base_encoder_2_pulse_delta = cur_locomotion_R;
    stateMsg.data.base_encoder_3_pulse_delta = cur_locomotion_B;

    stateMsg.data.dribbler_motor_l_pulse_delta = cur_dribbler_L;
    stateMsg.data.dribbler_motor_r_pulse_delta = cur_dribbler_R;

    stateMsg.data.dribbler_potentio_l_reading = 0.0;
    stateMsg.data.dribbler_potentio_r_reading = 0.0;

    stateMsg.data.ir_reading = ball_distance;
    stateMsg.data.compass_reading = theta_result;

    stateMsg.header.stamp = nh.now();
    
    statePub.publish(&stateMsg);
}

void commandCallback(const dgz_msgs::StampedHardwareCommand& commandMsg) {
      
    locomotion_FL_target_rate = commandMsg.data.base_motor_1_pwm;
    locomotion_FR_target_rate = commandMsg.data.base_motor_2_pwm;
    locomotion_BL_target_rate = commandMsg.data.base_motor_3_pwm;
    locomotion_BR_target_rate = commandMsg.data.base_motor_4_pwm;

    dribbler_L_target_rate = commandMsg.data.dribbler_motor_l_pwm;
    dribbler_R_target_rate = commandMsg.data.dribbler_motor_r_pwm;

    kick_power_target = commandMsg.data.kicker_pwm;
    kicker_shoot_mode = commandMsg.data.is_shoot;
    
    last_timer = t;
}

bool setBaseActive(std_srvs::SetBool::Request& req, std_srvs::SetBool:Response& res)
{
    res.success = true;
    res.message = "success";
    return res.success;
}

void getCompass(){
    //printf("test\n");
    //check compass
    bool startupPassed = false;
    compass._reset = 0;
    while (!startupPassed) {
        compass.reset();
        startupPassed = compass.init();
        Thread::wait(100);
    }
    compass._reset = 1;
    //check compass healthy
    //startupPassed = false;
    
    //while(startupPassed == false){
//        if (compass.bno055Healthy()){
//            pc.printf("\nBNO055 is Healthy..");
//            startupPassed = true;
//        } else {
//            pc.printf("\nBNO055 Has an Error..");  
//            //compassLed = 0;
////            wait(500);
////            compassLed = 1;
//            compass.reset();
//            compass.init(); 
//        }
//        Thread::wait(500);
//    }
    //initialize compass
   // compass.init();
    Thread::wait(100);
    Euler e = compass.getEulerAngles();
    float theta0 = e.heading;
    Thread::wait(100);
    while(1){
        
        //nh.spinOnce();
        
        if(compass.check()){
            theta_result = compass.getThetaDegree(theta0);
//            pc.printf("Theta : %.2f\n", theta_result);
            compass._reset = 1;
        }
        else{
            compass.reset();
            //reinitializing
            compass.init();
            compass._reset = 0;
        }
        Thread::wait(50);   
    }
}

/*
void mainProcess(){ //Kicker feedback
    bool kick_executed = 0;
    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    Thread::wait(1000);
    
    while(1){
        // Turn off kicker        
        kick_power_target = 0;
        kicker.write(1 - kick_power_target);
        
        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;
                
        nh.spinOnce();
        
        // Reset kick_executed once kick_power_target == 0
        if (kick_executed && kick_power_target == 0) {
            kick_executed = 0;   
        }
        
        //ball distance from IR
        ball_distance = infraRed.read();
        
        //read encoder
//        cur_locomotion_R = -locomotionEncR.GetCounter(1);
//        cur_locomotion_L = -locomotionEncL.GetCounter(1);
        
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
        
//        cur_dribbler_L = -dribblerEncL.GetCounter(1);
//        cur_dribbler_R = -dribblerEncR.GetCounter(1);
        
        moveLocomotion();
        moveDribbler();
        
        // If kick is not executed and kick_power_target > 0, kick
        if (kick_power_target > 0 && !kick_executed) {
            kicker.write(1-kick_power_target);
            
            kick_executed = 1;
        }
        
        publishMessage();
        
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
*/