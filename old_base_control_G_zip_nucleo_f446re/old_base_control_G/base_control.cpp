// Main code for Nucleo

#include "mbed.h"
#include "Motor.h"
#include "EncoderMotor.h"
#include "PID.h"
#include <ros.h>
#include <rosserial_mbed/NucleoMsg.h>
#include <rosserial_mbed/NucleoCmd.h>
#include <rosserial_mbed/LedMsg.h>

// Constants
const double sampling_time = 0.02; // second
const double pi = 3.141593;
const double wheel_radius = 0.05; // metres

/*****************************
  ROS node handle 
 *****************************/
ros::NodeHandle nh;

/******************************
  Motor Initialization
  Motor
  EncoderMotor
      Variabel encoder PPR PG45 dan PG26 yang digunakan sama = 573.6
      PPR Total = PPR Encoder * Mode Encoding * Gear Reduction

!!  Pin is now correct
******************************/
PwmOut kicker(PB_7); //pin D3
AnalogIn analog_value(PC_3);

Motor wheel1 (PA_9, PB_10, PA_8);
Motor wheel2 (PA_7, PB_6, PC_7);
Motor wheel3 (PA_5, PA_6, PC_8);
Motor dribbleL(PA_4, PB_0, PA_10);
Motor dribbleR(PA_1, PA_0, PB_4);

EncoderMotor enc1(PB_2, PB_12, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor enc2(PC_9, PA_11, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor enc3(PC_6 , PC_5, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor encL(PB_13, PC_4, 537.6, EncoderMotor::X4_ENCODING);
EncoderMotor encR(PB_14, PB_15, 537.6, EncoderMotor::X4_ENCODING);

DigitalOut LED_RED_VALUE(PA_13);
DigitalOut LED_GREEN_VALUE(PC_13);
DigitalOut LED_BLUE_VALUE(PH_1);

//ROBOT F
//PID pid1(0.380508455749675, 9.48147505683652, 0, 100, 0.02);
//PID pid2(0.396402258193212, 10.2847763665989, 0, 100, 0.02);
//PID pid3(0.406652530317574, 10.1660800838158, 0, 100, 0.02);
//PID pidL(0.397778686128279, 5.7550366964209, 0, 100, 0.02); //PID pidL(0.597778686128279, 12.7550366964209, 0, 100, 0.02);
//PID pidR(0.30272568701543, 6.0243755539975, 0, 100, 0.02);  //PID pidR(0.63272568701543, 14.5243755539975, 0, 100, 0.02);

//ROBOT G
//PID pid1(0.297430242013194, 9.0211605196814, 0, 100, 0.02);
//PID pid2(0.307343953442149, 9.03308447439312, 0, 100, 0.02);
//PID pid3(0.310277988423672, 8.54558353423818, 0, 100, 0.02);
//PID pidL(0.30374, 7.4118, 0, 100, 0.02); //PID pidL(0.397778686128279, 5.7550366964209, 0, 100, 0.02);
//PID pidR(0.27384, 7.3262, 0, 100, 0.02); //PID pidR(0.30272568701543, 6.0243755539975, 0, 100, 0.02);

//ROBOT H
PID pid1(0.22210959642179, 5.95044130151099, 0, 100, 0.02);
PID pid2(0.208433282254394, 6.80238393068066, 0, 100, 0.02);
PID pid3(0.209120668853316, 6.43233135464926, 0, 100, 0.02);
PID pidL(0.279684770785072, 7.8707254128385, 0, 100, 0.02);
PID pidR(0.275600964975253, 7.51195777371376, 0, 100, 0.02);

/******************************
  Publisher-Subscriber
 ******************************/
dagozilla_msg::NucleoMsg nucleoMsg;
ros::Publisher nucleoPub("nucleo_msg", &nucleoMsg);

void nucleoTargetCallback(const dagozilla_msg::NucleoCmd& cmdMsg);
ros::Subscriber<dagozilla_msg::NucleoCmd>nucleoTargetSub("nucleo_cmd", &nucleoTargetCallback);

// LED Subscriber
void ledCallback(const dagozilla_msg::LedMsg& ledMsg);
ros::Subscriber<dagozilla_msg::LedMsg>nucleoLedSub("led_color", &ledCallback);


/******************************
  Local Variables
 ******************************/
// Target motors speeds in ticks per second.
double wheel1TargetRate = 0;
double wheel2TargetRate = 0;
double wheel3TargetRate = 0;
double dribbleLTargetRate = 0;
double dribbleRTargetRate = 0;

double wheel_1_vel = 0;
double wheel_2_vel = 0;
double wheel_3_vel = 0;
double dribble_L_vel = 0;
double dribble_R_vel = 0;

double rot1 = 0;
double rot2 = 0;
double rot3 = 0;
double rotL = 0;
double rotR = 0;    

double wheel_1_vtarget = 0;
double wheel_2_vtarget = 0;
double wheel_3_vtarget = 0;
double dribble_L_vtarget = 0;
double dribble_R_vtarget = 0;

double kick_power_target = 0;
double meas = 0;
float distance;

// Initial declaration of wheel postion
float curWheel1 = 0;
float curWheel2 = 0;
float curWheel3 = 0;
float curDribbleL = 0;
float curDribbleR = 0;

/*****************
  Functions
*****************/
//void read_encoders();
//void correct_encoder_unit();
//void calculate_actual_velocity();
//void calculate_motor_pwm();
//void set_motor_pwm();
//void publish_data();

/*****************
  Main
******************/
int main() {
  kicker = 1;   //Activate kicker, active LOW
  
  nh.initNode();

  nh.advertise(nucleoPub);
  nh.subscribe(nucleoTargetSub);
  nh.subscribe(nucleoLedSub);

  
  while(1) { 
    //wait_ms should be a minimum of 20ms because of encoder limitations
    wait_ms(sampling_time * 1000); 

    nh.spinOnce();

    // Read encoder
    enc1.disableInterrupts();  // disable interrupts
    enc2.disableInterrupts();  // disable interrupts
    enc3.disableInterrupts();  // disable interrupts
    encL.disableInterrupts();  // disable interrupts
    encR.disableInterrupts();  // disable interrupts
    
    curWheel1 = enc1.getRevolutions(); // in rad
    curWheel2 = enc2.getRevolutions(); // in rad
    curWheel3 = enc3.getRevolutions(); // in rad
    curDribbleL = encL.getRevolutions(); // in rad
    curDribbleR = encR.getRevolutions(); // in rad
    
    enc1.enableInterrupts();  // enable interrupts
    enc2.enableInterrupts();  // enable interrupts
    enc3.enableInterrupts();  // enable interrupts
    encL.enableInterrupts();  // enable interrupts
    encR.enableInterrupts();  // enable interrupts

    // Correct encoder unit
    rot1 = curWheel1 * wheel_radius * 2 * pi; // in m
    rot2 = curWheel2 * wheel_radius * 2 * pi; // in m
    rot3 = curWheel3 * wheel_radius * 2 * pi; // in m
    rotL = curDribbleL * 0.0315 * 2 * pi; // in m. 0.0315m is wheel radius
    rotR = curDribbleR * 0.0315 * 2 * pi; // in m. 0.0315m is wheel radius
    
    // Calculate acutal velocity
    wheel_1_vel = rot1 / sampling_time;
    wheel_2_vel = rot2 / sampling_time;
    wheel_3_vel = rot3 / sampling_time;
    dribble_L_vel = rotL / sampling_time;
    dribble_R_vel = rotR / sampling_time;

    // Calculate motor pwm
    wheel1TargetRate = pid1.createpwm(wheel_1_vtarget, wheel_1_vel);
    wheel2TargetRate = pid2.createpwm(wheel_2_vtarget, wheel_2_vel);
    wheel3TargetRate = pid3.createpwm(wheel_3_vtarget, wheel_3_vel);
    dribbleLTargetRate = pidL.createpwm(dribble_L_vtarget, dribble_L_vel);
    dribbleRTargetRate = pidR.createpwm(dribble_R_vtarget, dribble_R_vel);    
  
    // Set motor pwm
    wheel1.setpwm(wheel1TargetRate);
    wheel2.setpwm(wheel2TargetRate);
    wheel3.setpwm(wheel3TargetRate);
    dribbleL.setpwm(dribbleLTargetRate);
    dribbleR.setpwm(dribbleRTargetRate);
    
    // Kick
    if(kick_power_target > 0) {
      kicker.write(1-kick_power_target);    //Active LOW
      wait_ms(15);
      kicker = 1;
      kick_power_target = 0;
    }
    
    // Detect ball distance at ball handler
    // Need to determine threshold at high-level to make sure that we don't get
    // false positives.
//    for (int i = 0; i < 100; i++){
//      meas += analog_value.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
//    }
//    meas /= 100;
        
    //Hasil regresi nguli!!!!!
//    distance = 196.582708 *meas*meas*meas*meas - 512.1342685 *meas*meas*meas + 505.7639623 *meas*meas - 235.2032652 *meas + 50.49651938;

    distance = analog_value.read();

    // Publish data
    nucleoMsg.wheel_1_pos = rot1; // in m
    nucleoMsg.wheel_2_pos = rot2; // in m
    nucleoMsg.wheel_3_pos = rot3; // in m
    nucleoMsg.dribble_L_pos = rotL; // in m
    nucleoMsg.dribble_R_pos = rotR; // in m

    nucleoMsg.wheel_1_vel = wheel_1_vel; // in m/s
    nucleoMsg.wheel_2_vel = wheel_2_vel; // in m/s
    nucleoMsg.wheel_3_vel = wheel_3_vel; // in m/s
    nucleoMsg.dribble_L_vel = dribble_L_vel; // in m/s
    nucleoMsg.dribble_R_vel = dribble_R_vel; // in m/s
    
    nucleoMsg.ball_distance = distance;
    
    nucleoPub.publish(&nucleoMsg);
    
    nh.spinOnce();  
  }
}

void ledCallback(const dagozilla_msg::LedMsg& ledMsg) {
 LED_RED_VALUE = ledMsg.red;
 LED_GREEN_VALUE = ledMsg.green;
 LED_BLUE_VALUE = ledMsg.blue;       
}

void nucleoTargetCallback(const dagozilla_msg::NucleoCmd& cmdMsg) {
  wheel_1_vtarget = cmdMsg.wheel_1_vtarget;
  wheel_2_vtarget = cmdMsg.wheel_2_vtarget;
  wheel_3_vtarget = cmdMsg.wheel_3_vtarget;
  dribble_L_vtarget = cmdMsg.dribble_L_vtarget;
  dribble_R_vtarget = cmdMsg.dribble_R_vtarget;
  kick_power_target = cmdMsg.kick_power;
}