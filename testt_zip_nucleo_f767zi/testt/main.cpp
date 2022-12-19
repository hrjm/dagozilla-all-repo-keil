#include "mbed.h"
#include "math.h"
#include <vector>
//#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
//#include <dgz_msgs/StampedHardwareCommand.h>
//#include <dgz_msgs/StampedHardwareState.h>
//#include <std_srvs/SetBool.h>
//#include "PIDController.h"
//#include "Clock.h"

float clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}
void controlCalculation();

static volatile uint32_t millisValue = 0;
static  Ticker ticker;
 
void millisTicker ()
{
    millisValue ++;
}
 
uint32_t millis ()
{
    return millisValue;
}
 
void setMillis (uint32_t theValue) {
    millisValue = theValue;
}
 
void startMillis () {
    ticker.attach (millisTicker, 0.001);    
}
 
void stopMillis () {
    ticker.detach ();
}

double prev_err = 0.;

double smc(double target, double feedback, double prev_err){
    double w = 0.;
    double wdot = 0;
    double prev_w = 0.;
    double prev_out = 0.;
    
    double deltaT = 0;
    uint32_t millisA = millis();

    double err = target - feedback;
    double der_err = err - prev_err;
    prev_err = err;
    uint32_t elapsedTime= millis() - millisA;
    double derivative = der_err/elapsedTime;
    double out = 0.;
    out = (1/138.5)*(12.81*err + (1.48-1)*derivative);
    if (target == 0) {
            return 0.0;
        }
        
        else {
            return clamp(out, -1., 1.);
        }
    millisA = millis();
}

int a = 1;

int main(){
    while(1){
    controlCalculation();
    
    }
    }



void controlCalculation()
{

//        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        // Get Internal Encoder Pulses
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
//        printf("Pulsenn %d \n", rotInFL);
//        printf("Pulsenn %d \n", rotInFL);
//        printf("Pulsenn %d \n", rotInFL);
//        printf("Pulsenn %d \n", rotInFL);
        
        ////read encoder
//        temp_cur_locomotion_R = locomotionEncR.GetCounter(1);
//        temp_cur_locomotion_L = locomotionEncL.GetCounter(1);
//        temp_cur_locomotion_B = locomotionEncB.GetCounter(1);

//        xSemaphorePulseData.wait();
        motorPulseFL += rotInFL;
        motorPulseFR += rotInFR;
        motorPulseBL += rotInBL;
        motorPulseBR += rotInBR;
        printf("Pulse %d \n", motorPulseFL);
        //int d1 = motorPulseFL;
//        char str[100];
//       // int d2 = trunc(motorPulseFL * 10000);
////        pc.printf(rotnew);
//        sprintf(str, "motorpulse", d1); 
       
//        pc.printf(rotnew);
        
        cur_locomotion_L -= temp_cur_locomotion_L;
        cur_locomotion_R -= temp_cur_locomotion_R;
        cur_locomotion_B -= temp_cur_locomotion_B;

//        xSemaphorePulseData.release();

        // Calculate Feedback Velocity from pulses
        locomotion_FL_vel = rotInFL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_1 * CONTROL_COMPUTE_PERIOD);
        printf("feedback %f \n", locomotion_FL_vel);
        locomotion_FR_vel = rotInFR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_2 * CONTROL_COMPUTE_PERIOD);
        locomotion_BL_vel = rotInBL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_3 * CONTROL_COMPUTE_PERIOD);
        locomotion_BR_vel = rotInBR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_4 * CONTROL_COMPUTE_PERIOD);

        // Compute action drom PIDController to determine PWM
//        locomotion_FR_target_rate = smc(locomotion_FR_target_vel, locomotion_FR_vel);
        locomotion_FL_target_rate = smc(10, locomotion_FL_vel, prev_err);
//        locomotion_BR_target_rate = smc(locomotion_BR_target_vel, locomotion_BR_vel);
//        locomotion_BL_target_rate = smc(locomotion_BL_target_vel, locomotion_BL_vel);

        // Execute PWM value to certain pin
        locomotionMotorFL.setpwm(locomotion_FL_target_rate);
        //locomotionMotorFR.setpwm(locomotion_FR_target_rate);
//        locomotionMotorBL.setpwm(locomotion_BL_target_rate);
//        locomotionMotorBR.setpwm(locomotion_BR_target_rate); 
           
}
