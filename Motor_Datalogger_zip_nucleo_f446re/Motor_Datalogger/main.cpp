#include "mbed.h"
#include "PID.h"
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>

//primitive function
void moveLocomotion();
void moveDribbler();

/*************************
  Dribbler variables
*************************/
double locomotion_L_Data[1000], locomotion_B_Data[1000], locomotion_R_Data[1000];
double dribbler_L_vel_Data[1000], dribbler_R_vel_Data[1000];
float SPDribL[25] = {0.68, -0.59, -1.52, -2.1, -2.47, -2.78, -3.07, -3.33, -3.49, -3.42, -2.95, -2.0, -0.68, 0.59, 1.52, 2.1, 2.47, 2.78, 3.07, 3.33, 3.49, 3.42, 2.95, 2.0, 0.68};
float SPDribR[25] = {0.68, 2.0, 2.95, 3.42, 3.49, 3.33, 3.07, 2.78, 2.47, 2.1, 1.52, 0.59, -0.69, -2.0, -2.95, -3.42, -3.49, -3.33, -3.07, -2.78, -2.47, -2.1, -1.52, -0.59, 0.68};
float SP = 0.0f;

/*****************************
  Main Function
 *****************************/

void main() {
 // for (int j = 1; j < 21; j++) {
    SP = 1.0;
    for (int i = 0; i < 1000; i++) {
      if (i <= 200) {
        dribbler_L_target_rate = 0;
        dribbler_R_target_rate = 0;
        //                locomotion_L_target_rate = 0;
        //                locomotion_B_target_rate = 0;
        //                locomotion_R_target_rate = 0;
      } else {
        dribbler_L_target_rate = SP;
        dribbler_R_target_rate = -SP;
        //                locomotion_L_target_rate = 1.0;
        //                locomotion_B_target_rate = 1.0;
        //                locomotion_R_target_rate = 1.0;
      }

      moveDribbler();
      //        moveLocomotion();

      //IntEncoder locomotion revolution measurement
      rotInB = intEncR.getRevolutions();
      rotInR = intEncL.getRevolutions();
      rotInL = intEncB.getRevolutions();

      //IntEncoder dribbler revolution measurement
      cur_dribbler_R = -dribblerEncR.GetCounter(1) / 537.6;
      cur_dribbler_L = -dribblerEncL.GetCounter(1) / 537.6;
      dribbler_R_rot = cur_dribbler_L * DRIBBLERWHEEL * 2 * PI; // in m
      dribbler_L_rot = cur_dribbler_R * DRIBBLERWHEEL * 2 * PI; // in m

      // Calculate acutal velocity
      locomotion_R_vel = (rotInR * 2 * PI * 0.05) / 0.01;
      locomotion_L_vel = (rotInL * 2 * PI * 0.05) / 0.01;
      locomotion_B_vel = (rotInB * 2 * PI * 0.05) / 0.01;
      dribbler_R_vel = dribbler_R_rot / 0.01;
      dribbler_L_vel = dribbler_L_rot / 0.01;


      locomotion_L_Data[i] = locomotion_L_vel;
      locomotion_B_Data[i] = locomotion_B_vel;
      locomotion_R_Data[i] = locomotion_R_vel;
      dribbler_L_vel_Data[i] = dribbler_L_vel;
      dribbler_R_vel_Data[i] = dribbler_R_vel;

      wait_ms(TS * 1000);

    }

    dribbler_L_target_rate = 0;
    dribbler_R_target_rate = 0;
       moveDribbler();
    //        locomotion_L_target_rate = 0;
    //        locomotion_B_target_rate = 0;
    //        locomotion_R_target_rate = 0;
    //        moveLocomotion();

    for (int i = 0; i < 500; i++) {
      pc.printf("pwm Vl Vr = %.3f %.3lf %.3lf\n", SP, dribbler_L_vel_Data[i], dribbler_R_vel_Data[i]);
//      pc.printf("pwm Vl Vr = %.3f %.3lf %.3lf\n", SP, dribbler_L_vel_Data[i], dribbler_R_vel_Data[i]);
//                  pc.printf("Vl Vb Vr = %.3lf %.3lf %.3lf\n", locomotion_L_Data[i], locomotion_B_Data[i], locomotion_R_Data[i]);
    }
//  }
  while (1) {

  }

}


void moveLocomotion() {
  locomotionMotorR.setpwm(locomotion_R_target_rate);
  locomotionMotorL.setpwm(locomotion_L_target_rate);
  locomotionMotorB.setpwm(locomotion_B_target_rate);
}

void moveDribbler() {
  dribblerMotorL.setpwm(dribbler_L_target_rate);
  dribblerMotorR.setpwm(dribbler_R_target_rate);
}

