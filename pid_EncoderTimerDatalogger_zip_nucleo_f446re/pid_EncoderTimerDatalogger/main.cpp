#include "mbed.h"
#include "CMPS_DAGOZ.h"
#include "EncoderDAGOZ.h"
#include "MotorDAGOZ.h"
#include "EncoderMotor.h"
#include "PID.h"
#include "Constanta.h"

/*****************************
        Pin Declaration
 *****************************/
//Encoder pin
EncoderDAGOZ locomotionEncR(TIM3);          //Locomotion Right Encoder
EncoderDAGOZ locomotionEncL(TIM2);          //Locomotion Left Encoder
EncoderDAGOZ locomotionEncB(TIM1);          //Locomotion Back Encoder
//EncoderDAGOZ dribblerEncR(TIM4);            //Dribbler Right Encoder
//EncoderDAGOZ dribblerEncL(TIM8);            //Dribbler Left Encoder

//Internal Encoder Pin
//EncoderMotor intEncL(PG_8, PE_0, 537.6, EncoderMotor::X4_ENCODING);
//EncoderMotor intEncR(PD_10, PG_14, 537.6, EncoderMotor::X4_ENCODING);
//EncoderMotor intEncB(PF_2, PF_1, 537.6, EncoderMotor::X4_ENCODING);

//EncoderMotor intEncL(PA_11, PB_12, 537.6, EncoderMotor::X4_ENCODING);
//EncoderMotor intEncB(PC_4, PB_13, 537.6, EncoderMotor::X4_ENCODING);
//EncoderMotor intEncR(PD_2, PC_8, 537.6, EncoderMotor::X4_ENCODING);

//Motor Pins
//MotorDagoz locomotionMotorR(PF_12, PF_8);   //Locomotion Right Motor
//MotorDagoz locomotionMotorL(PG_1, PF_9);    //Locomotion Left Motor
//MotorDagoz locomotionMotorB(PF_11, PF_7);   //Locomotion Back Motor
//
//MotorDagoz dribblerMotorR(PF_15, PE_6);     //Dribbler Right Motor
//MotorDagoz dribblerMotorL(PE_15, PE_5);     //Dribbler Left Motor

//Serial pin
Serial pc(USBTX, USBRX, 9600);        //Serial debug 

//Velocity Data Array
double vLocomotionL[1000], vLocomotionR[1000], vLocomotionB[1000];
double vDribblerL[1000], vDribblerR[1000];
double setpoint[1000];

//Sampling Time
double Ts = 0.005; //ms
double r = 0.05; //m 
double Pi = 3.14159265359;

int main()
{
//***DATA LOGGING BLOCK***
//    wait(10);
//    
//    //pc.printf("vDribblerL,vDribblerR\n");
//    pc.printf("Setpoint,vLocomotionL,vLocomotionR,vLocomotionB\n");
//    
//    for (int i = 0; i < 1000; i++){
//        if(i < 301){
//            setpoint[i] = 0;
//            locomotionMotorL.setpwm(0.0);
//            locomotionMotorR.setpwm(0.0);
//            locomotionMotorB.setpwm(0.0);
////            dribblerMotorL.setpwm(0.0);
////            dribblerMotorR.setpwm(0.0);
//        }
//        else{
//            setpoint[i] = 1;
//            locomotionMotorL.setpwm(1.0);
//            locomotionMotorR.setpwm(1.0);
//            locomotionMotorB.setpwm(1.0);
////            dribblerMotorL.setpwm(1.0);
////            dribblerMotorR.setpwm(1.0);
//        }
//        
//        vLocomotionL[i] = intEncL.getRevolutions();
//        vLocomotionR[i] = intEncR.getRevolutions();
//        vLocomotionB[i] = intEncB.getRevolutions();
//        
////        vDribblerL[i] = dribblerEncL.GetCounter(1) * 2 * Pi * 0.03125 / (537.6 * Ts);
////        vDribblerR[i] = dribblerEncR.GetCounter(1) * 2 * Pi * 0.03125 / (537.6 * Ts);
//        
//        wait_ms(Ts*1000);
//    }
//    
////    dribblerMotorL.setpwm(0.0);
////    dribblerMotorR.setpwm(0.0);
//    locomotionMotorL.setpwm(0.0);
//    locomotionMotorR.setpwm(0.0);
//    locomotionMotorB.setpwm(0.0);
//    
//    for (int i = 0; i < 1000; i++){
//        pc.printf("%d,%.0lf,%.0lf,%.0lf\n\r", (int)setpoint[i], vLocomotionL[i]*100000, vLocomotionR[i]*100000, vLocomotionB[i]*100000);
//    }
//        
//    while (true) {
//        //do nothing
//    }
//    
//***END OF BLOCK***

//***ENCODER TESTING BLOCK***
    int pulseR = 0;
    int pulseB = 0;
    int pulseL = 0;
    
    while (true) {
           pulseR = locomotionEncR.GetCounter(0);
           pulseB = locomotionEncB.GetCounter(0);
           pulseL = locomotionEncL.GetCounter(0);

//           pulseR = intEncR.getPulses();
//           pulseB = intEncB.getPulses();
//           pulseL = intEncL.getPulses();     
           pc.printf("%d, %d, %d\n", pulseR, pulseB, pulseL); 
           
           wait(0.05);
    }
//***END OF BLOCK***
}