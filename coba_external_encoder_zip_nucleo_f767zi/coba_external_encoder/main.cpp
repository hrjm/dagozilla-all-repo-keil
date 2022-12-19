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
EncoderDAGOZ locomotionEncR(TIM1);          //Locomotion Right Encoder
EncoderDAGOZ locomotionEncL(TIM2);          //Locomotion Left Encoder
EncoderDAGOZ locomotionEncB(TIM3);          //Locomotion Back Encoder
EncoderDAGOZ dribblerEncR(TIM4);            //Dribbler Right Encoder
EncoderDAGOZ dribblerEncL(TIM8);            //Dribbler Left Encoder

//Motor Pins
MotorDagoz locomotionMotorR(PF_12, PF_8);   //Locomotion Right Motor
MotorDagoz locomotionMotorL(PG_1, PF_9);    //Locomotion Left Motor
MotorDagoz locomotionMotorB(PF_11, PF_7);   //Locomotion Back Motor

MotorDagoz dribblerMotorR(PF_15, PE_6);     //Dribbler Right Motor
MotorDagoz dribblerMotorL(PE_15, PE_5);     //Dribbler Left Motor

//Serial pin
Serial pc(USBTX, USBRX, 9600);        //Serial debug 

//Velocity Data Array
double vDribblerL[1000], vDribblerR[1000];

//Sampling Time
double Ts = 0.005; //ms
double r = 0.05; //m 
double Pi = 3.14159265359;

int main()
{
    
    double rotL = 0.0 ;
    double rotB = 0.0 ;
    double rotR = 0.0 ;
    
    while(1) {
        rotL = rotL + locomotionEncL.GetCounter(1) * 2 * Pi * 0.03125/1024;
        rotB = rotB + locomotionEncB.GetCounter(1) * 2 * Pi * 0.03125/1024;
        rotR = rotR + locomotionEncR.GetCounter(1) * 2 * Pi * 0.03125/1024;
        pc.printf("\nL : %.2lf , B : %.2lf , R : %.2lf", rotL, rotB, rotR);
        wait(Ts);
    }
    
    //for (int i = 0; i < 1000; i++){
//        
//        vDribblerL[i] = dribblerEncL.GetCounter(1) * 2 * Pi * 0.03125 / (537.6 * Ts);
//        vDribblerR[i] = dribblerEncR.GetCounter(1) * 2 * Pi * 0.03125 / (537.6 * Ts);
//        
//        dribblerMotorL.setpwm(1.0);
//        dribblerMotorR.setpwm(1.0);
//        
//        wait_ms(Ts*1000);
//    }
//    
//    dribblerMotorL.setpwm(0.0);
//    dribblerMotorR.setpwm(0.0);
    
    //for (int i = 0; i < 1000; i++){
//        pc.printf("%d,%d\n\r", vDribblerL[i]*100000, vDribblerR[i]*100000);
//    }
        
    //while (true) {
//        //do nothing
//    }
}