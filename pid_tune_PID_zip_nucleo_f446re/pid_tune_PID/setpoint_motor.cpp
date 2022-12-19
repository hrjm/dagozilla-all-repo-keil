#include "mbed.h"
#include "motor.h"
#include "PID.h"
#include "encoderKRAI.h"


// Serial
Serial pc(USBTX,USBRX);

//DIG_R, DIG_L, PWM
motor motor1 (PA_9, PB_10, PA_8);
motor motor2 (PA_7, PB_6, PC_7);
motor motor3 (PA_5, PA_6, PC_8);

// CHA, CHB
//Encoder Motor PG36
encoderKRAI enc1(PC_5, PC_6, 537.6  , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
encoderKRAI enc2(PA_11, PC_9, 537.6  , encoderKRAI::X4_ENCODING); // M2 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
encoderKRAI enc3(PB_12 , PB_2, 537.6  , encoderKRAI::X4_ENCODING); // M3 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 

//Penggiring 
//motor peng_l (PB_0, PA_4, PA_10); 
//motor peng_r (PA_1, PA_0, PB_4);

//encoderKRAI enc_peng_l(PB_13, PC_4, 537.6 , encoderKRAI::X4_ENCODING); 
//encoderKRAI enc_peng_r(PB_15, PB_14, 537.6 , encoderKRAI::X4_ENCODING);

//Encoder External
encoderKRAI ext1 (PC_1, PC_0, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
encoderKRAI ext2 (PC_4, PB_13, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
encoderKRAI ext3 (PB_14, PB_15, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction

//ROBOT G
//pid pid1(0.297430242013194, 9.0211605196814, 0, 100, 0.02);
//pid pid2(0.307343953442149, 9.03308447439312, 0, 100, 0.02);
//pid pid3(0.310277988423672, 8.54558353423818, 0, 100, 0.02);
//pid pidL(0.30374, 7.4118, 0, 100, 0.02); //PID pidL(0.397778686128279, 5.7550366964209, 0, 100, 0.02);
//pid pidR(0.27384, 7.3262, 0, 100, 0.02); //PID pidR(0.30272568701543, 6.0243755539975, 0, 100, 0.02);

// Variabel rotasi ( dalam radian ) posisi, Kecepatan dan pid 
float rot1,rot2,rot3, rotL, rotR;
double v[5],vset[5],pidx[5],vold[5];
double xrobot,yrobot,teta,xworld,yworld ;

float PI = 3.14159 ;

//variabel jarak titik tengah robot ke roda
float d = 20.38 ;

double v_l[1000],v_r[1000]; //Jarak linear

double v_1[1000],v_2[1000], v_3[1000] , v_4[1000] , v_5[1000], v_6[1000];

double Ts = 0.002 ; //ms
double r = 0.05 ;//m
double Pi = 3.14159265359 ; 

int i,delay[1000],oldtime  ;

double Setpoint[1000];

Timer t; 

/*****************
  Main
******************/
int main()
{
    wait(20);
    //pc.printf("CLEARDATA");
    //pc.printf("\n");
    
    pc.printf("Setpoint,V_1,V_2,V_3,V_L,V_R,delay (ms)");
    pc.printf("\n");
    
    t.start();
    oldtime = 0 ;
    
               // vset[0] = 0.0;  //Locomotion
//                vset[1] = 0.0;  //Locomotion
//                vset[2] = 0.0;  //Locomotion//
//                vset[3] = 0.0;  //Dribbler
//                vset[4] = 0.0;  //Dribbler
//                
//                motor1.setpwm(0);
//                motor2.setpwm(0);        
//                motor3.setpwm(0);
//                peng_l.setpwm(0);
//                peng_r.setpwm(0);

    for ( int j = 1; j < 11; j++)
    {
        for ( i = 0 ; i < 300; i++ )
        {
            
            
            ////if ( (i >= 0) && (i < 200) )
//            { 
////                vset[0] = 0.0;  //Locomotion
////                vset[1] = 0.0;  //Locomotion
////                vset[2] = 0.0;  //Locomotion//
////                vset[3] = 0.0;  //Dribbler
////                vset[4] = 0.0;  //Dribbler
//                
//                v[0] = enc1.getRevolutions() * 2 * Pi * 0.05 / Ts ; 
//                v[1] = enc2.getRevolutions() * 2 * Pi * 0.05 / Ts ;
//                v[2] = enc3.getRevolutions() * 2 * Pi * 0.05 / Ts ;
//                v[3] = enc_peng_l.getRevolutions() * 2 * Pi * 0.03125 / Ts ;
//                v[4] = enc_peng_r.getRevolutions() * 2 * Pi * 0.03125 / Ts ; 
//                
//                //pidx[0] = pid1.createpwm(vset[0],v[0]) ;
//                //pidx[1] = pid2.createpwm(vset[1],v[1]) ;
//                //pidx[2] = pid3.createpwm(vset[2],v[2]) ;
//    //            pidx[3] = pidL.createpwm(vset[3],v[3]) ;
//    //            pidx[4] = pidR.createpwm(vset[4],v[4]) ;
//                
////                motor1.setpwm(0);
////                motor2.setpwm(0);        
////                motor3.setpwm(0);
////                peng_l.setpwm(0);
////                peng_r.setpwm(0);
//            }
//            
//            if ( (i >= 200) && (i < 1000) )
//            { 
                vset[0] = 1.0;  //Locomotion
                vset[1] = 1.0;  //Locomotion
                vset[2] = 1.0;  //Locomotion
                vset[3] = 1.0;  //encoder external
                vset[4] = 1.0;  //encoder external
                vset[5] = 1.0;  //encoder external
//                vset[3] = 1.0;  //Dribbler
//                vset[4] = 1.0;  //Dribbler
                
                motor1.setpwm((float) j/10.0);
                motor2.setpwm((float) j/10.0);        
                motor3.setpwm((float) j/10.0);
//                peng_l.setpwm((float) j/10.0);
//                peng_r.setpwm((float) j/10.0);
                
                v[0] = enc1.getRevolutions() * 2 * Pi * 0.05 / Ts ; 
                v[1] = enc2.getRevolutions() * 2 * Pi * 0.05 / Ts ;
                v[2] = enc3.getRevolutions() * 2 * Pi * 0.05 / Ts ; 
                v[3] = ext1.getRevolutions() * 2 * Pi * 0.024 / Ts ;
                v[4] = ext2.getRevolutions() * 2 * Pi * 0.024 / Ts ;
                v[5] = ext3.getRevolutions() * 2 * Pi * 0.024 / Ts ; 
//                v[3] = enc_peng_l.getRevolutions() * 2 * Pi * 0.03125 / Ts ;
//                v[4] = enc_peng_r.getRevolutions() * 2 * Pi * 0.03125 / Ts ; 
                
                //pidx[0] = pid1.createpwm(vset[0],v[0]) ;
                //pidx[1] = pid2.createpwm(vset[1],v[1]) ;
                //pidx[2] = pid3.createpwm(vset[2],v[2]) ;
    //            pidx[3] = pidL.createpwm(vset[3],v[3]) ;
    //            pidx[4] = pidR.createpwm(vset[4],v[4]) ;
                
 //           }
            
            /*if ( i >= 700 )
            { 
                vset[0] = 1.5;  //Locomotion
                vset[1] = 1.5;  //Locomotion
                vset[2] = 1.5;  //Locomotion
    //            vset[3] = 1.5;  //Dribbler
    //            vset[4] = 1.5;  //Dribbler
                
                v[0] = enc1.getRevolutions() * 2 * Pi * 0.05 / Ts ; 
                v[1] = enc2.getRevolutions() * 2 * Pi * 0.05 / Ts ;
                v[2] = enc3.getRevolutions() * 2 * Pi * 0.05 / Ts ; 
    //            v[3] = enc_peng_l.getRevolutions() * 2 * Pi * 0.03125 / Ts ;
    //            v[4] = enc_peng_r.getRevolutions() * 2 * Pi * 0.03125 / Ts ; 
                
                pidx[0] = pid1.createpwm(vset[0],v[0]) ;
                pidx[1] = pid2.createpwm(vset[1],v[1]) ;
                pidx[2] = pid3.createpwm(vset[2],v[2]) ;
    //            pidx[3] = pidL.createpwm(vset[3],v[3]) ;
    //            pidx[4] = pidR.createpwm(vset[4],v[4]) ;
                
                motor1.setpwm(pidx[0]);
                motor2.setpwm(pidx[1]);        
                motor3.setpwm(pidx[2]);
    //            peng_l.setpwm(pidx[3]);
    //            peng_r.setpwm(pidx[4]);
            }
            */
            //Output kecepatan dalam m/s
            Setpoint[i] = (float) j/10.0;
            v_1[i] = v[0];
            v_2[i] = v[1];
            v_3[i] = v[2];
            v_4[i] = v[3];
            v_5[i] = v[4];
            v_6[i] = v[5];
//            v_l[i] = v[3];
//            v_r[i] = v[4];
            delay[i] = t.read_ms() - oldtime;
            oldtime = t.read_ms();
            wait_ms(Ts*1000);
            
            //pc.printf("DATA,TIME,%d,%d,%d",int(v_l[i]*100000),int(v_r[i]*100000),delay[i]); 
//            pc.printf("%.2f,%d,%d,%d,%d,%d,%d",Setpoint[i],int(v_1[i]*100000),int(v_2[i]*100000),int(v_3[i]*100000),int(v_l[i]*100000),int(v_r[i]*100000),delay[i]); 
//            pc.printf("\n");
    
        }
        
        
        //for ( i = 0 ; i < 200 ; i++ )
//        {
//            Setpoint[i] = 0.0;
//        }
//        
//        for ( i = 200 ; i < 1000 ; i++ )
//        {
            //Setpoint[i] = (float) j/10.0;
//        }
        /*
        for ( i = 400 ; i < 700 ; i++ )
        {
            Setpoint[i] = 1.0;
        }
        
        for ( i = 700 ; i < 1000 ; i++ )
        {
            Setpoint[i] = 1.5;
        }*/
            
        t.reset();
        
        motor1.setpwm(0);
        motor2.setpwm(0);        
        motor3.setpwm(0);
//        peng_l.setpwm(0);
//        peng_r.setpwm(0);

            for ( i = 0 ; i < 300 ; i++ )
        {
            //Delay dalam ms
            //Jangan lupa bagi s_l dan s_r dibagi 100000 di excel
            //pc.printf("DATA,TIME,%d,%d,%d",int(v_l[i]*100000),int(v_r[i]*100000),delay[i]); 
            pc.printf("%.2f,%d,%d,%d,%d,%d,%d,%d",Setpoint[i],int(v_1[i]*100000),int(v_2[i]*100000),int(v_3[i]*100000),int(v_4[i]*100000),int(v_5[i]*100000),int(v_6[i]*100000),delay[i]); 
            pc.printf("\n");
        }
    
        
    }

    while(1)
    {
        
    }   
}