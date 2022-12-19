#include "mbed.h"
#include "motor.h"
#include "PID.h"
#include "encoderKRAI.h"


// Serial
Serial pc(USBTX,USBRX);

////DIG_R, DIG_L, PWM
//motor motor1 (PA_9, PB_10, PA_8);
//motor motor2 (PA_7, PB_6, PC_7);
//motor motor3 (PA_5, PA_6, PC_8);

////Motor PG45 F7
//motor motor1 (PB_6, PA_6, PA_7);
//motor motor2 (PB_8, PC_7, PA_5);
//motor motor3 (PB_9, PA_10, PB_10);

//Motor PG45 Kiper
motor motor1 (PA_12, PB_7);
motor motor2 (PA_10, PB_8);
motor motor3 (PH_1 , PB_6);

////Motor CRobot Calman
//encoderKRAI enc1(PA_9, PA_8, 196  , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc2(PA_1, PA_0, 196  , encoderKRAI::X4_ENCODING); // M2 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc3(PB_5 , PB_4, 196  , encoderKRAI::X4_ENCODING); // M3 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 

//Encoder Kiper
encoderKRAI enc1(PA_11, PB_12, 537.6  , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
encoderKRAI enc2(PC_4, PB_13, 537.6  , encoderKRAI::X4_ENCODING); // M3 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
encoderKRAI enc3(PD_2, PC_8, 537.6  , encoderKRAI::X4_ENCODING); // M2 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
 //CHA, CHB
//Encoder Motor PG45 F7
//encoderKRAI enc1(PG_14, PD_10, 537.6  , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc3(PF_1, PF_2, 537.6  , encoderKRAI::X4_ENCODING); // M2 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc2(PE_0 , PG_8, 537.6  , encoderKRAI::X4_ENCODING); // M3 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
//
////Encoder Motor PG36 pake pin yang timer
//encoderKRAI enc1(PA_0, PA_1, 537.6  , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc2(PB_4, PB_5, 537.6  , encoderKRAI::X4_ENCODING); // M2 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 
//encoderKRAI enc3(PB_6 , PB_7, 537.6  , encoderKRAI::X4_ENCODING); // M3 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction  

//Penggiring 
//motor peng_l (PB_0, PA_4, PA_10); 
//motor peng_r (PA_1, PA_0, PB_4);
//
//encoderKRAI enc_peng_l(PC_4, PB_13, 537.6 , encoderKRAI::X4_ENCODING); 
//encoderKRAI enc_peng_r(PB_14, PB_15, 537.6 , encoderKRAI::X4_ENCODING);

//Encoder External
//encoderKRAI ext1 (PC_5, PC_6, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
//encoderKRAI ext2 (PA_11, PC_9, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction
//encoderKRAI ext3 (PB_12, PB_2, 1440 , encoderKRAI::X4_ENCODING); // M1 PPR Total = PPR Encoder * Mode Encoding * Gear Reduction

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

double Ts = 0.005 ; //ms
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
    wait(2);
    //pc.printf("CLEARDATA");
    //pc.printf("\n");
    
    for(int z = 0 ; z <5 ;z++){
    pc.printf("Setpoint,V_1,V_2,V_3,V_L,V_R,delay (ms)");
    pc.printf("\n");
    
    t.start();
    oldtime = 0 ;
    
    //inisialisasi bacaan
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    

    for ( int j = 0; j < 11; j = j+10)
    {
        for ( i = 0 ; i < 300; i++ )
        {
            
            
            
                motor1.setpwm((float) j/10.0);
                motor2.setpwm((float) j/10.0);        
                motor3.setpwm((float) j/10.0);
//                peng_l.setpwm((float) j/10.0);
//                peng_r.setpwm((float) j/10.0);
                
                v[0] = enc1.getRevolutions() * 2 * Pi * 0.024 / Ts ; 
                v[1] = enc2.getRevolutions() * 2 * Pi * 0.024 / Ts ;
                v[2] = enc3.getRevolutions() * 2 * Pi * 0.024 / Ts ; 
//                v[3] = ext1.getRevolutions() * 2 * Pi * 0.024 / Ts ;
//                v[4] = ext2.getRevolutions() * 2 * Pi * 0.024 / Ts ;
//                v[5] = ext3.getRevolutions() * 2 * Pi * 0.024 / Ts ; 
//                v[3] = enc_peng_l.getRevolutions() * 2 * Pi * 0.03125 / Ts ;
//                v[4] = enc_peng_r.getRevolutions() * 2 * Pi * 0.03125 / Ts ; 
                
                
 //           }
            
            
            //Output kecepatan dalam m/s
            Setpoint[i] = (float) j/10.0;
            v_1[i] = v[0];
            v_2[i] = v[1];
            v_3[i] = v[2];
//            v_4[i] = v[3];
//            v_5[i] = v[4];
//            v_6[i] = v[5];
//            v_l[i] = v[3];
//            v_r[i] = v[4];
            delay[i] = t.read_ms() - oldtime;
            oldtime = t.read_ms();
            wait_ms(Ts*1000);
            
        }
        
        
            
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
            pc.printf("%.2f,%d,%d,%d,%d,%d,%d,%d",Setpoint[i],int(v_1[i]*100000),int(v_2[i]*100000),int(v_3[i]*100000),int (v_l[i] *100000), int (v_r[i]*100000),delay[i]); 
            pc.printf("\n");
        }
    
        
    }
    wait(3);
    }

    while(1)
    {
        
    }   
}