/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  *
 *      Thrower Nasional KRAI 2018                                              *
 *                                                                              *
 *  Bismillahirahmanirrahim                                                     *
 *                                                                              *
 *  "Hai hamba-hamba-Ku yang malampaui batas terhadap diri mereka sendiri,      *
 *  janganlah kamu berputus asa dari rahmat Allah.                              *
 *  Sesungguhnya Allah mengampuni dosa-dosa semuanya.                           *
 *  Sesungguhnya Dialah Yang Maha Pengampun lagi Maha Penyayang."               *
 *  Surat Az-Zumar Ayat 53                                                      *
 *  Built by : Thrower KRAI 2018                                                *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mbed.h"
#include "Motor.h"
#include "encoderKRAI.h"
#include "Servo.h"
#include "CMPS_KRAI.h"
/* * * * * * * * * * * * * * * * * * *
 *         Deklarasi Konstanta       *
 * * * * * * * * * * * * * * * * * * */

#define PI  3.14159265359           //konstanta PI 
#define RAD_TO_DEG  57.2957795131   //konstanta rad/deg

#define PULSE_TO_JARAK 0.88758751   //kll roda / pulses
#define L 409.0                     //roda to center of robot
#define TS 2.0                      //time sampling

#define MAX_SPEED   21000           //max speed of robot

//Inisiasi RTOS
Thread thread1(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
Thread thread2(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
Thread thread3(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
Thread thread4(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
Thread thread5(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
Thread thread6(osPriorityNormal, DEFAULT_STACK_SIZE, NULL);

/* * * * * * * * * * * * * * * * * * *
 *         Konfigurasi PIN           *
 * * * * * * * * * * * * * * * * * * */

//Deklarasi Rotary Encoder
encoderKRAI enc_belakang(PC_0, PC_1, 538, encoderKRAI::X4_ENCODING);  //chA, chB, revPulse, interruptType
encoderKRAI enc_kiri(PC_11, PC_10, 538, encoderKRAI::X4_ENCODING);    //chA, chB, revPulse, interruptType
encoderKRAI enc_kanan(PC_3, PC_2, 538, encoderKRAI::X4_ENCODING);     //chA, chB, revPulse, interruptType
//Deklarasi Motor
Motor motor_belakang(PB_6, PB_12, PA_7);    //pwm, fwd, rev
Motor motor_kiri(PB_9, PA_6, PA_5);         //pwm, fwd, rev
Motor motor_kanan(PB_8, PA_12, PA_11);      //pwm, fwd, rev
//compas
CMPS_KRAI compass(PC_9, PA_8, 0xC0);
//button
DigitalIn button2(PC_7, PullUp);   //tombol 2
DigitalIn button1(PA_0, PullUp);   //tombol 3
DigitalIn button3(PA_9, PullUp); //tombol 4
//Deklarasi Serial 
Serial pc(USBTX, USBRX, 115200);
//infrared
DigitalIn infrared_ki(PA_13, PullUp);   //pelontar kiri
DigitalIn infrared_ka(PC_12, PullUp);   //pelontar kanan 
DigitalIn infrared_safedepan(PA_4, PullUp); //safe depan (normal keluar)
DigitalIn infrared_safekiri(PC_5, PullUp); //safe kiri belakang (normal keluar)
DigitalIn infrared_tz3(PD_2, PullUp);
//limit sudut depan
DigitalIn lim_ka(PC_13, PullUp);
DigitalIn lim_ki(PB_7, PullUp);
//Pneumatic 
DigitalOut pneu_ki[4] = {(PC_4), (PA_10), (PB_1), (PB_5)};    //tarik, lontar, sudut, trigger
DigitalOut pneu_ka[4] = {(PB_14), (PB_13), (PB_10), (PB_4)};   //tarik, lontar, sudut, trigger
//led
DigitalOut led1 = PC_6;
DigitalOut led_cmps = PC_15;
//servo
Servo holder_ka(PC_8);
Servo holder_ki(PB_0);
/* * * * * * * * * * * * * * * * * * *
 *         Variabel Global           *
 * * * * * * * * * * * * * * * * * * */

//Odometry
float x          = 0, x_prev        = 0;    //komponen x
float y          = 0, y_prev        = 0;    //komponen y
float theta      = 0, theta_prev    = 0;    //komponen head robot
float theta0     = 0;
//kecepatan
float a     = 0, vr  = 0, vw    = 0;    //komponen arah gerak robot
float v1    = 0, v2  = 0, v3    = 0;    //kecepatan motor kanan kiri belakang
float vy;
float v_prev, v;
//variabel mapping
short goal       = 0, cp         = 0;   //check point map 
short count      = 0, count_t    = 0;   //counting PID
//logic
bool led         = false;
bool finish      = false, finish_t = false;    
bool tabrak      = false;
bool calibrate   = false;
bool safe        = false, safe_depan  = false;
bool standbyka   = false, standbyki   = false;
bool stop        = false, fstop       = false;

//variabel PID
float lastin_s   = 0, lastin_t      = 0;    //last input jarak theta
float outSum_s   = 0, outSum_t      = 0;    //integral jarak theta
float outMax_s   = 0, outMin_s      = 0;    //saturasi jarak
float outMax_t   = 0, outMin_t      = 0;    //saturasi theta

//Mapping posisi
typedef struct map {
    int n;
    float x_pos[4];
    float y_pos[4];
    float theta_pos[4];
} mapping;

const mapping map_NULL =    {  0,
                            {  0,  0,  0,  0},
                            {  0,  0,  0,  0},
                            {  0,  0,  0,  0}};
mapping map_data = map_NULL;

/* * * * * * * * * * * * * * * * * * *
 *         Primitive Function        *
 * * * * * * * * * * * * * * * * * * */

void main_Proccess();       //prosedur utama 
void check_Point1();        //prosedur checkpoint map odometry
void compute_Parameter();   //prosedur menghitung jarak odometry

void gerak_Motor();         //prosedur menggerakkan motor
void gerak_Robot();
void pneumatic();           //procedur penggerak pneumatic

void standby();
void standby_all();
void standby_ki();
void standby_ka();

void PID_theta(float t_s);
int PID_Odometry(float x_s, float y_s, float t_s, bool isLast);
float compute_Alpha(float x_s, float y_s, float x, float y,float theta);

/* * * * * * * * * * * * * * * * * * *
 *         Main Program              *
 * * * * * * * * * * * * * * * * * * */

int main(){
    thread1.start(main_Proccess);
    thread2.start(compute_Parameter);
    thread3.start(gerak_Motor);
    thread4.start(pneumatic);
    thread5.start(gerak_Robot);
    thread6.start(standby);

    while(1){
        //do nothing
    }
}

/* * * * * * * * * * * * * * * * * * *
 *     Deklarasi Prosedur            *
 * * * * * * * * * * * * * * * * * * */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*--------MAIN PROCESS--------*/
void main_Proccess(){
//fungsi yang mengatur segala proses robot
float theta_temp = (compass.readBearing()/10.0) - theta0;
Thread::wait(200);
    while(1){
        if(!button2){
            theta0 = (compass.readBearing()/10.0);
            x = 0;                  x_prev = 0;
            y = 0;                  y_prev = 0;
            cp = 1;                 pneu_ka[2] = 0;
            stop = false;           fstop = false;
            ///////////////////////////////////////
        } else if(!button1){
            
        }
        check_Point1();
        /*Proses Theta*/
            theta_temp = (compass.readBearing()/10.0) - theta0;
            if(theta_temp > 180.0 && theta_temp <= 360.0)
                theta = ((compass.readBearing()/10.0) - 360.0 - theta0)/-RAD_TO_DEG;    
            else if(theta_temp < -180.0 && theta_temp >= -360.0)
                theta = ((compass.readBearing()/10.0) + 360.0 - theta0)/-RAD_TO_DEG;
            else
                theta = ((compass.readBearing()/10.0) - theta0)/-RAD_TO_DEG;
            theta_prev = theta;
            if(count_t < map_data.n && fabs(map_data.theta_pos[count_t] - theta*RAD_TO_DEG) > 2 && !finish_t)   
                PID_theta(map_data.theta_pos[count_t]);
            else vw = 0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------ODOMETRY----------*/
void compute_Parameter(){
//Prosedur yang digunakan untuk menghitung parameter X Y dan Theta    
    while(1){
        //Get Value
        float p_ka = enc_kanan.getPulses()*PULSE_TO_JARAK;       //kanan
        float p_ki = enc_kiri.getPulses()*PULSE_TO_JARAK;        //kiri
        float p_bl = enc_belakang.getPulses()*PULSE_TO_JARAK;    //belakang
        
        //Compute value
        x = x_prev + (2*p_bl - p_ki - p_ka)/3*cos(theta_prev) - (-p_ki+p_ka)*0.5773*sin(theta_prev);
        y = y_prev + (2*p_bl - p_ki - p_ka)/3*sin(theta_prev) + (-p_ki+p_ka)*0.5773*cos(theta_prev);
        //update value
        x_prev = x;         y_prev = y; 
        //Reset encoder
        enc_kanan.reset();
        enc_kiri.reset();
        enc_belakang.reset();

        pc.printf("x=%.2f y=%.2f t(rot)=%.2f c=%d ct=%d cp=%d vr=%.2f\n\r", 
                    x, y, theta*RAD_TO_DEG, count, count_t, cp,  vr);
        Thread::wait(TS);
    }
}
float compute_Alpha(float x_s, float y_s, float x, float y,float t){
//fungsi untuk menghitung alpha sebagai arah gerak robot
    float temp = atan((y_s - y)/(x_s - x)) - t;
    if (x_s < x) return temp + PI;
    else            return temp;
}

/*----------------PID ALGORITHM-----------------*/
int PID_Odometry(float x_s, float y_s, float t_s, bool isLast){
    //inisiasi
    float kp_s, ki_s, kd_s;
    float input_s = sqrt(x*x + y*y);
    float target_s = sqrt(x_s*x_s + y_s*y_s);
    //get error
    float error_s = target_s - input_s;
    float dInput_s = input_s - lastin_s;
    //conditioning error
    if(error_s <= 200){
        if(isLast && cp==3){
            kp_s = 700; ki_s = 0; kd_s = 5;
            outMax_s = 0.7; outMin_s = -0.7;
        } else if(isLast && cp==5){
            kp_s = 700; ki_s = 0; kd_s = 5;
            outMax_s = 0.4; outMin_s = -0.4;
        } else if(cp == 5||cp==6){
            kp_s = 1000; ki_s = 0; kd_s = 15;
            outMax_s = 0.6; outMin_s = -0.6;
        } else{
            kp_s = 700; ki_s = 0; kd_s = 5;
            outMax_s = 0.9; outMin_s = -0.9;
        }
    } else if(error_s > 200 && error_s <= 500){
        if(isLast && cp==3){
            kp_s = 1000; ki_s = 0; kd_s = 5;
            outMax_s = 0.6; outMin_s = -0.6;
        } else if(isLast && cp==5){
            kp_s = 700; ki_s = 0; kd_s = 5;
            outMax_s = 0.4; outMin_s = -0.4;
        } else if(cp == 5||cp==6){
            kp_s = 1000; ki_s = 0; kd_s = 15;
            outMax_s = 0.7; outMin_s = -0.7;
        } else{
            kp_s = 200; ki_s = 0; kd_s = 5;
            outMax_s = 0.6; outMin_s = -0.6;
        }
    } else if(error_s >500){
        if(isLast && cp==5){
            kp_s = 700; ki_s = 0; kd_s = 5;
            outMax_s = 0.4; outMin_s = -0.4;
        } else if(cp == 5||cp==6){
            kp_s = 1000; ki_s = 0; kd_s = 15;
            outMax_s = 0.8; outMin_s = -0.8;
        } else if(cp==3){
            kp_s = 1000; ki_s = 0; kd_s = 5;
            outMax_s = 0.95; outMin_s = -0.95;
        } else{
            kp_s = 55; ki_s = 0; kd_s = 10;
            outMax_s = 0.9; outMin_s = -0.9;
        }
    }
    //summing output
    outSum_s += (ki_s*TS*error_s)/MAX_SPEED;
    if(outSum_s > outMax_s) outSum_s = outMax_s;
    else if(outSum_s < outMin_s) outSum_s = outMin_s;
    //compute PID algorithm
    float output_s = kp_s * error_s / MAX_SPEED;
    output_s += outSum_s - ((kd_s* dInput_s)/(MAX_SPEED*TS) );
    //normalisasi
    float temp_v = fabs(output_s);
    a  = compute_Alpha(x_s,y_s,x,y,theta);
    //saturasi
    if(temp_v > outMax_s) temp_v = outMax_s;
    else if(temp_v < outMin_s) temp_v = outMin_s;
    //cobacoba
    float v;
    if(cp == 5){
        if(fabs(temp_v - v_prev)>0.02){
            if(temp_v > v_prev){
                v = v_prev + 0.02;
                if (v > outMax_s) v = outMax_s;
            }
            else if(temp_v < v_prev){
                v = v_prev - 0.08;
                if (v < outMax_s) v = outMax_s;
            }
            v_prev = v;
            vr = v;
        } else{
            vr = temp_v;
        }
    } else {
        vr = temp_v;
    }
    vr = temp_v;
    //update value
    lastin_s = input_s;

    float xe = fabs(x_s - x); 
    float ye = fabs(y_s - y);
    float te = fabs(t_s - theta*RAD_TO_DEG);
    float tol_last, tol_last2, tol_t;

    if(cp==1)               tol_last = 300;
    else if(cp ==6 ||cp==2) tol_last = 150;
    else                    tol_last = 50;
    
    if(cp==5)               tol_last2 = 20;
    else                    tol_last2 = 50;

    if(cp==6||cp==5||cp==4) tol_t = 100;
    else tol_t = 10;

    if(isLast == true){
        if(xe<tol_last && ye<tol_last && te<tol_t){
            return 1;
            if (cp == 4) v_prev = 0;
        }
        else    return 0;
    } else{
        if(xe<300.0 && ye<300.0 && te<10.0){
            return 1;
        }
        else    return 0;
    } 
}

void PID_theta(float t_s){
    float kp_t, ki_t, kd_t;
    float input_t = (theta*RAD_TO_DEG);

    float error_theta = t_s - input_t;
    float dInput_t = input_t - lastin_t;

    if(error_theta <= 6){
        if(finish){
            kp_t = 7.0; ki_t = 0; kd_t = 0;
            outMax_t = 0.2; outMin_t = -0.2;
        } else{
            kp_t = 1.5; ki_t = 0.0; kd_t = 0;
            if(cp == 4||cp == 6||cp==5){
                outMax_t = 0.29; outMin_t = -0.29;
            } else{
                outMax_t = 0.2; outMin_t = -0.2;
            }
        }
    } else if(error_theta >6){
        if(finish){
            kp_t = 4.0; ki_t = 0; kd_t = 0;
            outMax_t = 0.2; outMin_t = -0.2;
        } else{
            kp_t = 2.0; ki_t = 0.0; kd_t = 0;
            if(cp == 4||cp == 6||cp==5){
                outMax_t = 0.2; outMin_t = -0.2;
            } else{
                outMax_t = 0.08; outMin_t = -0.08;
            }
        }
    }
    outSum_t += (ki_t*TS*error_theta)/MAX_SPEED;
    if(outSum_t > outMax_t) outSum_t = outMax_t;
    else if(outSum_t < outMin_t) outSum_t = outMin_t;

    float output_t = kp_t * error_theta/MAX_SPEED;
    output_t += outSum_t - ((kd_t * dInput_t)/(TS*MAX_SPEED));
    vw = (output_t*L);
    if(vw > outMax_t) vw = outMax_t;
    else if(vw < outMin_t) vw = outMin_t;
    lastin_t = input_t;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*---------AKTUATOR---------*/
void gerak_Robot(){
    while(1){
        while(count<map_data.n && !finish){
            count += PID_Odometry(map_data.x_pos[count], 
                                  map_data.y_pos[count],
                                  map_data.theta_pos[count],
                                  (count==(map_data.n-1)));
            if(count == map_data.n){
                count_t = count - 1;    
                finish = true;          stop = true;            
                fstop = true;           if(cp == 5 || cp == 6) tabrak = true;
            } else{
                stop = false;           fstop = false;
                finish = false;         count_t = count;
            }
            Thread::wait(TS);
        }
        //////////////////////////////////////////////////////////
        if(lim_ka && lim_ki && tabrak && finish){
            stop = false;    fstop = false;
            a = PI/2-theta;
            float temp_v;
            if(cp == 5) temp_v = 0.4 - (0.02*vy);
            else        temp_v = 0.5 - (0.02*vy);
            if(temp_v < 0.4) vr = 0.4;
            else vr = temp_v;
        }
        Thread::wait(5);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gerak_Motor(){
// procedure untuk menghitung pwm motor
    float vmax = 0;
    while(1){
        float vod1 = ((vr*(float)-0.5*cos(a))+(vr*(float)0.866*sin(a))) + vw;
        float vod2 = ((vr*(float)-0.5*cos(a))-(vr*(float)0.866*sin(a))) + vw;
        float vod3 = (vr*cos(a)) + vw;

        v1 = vod1;  v2 = vod2;  v3 = vod3;

        if(vmax < fabs(v1)) vmax = fabs(v1);
        if(vmax < fabs(v2)) vmax = fabs(v2);
        if(vmax < fabs(v3)) vmax = fabs(v3);

        if(vmax > 0.95){
            v1 = v1*0.95/vmax;
            v2 = v2*0.95/vmax;
            v3 = v3*0.95/vmax;
        } 

        motor_kanan.speed(v1);
        motor_kiri.speed(v2);
        motor_belakang.speed(v3);
        //////////////////////////////////////////////////////////
        if(cp == 5 && (!lim_ka || !lim_ki) && !finish) a = (-PI/6)-theta;
        else if(cp == 6 && (!lim_ka || !lim_ki) && !finish) a = (210/RAD_TO_DEG)-theta;
        //////////////////////////////////////////////////////////
        if(tabrak && finish){
            if(!lim_ka && !lim_ki){
                fstop = true;               stop = true;                
                tabrak = false;
                if(cp==5 || cp==6){
                    y = -6000;              y_prev=-6000;
                }
            } else if(!lim_ka){
                motor_kanan.speed(0.6);
                motor_kiri.speed(-0.4);
                motor_belakang.speed(-0.4);
                Thread::wait(300);
            } else if(!lim_ki){
                motor_kanan.speed(0.4);
                motor_kiri.speed(-0.6);
                motor_belakang.speed(0.4);
                Thread::wait(300);
            }
        }
        //////////////////////////////////////////////////////////
        if(stop){
            if(fstop){
                motor_kanan.forcebrake();
                motor_kiri.forcebrake();
                motor_belakang.forcebrake();
                if(!lim_ka && !lim_ki)  Thread::wait(500);
                else                    Thread::wait(400);
                fstop = false;
            }
            vr = 0; vw = 0;
            motor_kanan.brake(BRAKE_HIGH);
            motor_kiri.brake(BRAKE_HIGH);
            motor_belakang.brake(BRAKE_HIGH);
            stop = false;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pneumatic(){
/* * * * * * * * * * * * * * * * * * * * * *
 *  Konfigurasi Array Pneumatic
 *  pneu[0] = penarik       1 narik 0 extend
 *  pneu[1] = pelontar      1 extend 0 narik
 *  pneu[2] = sudut         1 extend 0 narik         
 *  pneu[3] = trigger       1 narik 0 extend
 *  holder 
 *  kanan : //holder_ka.position(90); // lepas
 * holder_ka.position(15); // standby
 * holder_ka.position(30); // hold
 * holder_ka.position(55); // landasan terbang
 * 
 *  kiri : hold = holder_ki.position(-30);
 *          pass = holder_ki.position(60);
 *          standby = holder_ki.position(-5);
 *          landasan terbang = holder_ki.position(25);
 * * * * * * * * * * * * * * * * * * * * * */
    standby_all();
    bool check = false;     bool tz3 = false;
    short count_cp5 = 0;
    while(1){
        if((cp==1 && finish)){
            count = 0;      cp += 1;
            Thread::wait(200);       
            stop = false;   fstop = false;
            finish = false;
        } else if(cp == 2 && finish){
            if(tabrak){
                stop = false;           fstop = false;
                finish_t = true;   
            }else if(!tabrak && finish_t){
                finish_t = false;
            }
            if(!infrared_safekiri && !finish_t){
                safe =true;              
                stop = false;                     fstop = false;
                a = (PI)-theta;                   vr = 0.35;   
            }else if(infrared_safekiri && safe){
                safe = false;
                stop = true;      fstop = true;
                x = 2200;         x_prev = 2200;
            }
            ///////////////////////////////////////
            if(!infrared_ki && !safe && !tabrak){
                holder_ki.position(60);
                Thread::wait(500);  pneu_ki[1] = 1;     
                Thread::wait(200);  pneu_ki[2] = 1;     
                led = true;         Thread::wait(500);           
                pneu_ki[3] = 1;     goal = 1;           
                led = false;        standbyki = true;
                holder_ka.position(90);
                Thread::wait(200);
            } else if(!infrared_ka && !safe && !tabrak){
                holder_ka.position(90);
                cp = 3;                 count = 0;
                Thread::wait(300);      
                pneu_ka[1] = 1;         pneu_ka[2] = 1;
                Thread::wait(300);
                holder_ka.position(10);
                holder_ki.position(-20);
                Thread::wait(700);      check = false;
                goal = 0;               finish = false;
                stop = false;           fstop = false;                          
            }  
        } else if(cp == 3 && finish){
            if(goal == 0 && !tabrak && !check){
                holder_ka.position(90);
                led = true;             Thread::wait(1500);
                pneu_ka[3] = 1;         Thread::wait(500);     
                led = false;            goal = 1;
                standbyka = true;       holder_ki.position(60);
                stop = false;           fstop = false;
                tabrak = true;          check = true;
                count_t += 1;
            }   
            if(!infrared_safedepan && !tabrak && check){
                safe = true;
                stop = false;   fstop =false;
                a = PI-theta;       vr = 0.7;
            } else if(infrared_safedepan && safe && check){
                stop = true;   fstop = true;
                x = 2080;       x_prev = 2080;
                y = -5950;      y_prev = -5950;
                count = 0;      cp = 4;
                safe = false;   finish = false;
            }
        } else if(cp==4 && finish){
            if(!infrared_tz3){
                safe = true; tz3 = false;
            } else if(infrared_tz3 && safe){
                safe = false;   tz3 = true;
            }
            if((!infrared_ka || !infrared_ki) && !safe && tz3){
                cp = 5;             count = 0;
                pneu_ka[2] = 1;     pneu_ki[2] = 1;
                pneu_ka[1] = 1;     pneu_ki[1] = 1;
                Thread::wait(900);
                holder_ka.position(15);
                holder_ki.position(-30);
                pneu_ka[2] = 0;     pneu_ki[2] = 0;
                stop = false;       fstop = false;      
                finish = false;     tz3 = false;
            }
        } else if(cp==5 && finish){
            
            if(!tabrak){
                finish_t= true;             Thread::wait(400);
                finish_t = false;           Thread::wait(400);          
                holder_ka.position(90);     led = true;
                Thread::wait(1500);         pneu_ka[3] = 1;
                Thread::wait(100);          led = false;
                stop = false;               fstop = false;
                finish_t= true;
                a = (-PI/6) - theta;        vr= 0.7;             
                
                if(count_cp5 == 0)          Thread::wait(700);
                else if(count_cp5 == 1)     Thread::wait(600);
                else if(count_cp5 == 2)     Thread::wait(500);
                else Thread::wait(500);
                
                tabrak = true;              
                Thread::wait(2000);         
                finish_t= true;             Thread::wait(500);          
                finish_t = false;           Thread::wait(500);
                holder_ki.position(60);     tabrak = false;
                fstop = true;               stop = true;
                led = true; 
                Thread::wait(1500);                         
                pneu_ki[3] = 1;             Thread::wait(300);
                cp =6;                      count = 0;  
                Thread::wait(400);          finish_t = false;
                led1 = 0;                   led = false;
                tabrak = false;             stop = false;               
                fstop = false;              finish = false;
                Thread::wait(200);
                standbyka=true;             standbyki = true;
            }
        } else if(cp==6 && finish){
            if(!infrared_safedepan && !tabrak){
                safe = true;
                stop = false;   fstop = false;
                a = PI-theta;       vr = 0.7;
                holder_ka.position(90);
                holder_ki.position(60);
            }else if(infrared_safedepan && safe){
                stop = true;   fstop = true;
                x = 2080;       x_prev = 2080;
                y = -5950;      y_prev = -5950;
                count = 0;      cp = 4;
                safe = false;   finish = false;
                count_cp5 += 1;
                if(count_cp5 > 2) count_cp5 = 0;
            }
        }  
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void standby(){
    while(1){
        if(standbyki){
            Thread::wait(3000);
            standby_ki();
        }
        if(standbyka){
            Thread::wait(1000);
            standby_ka();
        } 
        if(led){
            led1 = !led1;
            Thread::wait(100);
        } else  led1 = 0;
        if(theta != 0){
            led_cmps = !led_cmps;
            Thread::wait(300);
        } else led_cmps = 0;
    }
}

void standby_all(){
    led = true;
    holder_ki.position(60);
    holder_ka.position(90);
    pneu_ki[0] = 1;     pneu_ki[1] = 0; 
    pneu_ki[2] = 0;     pneu_ki[3] = 1;
    pneu_ka[0] = 1;     pneu_ka[1] = 0; 
    pneu_ka[2] = 1;     pneu_ka[3] = 1;
    Thread::wait(1000);  
    pneu_ki[3] = 0;     pneu_ka[3] = 0;     
    Thread::wait(200);  
    pneu_ki[0] = 0;     pneu_ka[0] = 0;
    holder_ki.position(-5);
    holder_ka.position(15);
    led = false;
}
void standby_ki(){
    //standby pneu kiri
    pneu_ki[0] = 1;     pneu_ki[1] = 0; 
    pneu_ki[2] = 0;     pneu_ki[3] = 1;
    Thread::wait(1000);  pneu_ki[3] = 0;     
    Thread::wait(200);  pneu_ki[0] = 0; 
    standbyki = false;     
}
void standby_ka(){
    //standby pneu kanan
    pneu_ka[0] = 1;     pneu_ka[1] = 0; 
    pneu_ka[2] = 0;     pneu_ka[3] = 1;
    Thread::wait(1000);  pneu_ka[3] = 0;     
    Thread::wait(200);  pneu_ka[0] = 0; 
    standbyka = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------TAMBAHAN-------------*/
void check_Point1(){
//prosedur untuk mengganti map odometry
//Urutan mapping X, Y, THETA
    switch(cp){
        case 1:
            const mapping map_cp1 = {1,
                                    {800},
                                    {-3700},
                                    {0}};
            map_data = map_cp1; break;
        case 2:
            const mapping map_cp2 = {1, 
                                    {2300},
                                    {-4050},
                                    {-20}};
            map_data = map_cp2; break;
        case 3:
            const mapping map_cp3 = {3, 
                                    {1100,   1100, 2450},
                                    {-4300, -5700, -6150},
                                    {-20,    0, -6}};
            map_data = map_cp3; break;
        case 4:
            const mapping map_cp4 = {1, 
                                    {2100},
                                    {-6200},
                                    {-20}};
            map_data = map_cp4; break;
        case 5:
            const mapping map_cp5 = {2, 
                                    {5000, 6460},
                                    {-6400,-6200},
                                    {0, 0}};
            map_data = map_cp5; break;
        case 6:
            const mapping map_cp6 = {1, 
                                    {3350},
                                    {-6400},
                                    {0}};
            map_data = map_cp6; break;
        default:    break;
    }
}