#include "mbed.h"
#include "PID.h"
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>

//primitive function
void moveLocomotion(float vy, float vx, float vw);
void moveLocomotionPI();
void moveDribbler();
void moveDribblerPI(float dribbler_R_vtarget, float dribbler_L_vtarget);
void dataLoggingDribbler();
void switchingPIDMode(int mode);
void mainProcess();
float trf(float deltaAngle, float coefFuzzy);
float makeRatio(float min, float max, float cur);
void CalculatePotFuzzy(float input);
void CalculateDirFuzzy(float input);
void CalculateVelFuzzy(float input);
float CalculateAllFuzzy(float vel, float dir, float pot, int type); //defuzzifikasi
float CalculateOutputRule(int type);
float Fuzzy(float a0, float a1, float a2, float input);
float findMin(float a, float b, float c);
float findMax(float a, float b, float c);
float CalculateOutputThreshold(int type);
float CalculateThresholdFuzzy(float vel, float dir, float type);
float CalculateSharingFuzzy(float potL, float potR, float type);
float CalculateOutputSharing(int type);

/*************************
  Dribbler variables
*************************/
int delayTime = 2;
float SPDribL[25] = {0.68, -0.59, -1.52, -2.1, -2.47, -2.78, -3.07, -3.33, -3.49, -3.42, -2.95, -2.0, -0.68, 0.59, 1.52, 2.1, 2.47, 2.78, 3.07, 3.33, 3.49, 3.42, 2.95, 2.0, 0.68};
float SPDribR[25] = {0.68, 2.0, 2.95, 3.42, 3.49, 3.33, 3.07, 2.78, 2.47, 2.1, 1.52, 0.59, -0.69, -2.0, -2.95, -3.42, -3.49, -3.33, -3.07, -2.78, -2.47, -2.1, -1.52, -0.59, 0.68};

//float SPDribL[25] = {-0.5, -1.0, -1.52, -2.1, -2.47, -2.5, -2.6, -2.65, -2.6, -2.42, -2.35, -2.0, 0.5, 1.0, 1.52, 2.1, 2.47, 2.78, 3.07, 3.33, 3.49, 3.42, 2.95, 2.0, -0.5};
//float SPDribR[25] = {1.0, 2.0, 2.47, 2.6, 2.65, 2.6, 2.5, 2.47, 2.2, 2.1, 1.52, 1.0, 0.5, -2.0, -2.95, -3.42, -3.49, -3.33, -3.07, -2.78, -2.47, -2.1, -1.52, -0.59, 1.0};

/*************************
  PID declaration
*************************/
PID locomotionPidR(KP_R, KI_R, KD_R, N_R, TS, FF_R, PID::PI_MODE);  //Right locomotion PID
PID locomotionPidB(KP_B, KI_B, KD_B, N_B, TS, FF_B, PID::PI_MODE);  //Back locomotion 
PID locomotionPidL(KP_L, KI_L, KD_L, N_L, TS, FF_L, PID::PI_MODE);  //Left locomotion PID
PID dribblerPidR(KP_DR_1, KI_DR_1, KD_DR_1, N_DR_1, TS, FF_DR_1, PID::PI_MODE);  //Right dribbler PID
PID dribblerPidL(KP_DL_1, KI_DL_1, KD_DL_1, N_DL_1, TS, FF_DL_1, PID::PI_MODE);  //Left dribbler PID

/*************************
  Data Logging Variables
**************************/
double vRobot_Data[500], rotSpeed_Data[500], teta_Data[500];
double vl_Data[500], vr_Data[500], potL_Data[500], potR_Data[500];
double vlcur_Data[500], vrcur_Data[500];
double Ts = 0.02; //s //sampling time for velocity tracking
double r = 0.05; //m
double Pi = 3.14159265359;
int counter = 0;
int i = 0; //dataloop
bool record = false;
bool dataLoggingStarted = false;
float locomotion_L_vtarget, locomotion_B_vtarget, locomotion_R_vtarget;

//thread for RTOS
//Thread threadTest;
Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

//Fuzzy variable declaration
float potFuz[5];
float velFuz[3];
float dirFuz[5];
float tempSharingFuz[5];
float motorFuz[5] = {0.0588f, 0.2f, 1.0f, 5.0f, 17.0f};
float thresholdFuz[5] = {1.2f, 1.15f, 1.10f, 1.05f, 1.0f};
float sharingVariableFuz[5] = {0.0f, 0.125f, 0.25f, 0.375f, 0.5f};

//Potentiometer Fuzzy Variable
const float large[3] = {0.0f, 0.0f, 0.4f};
const float mid[3] = {0.35f, 0.575f, 0.8f};
const float small[3] = {0.75f, 0.9f, 1.05f};
const float zero[3] = {1.0f, 1.05f, 1.1f};
const float rev[3] = {1.05f, 1.2f, 1.2f};

//Dir Fuzzy Var    
const float front[3] = {0.0f, 90.0f, 180.0f};
const float left[3] = {90.0f, 180.0f, 270.0f};
const float back[3] = {180.0f, 270.0f, 360.0f};
const float rightA[3] = {0.0f, 0.0f, 90.0f};
const float rightB[3] = {270.0f, 360.0f, 360.0f};

//Vel Fuzzy Var    
const float slow[3] = {0.0f, 0.0f, 1.0f};
const float normal[3] = {0.0f, 1.0f, 2.0f};
const float fast[3] = {1.0f, 2.0f, 2.0f};

//1st Fuzzy Variable
int Fuz1SerL[60]= {1,2,3,4,4,
                   1,2,3,4,4,
                   0,0,1,2,2,
                   0,0,1,2,2,
                   2,2,3,4,4,
                   1,3,3,4,4,
                   1,1,2,2,3,
                   0,1,2,2,2,
                   3,3,4,4,4,
                   2,3,4,4,4,
                   1,2,3,4,4,
                   1,1,2,2,3};
int Fuz1SerR[60]= {1,2,3,4,4,
                   0,0,1,2,2,
                   0,0,1,2,2,
                   1,2,3,4,4,
                   2,2,3,4,4,
                   0,1,2,2,2,
                   1,1,2,2,3,
                   1,3,3,4,4,
                   3,3,4,4,4,
                   1,1,2,2,3,
                   1,2,3,4,4,
                   2,3,4,4,4};
                   
//2nd Fuzzy Variable    
int Fuz2SerL[12]={0,4,0,0,
                 1,3,0,1,
                 1,2,0,1};
int Fuz2SerR[12]={0,0,0,4,
                 1,1,0,3,
                 1,1,0,2};

//3rd Fuzzy Variable    
int Fuz3SerL[25]={4,4,4,4,4,
                 3,3,3,3,3,
                 2,2,2,2,2,
                 1,1,1,1,1,
                 0,0,0,0,0};
int Fuz3SerR[25]={4,3,2,1,0,
                 4,3,2,1,0,
                 4,3,2,1,0,
                 4,3,2,1,0,
                 4,3,2,1,0};
bool loop = false;
float cur_pot_L0;
float cur_pot_R0;
void pressed()
{
  dataLoggingStarted = true;
}

/*****************************
  Main Function
 *****************************/

int main() {
  button.fall(&pressed);
  cur_pot_L0 = -(float)dribblerPotL.read() * 10.0f;
  cur_pot_R0 = -(float)dribblerPotR.read() * 10.0f;

  thread1.start(mainProcess);
  thread2.start(moveDribbler);
    

  while (true) {

  }

}

void mainProcess() {
  wait_ms(1000);

  max_pot_L = 0.01f;
  max_pot_R = 0.01f;

  while (1) {
    cur_pot_L = -(float)dribblerPotL.read() * 10.0f - cur_pot_L0;
    cur_pot_R = -(float)dribblerPotR.read() * 10.0f - cur_pot_R0;

    cur_pot_L -= 0.34f*cur_pot_R;

    cur_pot_L /= (1.3f/1.25f);
    cur_pot_R /= (1.25f/1.25f);    

    //IntEncoder locomotion revolution measurement
    rotInB = intEncR.getRevolutions();
    rotInR = intEncL.getRevolutions();
    rotInL = intEncB.getRevolutions();

    //IntEncoder dribbler revolution measurement
    cur_dribbler_R = -dribblerEncR.GetCounter(1) / 537.6;
    cur_dribbler_L = -dribblerEncL.GetCounter(1) / 537.6;
    dribbler_R_rot = cur_dribbler_L * DRIBBLERWHEEL * 2 * PI; // in m
    dribbler_L_rot = cur_dribbler_R * DRIBBLERWHEEL * 2 * PI; // in m

    // Calculate actual velocity
    locomotion_R_vel = (rotInR * 2 * PI * 0.05) / 0.01;
    locomotion_L_vel = (rotInL * 2 * PI * 0.05) / 0.01;
    locomotion_B_vel = (rotInB * 2 * PI * 0.05) / 0.01;
    dribbler_R_vel = dribbler_R_rot / 0.01;
    dribbler_L_vel = dribbler_L_rot / 0.01;

    moveDribblerPI(vrres, vlres);
    moveLocomotion(0,-1.5,0);
    Thread::wait(5);
    
  }
}

float makeRatio(float min, float max, float cur){
     float ratioResult;
            ratioResult = ((cur-min)/(max-min));
     return ratioResult;  
}

void moveDribbler() {
  
  wait_ms(1000);
  while (1) {
        //Potentiometer measurement
    vy = (locomotion_R_vel * 0.57731) - (locomotion_L_vel * 0.57731);
    vx = (2 * locomotion_B_vel / 3) - (locomotion_R_vel / 3) - (locomotion_L_vel / 3);
    vw = (locomotion_R_vel + locomotion_L_vel + locomotion_B_vel) / (3 * 0.264);

    //dribbler parameter
    xResSpeed = vx - 0.26f * vw * distToWheel; //0.26
    yResSpeed = vy - abs(0.966f * vw * distToWheel); //0.966
    actualMovingAngle = atan2(yResSpeed, xResSpeed);

    if ( actualMovingAngle < 0 ) actualMovingAngle += 2 * PI;

    actualMovingAngle = 15.0f * ((actualMovingAngle / 0.261799f) + 0);

    if (actualMovingAngle >= 360) {
      actualMovingAngle -= 360;
    }

    for (int i = 0; i < 24; i++) {
      if (actualMovingAngle >= (15 * i) && actualMovingAngle < (15 * (i + 1))) {
        regType = i;
      }
    }

    v_magnitude_cur = sqrt(xResSpeed * xResSpeed + yResSpeed * yResSpeed);

    float maxPotL = CalculateThresholdFuzzy(v_magnitude_cur, actualMovingAngle, 0);
    float maxPotR = CalculateThresholdFuzzy(v_magnitude_cur, actualMovingAngle, 1);
    
    //speed ratio pos ctrl
    vl = trf((maxPotL - cur_pot_L), CalculateAllFuzzy(v_magnitude_cur, actualMovingAngle, cur_pot_L, 0)) * speedReferenceFunc; //from 0 to 1 for L  (maxspeed 2.27)
    vr = (-1)*trf((maxPotR - cur_pot_R), CalculateAllFuzzy(v_magnitude_cur, actualMovingAngle, cur_pot_R, 1)) * speedReferenceFunc; //from 0 to -1 for R  (maxspeed 2.27)

   // vl += trf((1.0f - cur_pot_L),0.001f) * speedReferenceFunc; //from 0 to 1 for L  (maxspeed 2.27)
//    vr += -trf((1.0f - cur_pot_R),0.001f) * speedReferenceFunc; //from 0 to -1 for R  (maxspeed 2.27)
    
    float coefL = CalculateSharingFuzzy(cur_pot_L, cur_pot_R, 0);
    float coefR = CalculateSharingFuzzy(cur_pot_L, cur_pot_R, 1);
    
    vlres = vl - coefL * vr;
    vrres = vr - coefR * vl;

    //geometrical analysis, OK for forward and backward movement + rot motion ||| rotspeed kanan minus
    vlres += (v_magnitude_cur / speedReferenceFunc) * (1 / bevelRatio) * (SPDribL[regType] + ((actualMovingAngle - regType * 15) * ((SPDribL[regType + 1] - SPDribL[regType]) / (15))));
    vrres += (v_magnitude_cur / speedReferenceFunc) * (1 / bevelRatio) * (SPDribR[regType] + ((actualMovingAngle - regType * 15) * ((SPDribR[regType + 1] - SPDribR[regType]) / (15))));

    if (dataLoggingStarted || record) {
      dataLoggingDribbler();
    }

    Thread::wait(10);

//      pc.printf("%.3lf %.3lf\n", vlres, vrres);
//      pc.printf("VRobot RotSpeed Teta Vl Vr vlcur vrcur potL potR= %.2lf %.2lf %.1lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n", v_magnitude_cur, vw, actualMovingAngle, vlres, vrres, dribbler_L_vel, dribbler_R_vel, cur_pot_L, cur_pot_R);
    }
}

float trf(float deltaAngle, float coefFuzzy){
    float a;
    if(coefFuzzy == 1.0f){
        a = deltaAngle;    
    }else{
        a = ((pow(coefFuzzy,deltaAngle))-1.0f)*(1/(coefFuzzy-1.0f));
    }
    return a;
}

void dataLoggingDribbler() {
  if (dataLoggingStarted) {
    led = 1;
    vRobot_Data[counter] = v_magnitude_cur;
    rotSpeed_Data[counter] = vw;
    teta_Data[counter] = actualMovingAngle;
    vl_Data[counter] = vlres;
    vr_Data[counter] = vrres;
    vlcur_Data[counter] = dribbler_L_vel;
    vrcur_Data[counter] = dribbler_R_vel;
    potL_Data[counter] = cur_pot_L;
    potR_Data[counter] = cur_pot_R;
    if (counter == 0) {
      infraRed = 1;
    } else {
      infraRed = 0;
    }
    counter++;
    if (counter >= 500) {
      led = 0;
      counter = 0;
      dataLoggingStarted = false;
      record = true;
    }
  }

  if (record) {
    pc.printf("VRobot;RotSpeed;Teta;Vl;Vr;vlcur;vrcur;potL;potR\n");
    for(int j = 0; j<=8; j++){
        for (int i = 0; i < 500; i++) {
            switch(j){
                case 0:
//                    pc.printf("%.3lf\n", vl_Data[i], vr_Data[i], vlcur_Data[i], vrcur_Data[i], potL_Data[i], potR_Data[i]);    
                    pc.printf("%.2lf\n", vRobot_Data[i]);    
                break;
                case 1:
                    pc.printf("%.2lf\n", rotSpeed_Data[i]);
                break;
                case 2:
                    pc.printf("%.1lf\n", teta_Data[i]);
                break;
                case 3:
                    pc.printf("%.3lf\n", vl_Data[i]);
                break;
                case 4:
                    pc.printf("%.3lf\n", vr_Data[i]);
                break;
                case 5:
                    pc.printf("%.3lf\n", vlcur_Data[i]);
                break;
                case 6:
                    pc.printf("%.3lf\n", vrcur_Data[i]);
                break;
                case 7:
                    pc.printf("%.3lf\n", potL_Data[i]);
                break;
                case 8:
                    pc.printf("%.3lf\n", potR_Data[i]);
                break;
                
            }
        }
    }
    record = false;
  }
}

void moveLocomotion(float vy, float vx, float vw) {
  float v0, v1, v2;
  float sRoda = 0.264f;
  v0 = -0.866f*vy + 0.5f*vx+ sRoda*vw;
  v1 = -1*vx+sRoda*vw;
  v2 = 0.866f*vy+0.5f*vx+sRoda*vw;
  
  locomotion_L_vtarget = v0;
  locomotion_B_vtarget = v1;
  locomotion_R_vtarget = v2;
  
  moveLocomotionPI();
}

void moveLocomotionPI(){
    // Calculate motor pwm
    locomotion_R_target_rate = locomotionPidR.createpwm(locomotion_R_vtarget, locomotion_R_vel);
    locomotion_L_target_rate = locomotionPidL.createpwm(locomotion_L_vtarget, locomotion_L_vel);
    locomotion_B_target_rate = locomotionPidB.createpwm(locomotion_B_vtarget, locomotion_B_vel);
    
    //Motor pwm    
    locomotionMotorR.setpwm(locomotion_R_target_rate);
    locomotionMotorL.setpwm(locomotion_L_target_rate);
    locomotionMotorB.setpwm(locomotion_B_target_rate);   
}

void moveDribblerPI(float dribbler_R_vtarget, float dribbler_L_vtarget) {
  dribbler_R_target_rate = dribblerPidR.createpwm(dribbler_R_vtarget, dribbler_R_vel);
  dribbler_L_target_rate = dribblerPidL.createpwm(dribbler_L_vtarget, dribbler_L_vel);
  //motor pwm
  dribblerMotorR.setpwm(dribbler_R_target_rate);
  dribblerMotorL.setpwm(dribbler_L_target_rate);
}

void CalculatePotFuzzy(float input){
    potFuz[0] = Fuzzy(large[0], large[1], large[2], input); 
    potFuz[1] = Fuzzy(mid[0], mid[1], mid[2], input); 
    potFuz[2] = Fuzzy(small[0], small[1], small[2], input); 
    potFuz[3] = Fuzzy(zero[0], zero[1], zero[2], input); 
    potFuz[4] = Fuzzy(rev[0], rev[1], rev[2], input); 
}

void CalculateDirFuzzy(float input){
    float temp;    
    dirFuz[0] = Fuzzy(front[0], front[1], front[2], input); 
    dirFuz[1] = Fuzzy(rightA[0], rightA[1], rightA[2], input); 
    dirFuz[2] = Fuzzy(rightB[0], rightB[1], rightB[2], input); 
    dirFuz[3] = Fuzzy(back[0], back[1], back[2], input); 
    dirFuz[4] = Fuzzy(left[0], left[1], left[2], input); 
    temp = findMax(dirFuz[1], dirFuz[2], 0.0f); 
    dirFuz[1] = temp;
    dirFuz[2] = dirFuz[3];
    dirFuz[3] = dirFuz[4];
}

void CalculateVelFuzzy(float input){
    velFuz[2] = Fuzzy(slow[0], slow[1], slow[2], input); 
    velFuz[1] = Fuzzy(normal[0], normal[1], normal[2], input); 
    velFuz[0] = Fuzzy(fast[0], fast[1], fast[2], input); 
}

float CalculateAllFuzzy(float vel, float dir, float pot, int type){ //defuzzifikasi
    float output;
        CalculateVelFuzzy(vel);
        CalculateDirFuzzy(dir);
        CalculatePotFuzzy(pot);
    //rule output
        output = CalculateOutputRule(type);
    return output;
}

float CalculateThresholdFuzzy(float vel, float dir, float type){
    float output;
        CalculateVelFuzzy(vel);
        CalculateDirFuzzy(dir);
            
        output = CalculateOutputThreshold(type);
    return output;
}

float CalculateSharingFuzzy(float potL, float potR, float type){
    float output;
        CalculatePotFuzzy(potL);
            for(int i=0; i<=4; i++){
                tempSharingFuz[i] = potFuz[i];
            }
        CalculatePotFuzzy(potR);
        
        output = CalculateOutputSharing(type);
    return output;
}


float CalculateOutputSharing(int type){
    float output;
    float rule[5][5];
    float numerator = 0.0;
    float denumerator = 0.0;
        for(int p1=0; p1<=4; p1++){//analisis keadaan kecepatan robot
            for(int p2=0; p2<=4; p2++){//analisis keadaan arah translasi robot
                rule[p1][p2] = findMin(potFuz[p1], tempSharingFuz[p2], 1.0f); //defuzzifikasi
                if(type == 0){
                    numerator += rule[p1][p2] * sharingVariableFuz[Fuz3SerL[p2+5*p1]];
                }else if(type == 1){
                    numerator += rule[p1][p2] * sharingVariableFuz[Fuz3SerR[p2+5*p1]];    
                }
                denumerator += rule[p1][p2];
            }
        }
        
    output = numerator / denumerator;
    
    return output;
}


float CalculateOutputThreshold(int type){
    float output;
    float rule[3][4];
    float numerator = 0.0;
    float denumerator = 0.0;
        for(int v=0; v<=2; v++){//analisis keadaan kecepatan robot
            for(int d=0; d<=3; d++){//analisis keadaan arah translasi robot
                rule[v][d] = findMin(velFuz[v], dirFuz[d], 1.0f); //defuzzifikasi
                if(type == 0){
                    numerator += rule[v][d] * thresholdFuz[Fuz2SerL[d+4*v]];
                }else if(type == 1){
                    numerator += rule[v][d] * thresholdFuz[Fuz2SerR[d+4*v]];    
                }
                denumerator += rule[v][d];
            }
        }
        
    output = numerator / denumerator;
    
    return output;
}

float CalculateOutputRule(int type){//type 0-Left, 1-Right
    float output;
    float rule[3][4][5];
    float numerator = 0.0;
    float denumerator = 0.0;
    for(int v=0; v<=2; v++){//analisis keadaan kecepatan robot
        for(int d=0; d<=3; d++){//analisis keadaan arah translasi robot
            for(int p=0; p<=4; p++){//analisis keadaan potensio robot 
                rule[v][d][p] = findMin(velFuz[v], dirFuz[d], potFuz[p]); //defuzzifikasi
                if(type == 0){
                    numerator += rule[v][d][p] * motorFuz[Fuz1SerL[p+5*d+20*v]];
                }else if(type == 1){
                    numerator += rule[v][d][p] * motorFuz[Fuz1SerR[p+5*d+20*v]];    
                }
                denumerator += rule[v][d][p];
            }
        }   
    } 

    output = numerator / denumerator;
    
    return output;
}


float findMin(float a, float b, float c){
    float out;
    if(a<=b && a<=c){
        out = a;    
    }
    if(b<=a && b<=c){
        out = b;    
    }
    if(c<=a && c<=b){
        out = c;    
    }
    return out;
}

float findMax(float a, float b, float c){
    float out;
    if(a>=b && a>=c){
        out = a;    
    }
    if(b>=a && b>=c){
        out = b;    
    }
    if(c>=a && c>=b){
        out = c;    
    }
    return out;
}

float Fuzzy(float a0, float a1, float a2, float input){
    float result;
    if(a0 == a1){
         if(input <= a0){
            result = 1.0f;
        }else if(input >= a1 && input <= a2){
            result = makeRatio(a2, a1, input);
        }else if(input >= a2){
            result = 0.0f;
        }   
    }else if(a1 == a2){
         if(input <= a0){
            result = 0.0f;
        }else if(input >= a0 && input <= a1){
            result = makeRatio(a0, a1, input);
        }else if(input >= a2){
            result = 1.0f;
        }   
    }else{
        if(input <= a0){
            result = 0.0f;
        }else if(input >= a0 && input <= a1){
            result = makeRatio(a0, a1, input);
        }else if(input >= a1 && input <= a2){
            result = makeRatio(a2, a1, input);
        }else if(input >= a2){
            result = 0.0f;
        }
    }
    return result;        
}
