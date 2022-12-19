/*
 * Author       : Garudago ITB 2019
 * Developer    : Garudago ITB 2019
 * Maintainer   : Calmantara Sumpono Putra
 * Version      : 2.0.0
 */

#include "mbed.h"
#include "millis.h"
#include <garudago_conf/robotpin.h>
#include <garudago_conf/variable.h>
#include <garudago_conf/config.h>
#include <garudago_hardware_interface/actuator.h>
#include <garudago_hardware_interface/sensor.h>
#include <pid_dagoz/PID.h>

PID locomotion_right_top(right_top_kp, 
                         right_top_ki,
                         right_top_kd, 
                         right_top_N, 
                         right_top_TS, 
                         right_top_FF, 
                         PID::PI_MODE);

PID locomotion_left_top(left_top_kp, 
                        left_top_ki,
                        left_top_kd, 
                        left_top_N, 
                        left_top_TS, 
                        left_top_FF, 
                        PID::PI_MODE);

PID locomotion_right_back(right_back_kp, 
                          right_back_ki,
                          right_back_kd, 
                          right_back_N, 
                          right_back_TS, 
                          right_back_FF, 
                          PID::PI_MODE);

PID locomotion_left_back(left_back_kp, 
                         left_back_ki,
                         left_back_kd, 
                         left_back_N, 
                         left_back_TS, 
                         left_back_FF, 
                         PID::PI_MODE);

Thread thread1(osPriorityNormal);
Thread thread2(osPriorityAboveNormal);

Base base;
Compass compass;

/*debug*/
float degree;
float deg;
////////////////////

void locomotionMovement();
void shagaiMovement(int _state, float _theta, bool _up);
void shagaiParameter();
void pneumaticMovement();
void stickState();
void main_thread();
void compass_thread();

Serial pc(USBTX, USBRX, 115200);

int main(){
    stick.setup();
    compass.compass_reset((float)compass_sensor.getAngle()/10);
    startMillis();
    stick.idle();       //start stick

    //start robot geometry
    robot_geometry[0] = 0.0;
    robot_geometry[1] = 0.0;
    robot_geometry[2] = 0.0;

    thread1.start(main_thread);
    thread2.start(compass_thread);

    while(1){
        // do nothing for idle thread
        if(stick.readable()){
        //procedure to read stick data
            stick.baca_data();
            stick.olah_data();
            pc.printf("Joystick Thread\n");
        }
    }
    // while(1){
    //     if(stick.readable()){
    //     //procedure to read stick data
    //         stick.baca_data();
    //         stick.olah_data();
    //     }
    //     if(millis() - prev_compass_timer > 50){
    //      //before external encoder fixed, use faster sampling rate
    //          compass.compass_update((float)compass_sensor.getAngle()/10.0f);
    //          robot_geometry[2] = compass.compass_value();
    //          prev_compass_timer = millis();
    //          printf("cmps = %.2f",robot_geometry[2]);
    //      }
    //     if(millis() - prev_motor_timer > 10){
    //         stickState();
    //         locomotionMovement();
    //         prev_motor_timer = millis();
    //     }
    //     if(millis() - prev_pneumatic_timer > 15){
    //         shagaiParameter();
    //         pneumaticMovement();
    //         shagaiMovement(state_condition, theta_shagai, up);
    //         prev_pneumatic_timer = millis();
    //     }  
    // }
}


void main_thread(){
    while(1){
        if(millis() - prev_motor_timer > 10){
            stickState();
            locomotionMovement();
            prev_motor_timer = millis();
        }
        if(millis() - prev_pneumatic_timer > 15){
            shagaiParameter();
            pneumaticMovement();
            shagaiMovement(state_condition, theta_shagai, up);
            prev_pneumatic_timer = millis();
        }
        pc.printf("Main Process Thread\n");
        Thread::wait(5);
    }
}


void compass_thread(){
    while(1){
        //before external encoder fixed, use faster sampling rate
        compass.compass_update((float)compass_sensor.getAngle()/10.0f);
        robot_geometry[2] = compass.compass_value();
        // pc.printf("cmps = %.2f\n",robot_geometry[2]);
        pc.printf("Compass Thread\n");
        Thread::wait(50);
    }
}


/*Function and Procedure Declaration*/
void locomotionMovement(){
    left_top_vel    = (enc_left_top.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
    right_top_vel   = (enc_right_top.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
    left_back_vel   = (enc_left_back.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
    right_back_vel  = (enc_right_back.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);

    //compute PID
    float left_top_target_rate = locomotion_left_top.createpwm(wheels_target_velocity[0], 
                                                              left_top_vel);
    float right_top_target_rate = locomotion_right_top.createpwm(wheels_target_velocity[1], 
                                                              right_top_vel);
    float left_back_target_rate = locomotion_left_back.createpwm(wheels_target_velocity[2], 
                                                              left_back_vel);
    float right_back_target_rate = locomotion_right_back.createpwm(wheels_target_velocity[3], 
                                                              right_back_vel);
    
    //conditional
     if (abs(left_top_target_rate)<=0.07f) {
         left_top_target_rate = 0; } 
     if (abs(right_top_target_rate)<=0.07f) {
         right_top_target_rate=0; } 
     if (abs(left_back_target_rate)<=0.07f){
         left_back_target_rate = 0; }
     if (abs(right_back_target_rate)<=0.07f){
         right_back_target_rate = 0; }

    //rate to motor
    motor_left_top.speed(left_top_target_rate);
    motor_right_top.speed(right_top_target_rate);
    motor_left_back.speed(left_back_target_rate);
    motor_right_back.speed(right_back_target_rate);

    enc_left_back.reset();
    enc_right_back.reset();
    enc_left_top.reset();
    enc_right_top.reset();
}


void shagaiMovement(int _state, float _theta, bool _up){
    if ((_state == 5) && (!(_up))){
        if(_theta>-30){
            motor_shagai.speed(-0.5); 
        }else if(_theta>-100.0f){
            motor_shagai.speed(-0.2);
        }else if((_theta<-220.0f) && (_theta>-250.0f)){
            motor_shagai.speed(0.3);
        }
        else{
            motor_shagai.brake(1);
        }
    }else if ((_state == 8)&&(up)){
        if((fabs(_theta))<150){
            motor_shagai.speed(-0.8);
        }else if((fabs(_theta))<210){
            motor_shagai.speed(-0.5);
        }
        else{
            motor_shagai.brake(1);
        }
    }else{
        motor_shagai.brake(1);
    } 
}


void shagaiParameter(){
    float shagai_pulse = enc_shagai.getPulses()*360.0/538;
    theta_shagai += shagai_pulse;
    enc_shagai.reset();
}


void pneumaticMovement(){
    //Kondisi untuk pneumatic pembelok gerege
    if(belok){
        pneumatic_pembelok=1; //awalnya 0
    }else{
        pneumatic_pembelok=0; // awalnya 1
    }

    //state
    if (state_condition<0)
        state_condition=0;
    if (state_condition>15)
        state_condition=5;
    //State 0: Kondisi awal
    if (state_condition == 0){
        up=0;
        theta_shagai=0;
        count_reset_arm=0;

        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 1: kondisi pencapit tertutup
    if (state_condition == 1){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 0;         //Tutup
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 2: tangan gerege turun untuk persiapan menyerahkan gerege ke kuda
    if (state_condition == 2){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 0;         //Turun
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 0;         //Tutup
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 3: penjepit gerege terbuka,gerege akan jatuh ke kuda
    if (state_condition == 3){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 0;         //Turun
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state4: penaik gerege naik ke atas
    if (state_condition==4){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 5: Motor pelontar turun
    if(state_condition==5){
       up=0;
    }
    //state 6: arm terbuka untuk mengambil shagai
    if (state_condition == 6) {
        //Reset encoder motor Shagai
        if(count_reset_arm==0){
            theta_shagai=0;
            count_reset_arm++;
        }
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 0;         //Buka
    }
    //state 7: arm tertutup untuk menjepit shagai
    if (state_condition == 7) {
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    if(state_condition==8){
       up=1;
    }
    //state 9: pengambil shagai terbuka
    if (state_condition==9){
        //Reset encoder motor Shagai
        if(count_reset_arm==1){
            theta_shagai=0;
            count_reset_arm--;
        }
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 0;         //Buka
    }
    //state 10: pengambil shagai tertutup
    if (state_condition==10){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 11 : arm ekstension maju
    if (state_condition == 11) {
        pneumatic_extension = 0;         //Panjang
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 12 : arm terbuka, persiapan menembak
    if (state_condition == 12){
        pneumatic_extension = 0;         //Panjang
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 0;         //Buka
    }
    //state 13 : shagai ditembak ke depan
    if (state_condition == 13) {
        pneumatic_extension = 0;         //Panjang
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 1;         //Panjang
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 0;         //Buka
    }
    //state 14: penjepit shagai tertutup dan pelontarnya memendek
    if (state_condition==14){
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 1;         //Buka
        pneumatic_pengambil = 1;         //Tutup
    }
    //state 15 kembali ke state 5
    if (state_condition==15){
        //motor turun
        up=0;
        state_condition=5;
    }
    
}


void stickState(){
    
    // RESET STATE //
    if (stick.START) {} 
    else if (stick.SELECT){}

    // STATE-STATE PENUMATIC //
    if ((millis()-prev_stick_symbol_timer>700)&&(stick.segitiga)){
        state_condition++;
        prev_stick_symbol_timer = millis();
    } else if ((millis()-prev_stick_symbol_timer>700)&&(stick.kotak)){
        state_condition--;
        prev_stick_symbol_timer = millis();
    } else if((millis()-prev_stick_symbol_timer>700)&&(stick.lingkaran)){
        state_condition=0;
        prev_stick_symbol_timer = millis();
    } else if((millis()-prev_stick_symbol_timer>700)&&(stick.silang)){
        belok = !(belok);
        prev_stick_symbol_timer = millis();
    }


    // STICK ROTATION STATE //
    if (stick.L1 && !stick.R1 && !stick.R2){   
        linear_velocity = 0;
        rotation_velocity = 2.5;
        manual_alpha = 0;
        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
        theta_target = compass.compass_value();
    } 
    else if (stick.R1 && !stick.L1 && !stick.R2){
        linear_velocity = 0;
        rotation_velocity = -2.5;
        manual_alpha = 0;
        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
        theta_target = compass.compass_value();
    } else if ((stick.L1) && (!stick.R1) && (stick.R2)){    
        linear_velocity = 0;
        rotation_velocity = 1.5;
        manual_alpha = 0;
        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
        theta_target = compass.compass_value();
    } 
    else if ((stick.R1) && (!stick.L1) && (stick.R2)){
        linear_velocity = 0;
        rotation_velocity = -1.5;
        manual_alpha = 0;
        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
        theta_target = compass.compass_value();
    } 


    // STICK ARROW STATE //
    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(!stick.R2)  && (!stick.L2)&&(!stick.R1) && (!stick.L1)) {
        //no input condition
        linear_velocity = 0;
        manual_alpha = 0;
        theta_target = compass.compass_value();
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);       
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
        //stick up
        linear_velocity = manual_linear_velocity;
        manual_alpha = PI/2;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
        //stick down
        linear_velocity = manual_linear_velocity;
        manual_alpha = -PI/2;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
        //stick right
        linear_velocity = manual_linear_velocity;
        manual_alpha = 0;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left
        linear_velocity = manual_linear_velocity;
        manual_alpha = PI;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
        //stick right up
        linear_velocity = manual_linear_velocity;
        manual_alpha = PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left up
        linear_velocity = manual_linear_velocity;
        manual_alpha = 3*PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
        //stick right down
        linear_velocity = manual_linear_velocity;
        manual_alpha = -PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left down
        linear_velocity = manual_linear_velocity;
        manual_alpha = -3*PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    }
        // kondisi kalo R2 ditekan
         
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(stick.R2)  && (!stick.L2)&&(!stick.R1) && (!stick.L1)) {
        //no input condition
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = 0;
        manual_alpha = 0;
        theta_target = compass.compass_value();
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);       
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)&&(!stick.L1)&& (!stick.R1)){
        //stick up R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = PI/2;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
        //stick down R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = -PI/2;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
        //stick right R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = 0;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
        //stick left R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = PI;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
        //stick right up R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
        //stick left up R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = 3*PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){ 
        //stick right down R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = -PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    } 
    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
        //stick left down R2
        locomotion_left_top.setTunings(kp,ki,0.0);
        locomotion_right_top.setTunings(kp,ki,0.0);
        locomotion_right_back.setTunings(kp,ki,0.0);
        locomotion_left_back.setTunings(kp2,ki2,0.0);
        linear_velocity = manual_linear_velocityR2;
        manual_alpha = -3*PI/4;
        wheels_target_velocity = base.manual_movement(robot_geometry[2],
                                                    theta_target,
                                                    linear_velocity,
                                                    manual_alpha);
    }
}

///*
// * Author       : Garudago ITB 2019
// * Developer    : Garudago ITB 2019
// * Maintainer   : Calmantara Sumpono Putra
// * Version      : 2.0.0
// */
//
//#include "mbed.h"
//#include "millis.h"
//#include <garudago_conf/robotpin.h>
//#include <garudago_conf/variable.h>
//#include <garudago_conf/config.h>
//#include <garudago_hardware_interface/actuator.h>
//#include <garudago_hardware_interface/sensor.h>
//#include <pid_dagoz/PID.h>
//
//PID locomotion_right_top(right_top_kp, 
//                         right_top_ki,
//                         right_top_kd, 
//                         right_top_N, 
//                         right_top_TS, 
//                         right_top_FF, 
//                         PID::PI_MODE);
//
//PID locomotion_left_top(left_top_kp, 
//                        left_top_ki,
//                        left_top_kd, 
//                        left_top_N, 
//                        left_top_TS, 
//                        left_top_FF, 
//                        PID::PI_MODE);
//
//PID locomotion_right_back(right_back_kp, 
//                          right_back_ki,
//                          right_back_kd, 
//                          right_back_N, 
//                          right_back_TS, 
//                          right_back_FF, 
//                          PID::PI_MODE);
//
//PID locomotion_left_back(left_back_kp, 
//                         left_back_ki,
//                         left_back_kd, 
//                         left_back_N, 
//                         left_back_TS, 
//                         left_back_FF, 
//                         PID::PI_MODE);
//
//Thread thread1(osPriorityNormal);
//Thread thread2(osPriorityAboveNormal);
//
//Base base;
//Compass compass;
//
///*debug*/
//float degree;
//float deg;
//////////////////////
//
//void locomotionMovement();
//void shagaiMovement(int _state, float _theta, bool _up);
//void shagaiParameter();
//void pneumaticMovement();
//void stickState();
//void main_thread();
//void compass_thread();
//
//
//int main(){
//    stick.setup();
//    compass.compass_reset((float)compass_sensor.getAngle()/10);
//    startMillis();
//    stick.idle();       //start stick
//
//    //start robot geometry
//    robot_geometry[0] = 0.0;
//    robot_geometry[1] = 0.0;
//    robot_geometry[2] = 0.0;
//
//    thread1.start(main_thread);
//    thread2.start(compass_thread);
//
//    while(1){
//        // do nothing for idle thread
//        if(stick.readable()){
//        //procedure to read stick data
//            stick.baca_data();
//            stick.olah_data();
//        }
//        // Thread::wait(5);
//    }
//    // while(1){
//    //     if(stick.readable()){
//    //     //procedure to read stick data
//    //         stick.baca_data();
//    //         stick.olah_data();
//    //     }
//    //     if(millis() - prev_compass_timer > 50){
//    //      //before external encoder fixed, use faster sampling rate
//    //          compass.compass_update((float)compass_sensor.getAngle()/10.0f);
//    //          robot_geometry[2] = compass.compass_value();
//    //          prev_compass_timer = millis();
//    //          printf("cmps = %.2f",robot_geometry[2]);
//    //      }
//    //     if(millis() - prev_motor_timer > 10){
//    //         stickState();
//    //         locomotionMovement();
//    //         prev_motor_timer = millis();
//    //     }
//    //     if(millis() - prev_pneumatic_timer > 15){
//    //         shagaiParameter();
//    //         pneumaticMovement();
//    //         shagaiMovement(state_condition, theta_shagai, up);
//    //         prev_pneumatic_timer = millis();
//    //     }  
//    // }
//}
//
//
//void main_thread(){
//    while(1){
//        if(millis() - prev_motor_timer > 10){
//            stickState();
//            locomotionMovement();
//            prev_motor_timer = millis();
//        }
//        if(millis() - prev_pneumatic_timer > 15){
//            shagaiParameter();
//            pneumaticMovement();
//            shagaiMovement(state_condition, theta_shagai, up);
//            prev_pneumatic_timer = millis();
//        }
//        Thread::wait(5);
//    }
//}
//
//
//void compass_thread(){
//    while(1){
//        //before external encoder fixed, use faster sampling rate
//        compass.compass_update((float)compass_sensor.getAngle()/10.0f);
//        robot_geometry[2] = compass.compass_value();
//        // pc.printf("cmps = %.2f\n",robot_geometry[2]);
//        Thread::wait(50);
//    }
//}
//
//
///*Function and Procedure Declaration*/
//void locomotionMovement(){
//    left_top_vel    = (enc_left_top.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
//    right_top_vel   = (enc_right_top.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
//    left_back_vel   = (enc_left_back.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
//    right_back_vel  = (enc_right_back.getPulses() * 2 * PI * WHEEL_RAD)/(0.01*537.6);
//
//    //compute PID
//    float left_top_target_rate = locomotion_left_top.createpwm(wheels_target_velocity[0], 
//                                                              left_top_vel);
//    float right_top_target_rate = locomotion_right_top.createpwm(wheels_target_velocity[1], 
//                                                              right_top_vel);
//    float left_back_target_rate = locomotion_left_back.createpwm(wheels_target_velocity[2], 
//                                                              left_back_vel);
//    float right_back_target_rate = locomotion_right_back.createpwm(wheels_target_velocity[3], 
//                                                              right_back_vel);
//    
//    //conditional
//    // if (abs(left_top_target_rate)<=0.2f) {
//    //     left_top_target_rate = 0; } 
//    // if (abs(right_top_target_rate)<=0.2f) {
//    //     right_top_target_rate=0; } 
//    // if (abs(left_back_target_rate)<=0.2f){
//    //     left_back_target_rate = 0; }
//    // if (abs(right_back_target_rate)<=0.2f){
//    //     right_back_target_rate = 0; }
//
//    //rate to motor
//    motor_left_top.speed(left_top_target_rate);
//    motor_right_top.speed(right_top_target_rate);
//    motor_left_back.speed(left_back_target_rate);
//    motor_right_back.speed(right_back_target_rate);
//
//    enc_left_back.reset();
//    enc_right_back.reset();
//    enc_left_top.reset();
//    enc_right_top.reset();
//}
//
//
//void shagaiMovement(int _state, float _theta, bool _up){
//    if ((_state == 5) && (!(_up))){
//        if(_theta>-30){
//            motor_shagai.speed(-0.5); 
//        }else if(_theta>-100.0f){
//            motor_shagai.speed(-0.2);
//        }else if((_theta<-220.0f) && (_theta>-250.0f)){
//            motor_shagai.speed(0.3);
//        }
//        else{
//            motor_shagai.brake(1);
//        }
//    }else if ((_state == 8)&&(up)){
//        if((fabs(_theta))<150){
//            motor_shagai.speed(-0.8);
//        }else if((fabs(_theta))<210){
//            motor_shagai.speed(-0.5);
//        }
//        else{
//            motor_shagai.brake(1);
//        }
//    }else{
//        motor_shagai.brake(1);
//    } 
//}
//
//
//void shagaiParameter(){
//    float shagai_pulse = enc_shagai.getPulses()*360.0/538;
//    theta_shagai += shagai_pulse;
//    enc_shagai.reset();
//}
//
//
//void pneumaticMovement(){
//    //Kondisi untuk pneumatic pembelok gerege
//    if(belok){
//        pneumatic_pembelok=1; //awalnya 0
//    }else{
//        pneumatic_pembelok=0; // awalnya 1
//    }
//
//    //state
//    if (state_condition<0)
//        state_condition=0;
//    if (state_condition>15)
//        state_condition=5;
//    //State 0: Kondisi awal
//    if (state_condition == 0){
//        up=0;
//        theta_shagai=0;
//        count_reset_arm=0;
//
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 1: kondisi pencapit tertutup
//    if (state_condition == 1){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 0;         //Tutup
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 2: tangan gerege turun untuk persiapan menyerahkan gerege ke kuda
//    if (state_condition == 2){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 0;         //Turun
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 0;         //Tutup
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 3: penjepit gerege terbuka,gerege akan jatuh ke kuda
//    if (state_condition == 3){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 0;         //Turun
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state4: penaik gerege naik ke atas
//    if (state_condition==4){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 5: Motor pelontar turun
//    if(state_condition==5){
//       up=0;
//    }
//    //state 6: arm terbuka untuk mengambil shagai
//    if (state_condition == 6) {
//        //Reset encoder motor Shagai
//        if(count_reset_arm==0){
//            theta_shagai=0;
//            count_reset_arm++;
//        }
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 0;         //Buka
//    }
//    //state 7: arm tertutup untuk menjepit shagai
//    if (state_condition == 7) {
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    if(state_condition==8){
//       up=1;
//    }
//    //state 9: pengambil shagai terbuka
//    if (state_condition==9){
//        //Reset encoder motor Shagai
//        if(count_reset_arm==1){
//            theta_shagai=0;
//            count_reset_arm--;
//        }
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 0;         //Buka
//    }
//    //state 10: pengambil shagai tertutup
//    if (state_condition==10){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 11 : arm ekstension maju
//    if (state_condition == 11) {
//        pneumatic_extension = 0;         //Panjang
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 12 : arm terbuka, persiapan menembak
//    if (state_condition == 12){
//        pneumatic_extension = 0;         //Panjang
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 0;         //Buka
//    }
//    //state 13 : shagai ditembak ke depan
//    if (state_condition == 13) {
//        pneumatic_extension = 0;         //Panjang
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 1;         //Panjang
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 0;         //Buka
//    }
//    //state 14: penjepit shagai tertutup dan pelontarnya memendek
//    if (state_condition==14){
//        pneumatic_extension = 1;         //Pendek
//        pneumatic_penaik    = 1;         //Naik
//        pneumatic_pelontar  = 0;         //Pendek
//        pneumatic_pencapit  = 1;         //Buka
//        pneumatic_pengambil = 1;         //Tutup
//    }
//    //state 15 kembali ke state 5
//    if (state_condition==15){
//        //motor turun
//        up=0;
//        state_condition=5;
//    }
//    
//}
//
//
//void stickState(){
//    
//    // RESET STATE //
//    if (stick.START) {} 
//    else if (stick.SELECT){}
//
//    // STATE-STATE PENUMATIC //
//    if ((millis()-prev_stick_symbol_timer>700)&&(stick.segitiga)){
//        state_condition++;
//        prev_stick_symbol_timer = millis();
//    } else if ((millis()-prev_stick_symbol_timer>700)&&(stick.kotak)){
//        state_condition--;
//        prev_stick_symbol_timer = millis();
//    } else if((millis()-prev_stick_symbol_timer>700)&&(stick.lingkaran)){
//        state_condition=0;
//        prev_stick_symbol_timer = millis();
//    } else if((millis()-prev_stick_symbol_timer>700)&&(stick.silang)){
//        belok = !(belok);
//        prev_stick_symbol_timer = millis();
//    }
//
//
//    // STICK ROTATION STATE //
//    if (stick.L1 && !stick.R1 && !stick.R2){   
//        linear_velocity = 0;
//        rotation_velocity = 2.5;
//        manual_alpha = 0;
//        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
//        theta_target = compass.compass_value();
//    } 
//    else if (stick.R1 && !stick.L1 && !stick.R2){
//        linear_velocity = 0;
//        rotation_velocity = -2.5;
//        manual_alpha = 0;
//        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
//        theta_target = compass.compass_value();
//    } else if ((stick.L1) && (!stick.R1) && (stick.R2)){    
//        linear_velocity = 0;
//        rotation_velocity = 1.5;
//        manual_alpha = 0;
//        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
//        theta_target = compass.compass_value();
//    } 
//    else if ((stick.R1) && (!stick.L1) && (stick.R2)){
//        linear_velocity = 0;
//        rotation_velocity = -1.5;
//        manual_alpha = 0;
//        wheels_target_velocity = base._wheels_from_base(linear_velocity, rotation_velocity, manual_alpha);
//        theta_target = compass.compass_value();
//    } 
//
//
//    // STICK ARROW STATE //
//    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
//            &&(!stick.R2)  && (!stick.L2)&&(!stick.R1) && (!stick.L1)) {
//        //no input condition
//        linear_velocity = 0;
//        manual_alpha = 0;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);       
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//        //stick up
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = PI/2;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//        //stick down
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = -PI/2;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//        //stick right
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = 0;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//        //stick left
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = PI;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//        //stick right up
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//        //stick left up
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = 3*PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
//        //stick right down
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = -PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//        //stick left down
//        linear_velocity = manual_linear_velocity;
//        manual_alpha = -3*PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    }
//        // kondisi kalo R2 ditekan
//         
//    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
//            &&(stick.R2)  && (!stick.L2)&&(!stick.R1) && (!stick.L1)) {
//        //no input condition
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = 0;
//        manual_alpha = 0;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);       
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)&&(!stick.L1)&& (!stick.R1)){
//        //stick up R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = PI/2;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
//        //stick down R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = -PI/2;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
//        //stick right R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = 0;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
//        //stick left R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = PI;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
//        //stick right up R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
//        //stick left up R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = 3*PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){ 
//        //stick right down R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = -PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    } 
//    else if ((!stick.lingkaran)&&(!stick.segitiga)&&(!stick.silang)
//            &&(!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
//        //stick left down R2
//        locomotion_left_top.setTunings(kp,ki,0.0);
//        locomotion_right_top.setTunings(kp,ki,0.0);
//        locomotion_right_back.setTunings(kp,ki,0.0);
//        locomotion_left_back.setTunings(kp2,ki2,0.0);
//        linear_velocity = manual_linear_velocityR2;
//        manual_alpha = -3*PI/4;
//        wheels_target_velocity = base.manual_movement(robot_geometry[2],
//                                                    theta_target,
//                                                    linear_velocity,
//                                                    manual_alpha);
//    }
//}