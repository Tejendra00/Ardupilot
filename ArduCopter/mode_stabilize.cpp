#include "Copter.h"
#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Motors/AP_Motors_Class.h>
#include "mycontroller_usercode.h"
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
// #include "AP_Math.h"
int position_hold_flag = 0;

#define ESC_HZ 490
#ifndef PI
  #define PI               3.14159265358979f
#endif

int code_starting_flag = 0;

int yaw_flag_start  = 0;

float current_time  = 0.0;
float IITGN_text_start_time = 0.0;
float IITGN_text_start_time_flag = 0.0;
float H_roll        = 0.0;
float H_pitch       = 0.0;
float H_yaw_rate    = 0.0;
float H_throttle    = 0.0;
float H_yaw         = 0.0;


float H_roll_dot    = 0.0;
float H_pitch_dot   = 0.0;

float H_pitch_prev  = 0.0;
float H_roll_prev   = 0.0;

float imu_roll      = 0.0;
float imu_pitch     = 0.0;
float imu_yaw       = 0.0;

float imu_roll_dot  = 0.0;
float imu_pitch_dot = 0.0;
float imu_yaw_dot   = 0.0;
float battvolt      = 0.0;

float quad_x = 0.0;
float quad_y = 0.0;
float quad_z = 0.0;

float quad_x_ini = 0.0;
float quad_y_ini = 0.0;
float quad_z_ini = 0.0;

float quad_x_dot = 0.0;
float quad_y_dot = 0.0;
float quad_z_dot = 0.0;

float latitude  = 0.0;
float longitude = 0.0;

int pwm__thrust_measurement = 1000;
int flag_thrust_measurement = 0;

float e_phi_prev    = 0.0;
float e_theta_prev  = 0.0;
float e_psi_prev    = 0.0;

float tstart = 0.0;
float time_thrust_mea = 0.0;

float x_des  = 0.0;
float y_des  = 0.0;
float z_des  = 0.0;
float yaw_des_position = 0.0;
float x_des_dot  = 0.0;
float y_des_dot  = 0.0;
float z_des_dot  = 0.0;

float z_des_init = 0.0;
float yaw_initially = 0.0;

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

uint16_t Pf  = 0;
uint16_t Pm1 = 0;
uint16_t Pm2 = 0;
uint16_t Pm3 = 0;
uint16_t Pm4 = 0;

int arm_disarm_flag = 0;

float Force     = 0.0;
float Mb1       = 0.0;
float Mb2       = 0.0;
float Mb3       = 0.0;

float t1_IITGN_traj = 0.0;
float t2_IITGN_traj = 0.0;

float sty_IITGN_traj = 0.0;
float stz_IITGN_traj = 0.0;
float eny_IITGN_traj = 0.0;
float enz_IITGN_traj = 0.0;

float light_on_off = 0.0;

//External Force

float mass = 1.236;
float KIF_x=0.0;  //25
float KIF_y=0.0;  //25
float KIF_z=0.0;  //25
float Fe_initial_x=0.0;
float Fe_initial_y=0.0;
float Fe_initial_z=0.0;
float Ts=0.001;


/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    if (code_starting_flag == 0){

        // To intialize the code
        copter.init_rc_out();
        hal.rcout->set_freq( 15, ESC_HZ); //0xFF  0x0F->b'00001111'
        hal.rcout->enable_ch(0);
        hal.rcout->enable_ch(1);
        hal.rcout->enable_ch(2);
        hal.rcout->enable_ch(3);

        code_starting_flag = 1;

    }else{

    ///////////// Arming checks  /////////////
        if (copter.motors->armed()){
            arm_disarm_flag = 1;
        }else{
            arm_disarm_flag = 0;
        }
///////////// getting pilot inputs  /////////////
        pilot_input();
    ///////////// getting states  /////////////
        quad_states();
    ///////////// For attitude controller controller  /////////////
        attitude_altitude_controller();


    }
    
}

void ModeStabilize::attitude_altitude_controller(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

            PWM1 = 1000;
            PWM2 = 1000;
            PWM3 = 1000;
            PWM4 = 1000;

            // initialize the states
            H_yaw           = 360.0 - (ahrs.yaw_sensor) / 100.0;
            yaw_initially   = 360.0 - (ahrs.yaw_sensor)   / 100.0;     // degrees 

            quad_x_ini =   inertial_nav.get_position_neu_cm().x / 100.0;
            quad_y_ini =  -inertial_nav.get_position_neu_cm().y / 100.0;
            quad_z_ini =   inertial_nav.get_position_neu_cm().z / 100.0;

            // quad_x_ini = (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y);
            // quad_y_ini = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y);
            // quad_z_ini = quad_z;

        }
        else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
            if (copter.motors->armed()){
                custom_PID_controller(H_roll, H_pitch, H_yaw, 0.0 ,0.0, 0.0, z_des ,0.0);

                // IITGN_text_start_time_flag = AP_HAL::millis()/1000.0;
                x_des       = quad_x;
                y_des       = quad_y;
                z_des       = quad_z;
                x_des_dot   = 0.0;
                y_des_dot   = 0.0;
                z_des_dot   = 0.0;
                yaw_des_position       = imu_yaw;
                
            }
        }else if (RC_Channels::get_radio_in(CH_6) > 1600){
            if(copter.motors->armed()){
                    // custom_position_controller(x_des, y_des, z_des, x_des_dot, y_des_dot, z_des_dot, yaw_des_position, 0.0);

            }
        }
}

void ModeStabilize::custom_PID_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.025;   // 0.012 (best) 
    float Kp_theta    = 0.025;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.35;     // 0.2;
    float Kd_theta    = 0.35;    // 0.15; 
    float Kd_psi      = 0.25;     // 0.2

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    float e_phi       = des_phi   - imu_roll;
    float e_theta     = des_theta - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi * saturation_for_roll_pitch_angle_error(e_phi)  \
        + Kd_phi * (des_phi_dot - imu_roll_dot) \
        + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta) \
        + Kd_theta  * (des_theta_dot - imu_pitch_dot) \
        + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
    e_theta_prev = e_theta;

    ////// Yaw controller customized //////
    float e_psi     = des_psi   - imu_yaw;

    if (e_psi > 0.0){
        if ( e_psi > 180.0 ){
            e_psi = -(360.0 - e_psi);
        }
    }else if ( e_psi < 0.0 ) {
        if ( -e_psi < 180.0 ){
            e_psi = e_psi;
        }else{
            e_psi = 360.0 + e_psi;
        }
    }

    float e_psi_sum     = e_psi + e_psi_prev;
    Mb3           = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev          = e_psi;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////
    float e_z   = z_des - quad_z;

    // hal.console->printf("Zd-> %5.2f, z-> %5.2f\n",z_des,quad_z);

    if (e_z > 5.0){
        e_z = 5.0;
    }
    if (e_z < -5.0){
        e_z = -5.0;
    }

    float Kp_z        = 10.0;    // 2.0 (best)
    float Kd_z        = 5.0;    // 1.0 (best)
    Force     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // Force     =  mass * GRAVITY_MSS ;
    // Force     

    if (Force > 20.0){
        Force = 20.0;
    }

    if (Force < 0.0){
        Force =  0.0;
    }

    float function_F1 = Force/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = Force/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = Force/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = Force/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    // PWM1 = 1000;
    // PWM2 = 1000;
    // PWM3 = 1000;
    // PWM4 = 1000;

}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    quad_x =    inertial_nav.get_position_neu_cm().x / 100.0;
    quad_y =   -inertial_nav.get_position_neu_cm().y / 100.0;
    quad_z =    inertial_nav.get_position_neu_cm().z / 100.0;

    quad_x =   quad_x - quad_x_ini;
    quad_y =   quad_y - quad_y_ini;
    quad_z =   quad_z - quad_z_ini;

    // linear velocity in inertial frame of reference
    quad_x_dot =    inertial_nav.get_velocity_neu_cms().x /100.0;
    quad_y_dot =   -inertial_nav.get_velocity_neu_cms().y /100.0;
    quad_z_dot =    inertial_nav.get_velocity_neu_cms().z /100.0;

    // linear velocity in body reference frame
    // quad_x_dot =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y);
    // quad_y_dot = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second
    // float imu_yaw_rad   = imu_yaw * PI /180.0;

    // position in body reference frame
    // quad_x =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y) - quad_x_ini;
    // quad_y = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y) - quad_y_ini;
    // quad_z =  inertial_nav.get_position().z / 100.0 - quad_z_ini;

    // hal.console->printf("roll %5.3f, pitch %5.3f, yaw %5.3f \n",imu_roll, imu_pitch, imu_yaw);

}
void ModeStabilize::custom_position_controller(float x_des_func, float y_des_func, float z_des_func, float x_des_dot_func, float y_des_dot_func, float z_des_dot_func, float des_psi, float des_psi_dot){

    // Translational Position controller 
    hal.console->printf("%5.3f, %5.3f,%5.3f\n",quad_x, quad_y, quad_z);

    float Kp_x = 10.0;
    float Kp_y = 10.0;

    float Kd_x = 5.0;
    float Kd_y = 5.0;

    float e_x  = x_des_func - quad_x ;
    float e_y  = y_des_func - quad_y ;

    float e_x_dot = x_des_dot_func - quad_x_dot;
    float e_y_dot = y_des_dot_func - quad_y_dot;

    float des_theta_x   =   Kp_x * (e_x) + Kd_x * (e_x_dot);
    float des_phi_y     = -(Kp_y * (e_y) + Kd_y * (e_y_dot));

    // des_phi_y           = H_roll;

    float des_phi_dot   = 0.0;
    float des_theta_dot = 0.0;

    float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.025;   // 0.012 (best) 
    float Kp_theta    = 0.025;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.35;     // 0.2;
    float Kd_theta    = 0.35;    // 0.15; 
    float Kd_psi      = 0.25;     // 0.2

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    float e_phi       = des_phi_y   - imu_roll;
    float e_theta     = des_theta_x - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi * saturation_for_roll_pitch_angle_error(e_phi)  \
        + Kd_phi * (des_phi_dot - imu_roll_dot) \
        + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta) \
        + Kd_theta  * (des_theta_dot - imu_pitch_dot) \
        + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
    e_theta_prev = e_theta;

    ////// Yaw controller customized //////
    float e_psi     = des_psi   - imu_yaw;

    if (e_psi > 0.0){
        if ( e_psi > 180.0 ){
            e_psi = -(360.0 - e_psi);
        }
    }else if ( e_psi < 0.0 ) {
        if ( -e_psi < 180.0 ){
            e_psi = e_psi;
        }else{
            e_psi = 360.0 + e_psi;
        }
    }
    float e_psi_sum = e_psi + e_psi_prev;
    Mb3             = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev      = e_psi;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////
    float e_z     = z_des_func     - quad_z;
    float e_z_dot = z_des_dot_func - quad_z_dot;
    // hal.console->printf("Zd-> %5.2f, z-> %5.2f\n",z_des,quad_z);

    if (e_z > 5.0){
        e_z = 5.0;
    }
    if (e_z < -5.0){
        e_z = -5.0;
    }

    float Kp_z        = 10.0;    // 2.0 (best)
    float Kd_z        = 5.0;    // 1.0 (best)
    Force     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (e_z_dot);
    // Force     =  mass * GRAVITY_MSS ;

    if (Force > 20.0){
        Force = 20.0;
    }

    if (Force < 0.0){
        Force =  0.0;
    }

    float function_F1 = Force/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = Force/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = Force/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = Force/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    // PWM1 = 1000;
    // PWM2 = 1000;
    // PWM3 = 1000;
    // PWM4 = 1000;

    // hal.console->printf("%5.3f, %5.3f\n",quad_x, quad_x_dot);

}

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    if (copter.motors->armed()){
        H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
        float dt_yaw = 1.0/100.0;
        H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);
        // hal.console->printf("H Roll %5.3f, H Roll %5.3f, H Roll %5.3f \n",H_roll, H_pitch, H_yaw);
    }

    H_throttle  =  (double)channel_throttle->get_control_in() - 500.0;

    // if (H_throttle < 0){
    //     z_des = 0.0;
    // }

    // if (H_throttle > 0){
    //     z_des = H_throttle/100.0;
    // }

    if (H_throttle > -20 && H_throttle < 20){
        H_throttle = 0.0;
    }

    float dt_z = 1.0/18000.0;
    z_des       =  z_des + H_throttle * dt_z;

    if (z_des > 5.0){
        z_des = 5.0;
    }
    if (z_des < 0.0){
        z_des = 0.0;
    }

    // hal.console->printf("z_des - %f\n",z_des);

}

float ModeStabilize::saturation_for_roll_pitch_angle_error(float error){

    float lim = 25.0;

    if (error > lim){
        error = lim;
    }else if (error < -lim){
        error = -lim;
    }else {
        error = error;
    }
    return error;
}


float ModeStabilize::sat_I_gain_ph_th(float sum){

    float lim = 10.0;

    if (sum > lim){
        sum = lim;
    }else if (sum < -lim){
        sum = -lim;
    }else {
        sum = sum;
    }
    return sum;
}

float ModeStabilize::sat_I_gain_psi(float sum){

    float lim = 20.0;

    if (sum > lim){
        sum = lim;
    }else if (sum < -lim){
        sum = -lim;
    }else {
        sum = sum;
    }
    return sum;
}

int ModeStabilize::Inverse_thrust_function(float Force_){
    int PWM = 1200;

/////////////////////////// From the quadcopter motors  ///////////////////////////

    if (battvolt >= 11.5 ){PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264*Force_)/(15.1632)));
    }else{PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418*Force_)/(9.5740)));}
    if (PWM > 2000){PWM = 2000;}
    if (PWM < 1000){PWM = 1000;}

    return PWM;
}

float ModeStabilize::saturation_for_yaw_angle_error(float error){

    float lim = 30.0;

    if (error > lim){
        error = lim;
    }else if (error < -lim){
        error = -lim;
    }else {
        error = error;
    }
    return error;
}

/////////////////////////// Exteral Force Estimation  ///////////////////////////

// Rotatiion Matrix?? Not included right now

float ModeStabilize::external_force_eqns(float quad_acc, float Force_Thrust, float Fe, float KIF){ 
    float eqf= KIF*(mass*quad_acc- Force_Thrust - Fe);
    return eqf;
}

float ModeStabilize::RKF45(float Fe , float Force_Thrust, float quad_acc, float KIF){ 
    float A_21 = 1/4, A_31 = 3/32, A_32 = 9/32, A_41 = 1932/2197, A_42 = -7200/2197, A_43 = 7296/2197;
    float A_51 = 439/216, A_52 = -8 , A_53 = 3680/513, A_54 = -845/4104;
    float B1   = 25/216, B2   = 0, B3   = 1408/2565, B4   = 2197/4104, B5 = -1/5;

    float KK1     = Ts*external_force_eqns(quad_acc,Force_Thrust,Fe,KIF);

    float XK2     = Fe + A_21*KK1;
    float KK2     = Ts*external_force_eqns(quad_acc,Force_Thrust,XK2,KIF);

    float XK3     = Fe + A_31*KK1 + A_32*KK2;
    float KK3     = Ts*external_force_eqns(quad_acc,Force_Thrust,XK3,KIF);

    float XK4     = Fe + A_41*KK1 + A_42*KK2 + A_43*KK3;
    float KK4     = Ts*external_force_eqns(quad_acc,Force_Thrust,XK4,KIF);

    float XK5     = Fe + A_51*KK1 + A_52*KK2 + A_53*KK3 + A_54*KK4;
    float KK5     = Ts*external_force_eqns(quad_acc,Force_Thrust,XK5,KIF);

    float XK_2    = Fe+ (B1*KK1) + (B2*KK2) + (B3*KK3) + (B4*KK4) + (B5*KK5);

    return XK_2;


}

