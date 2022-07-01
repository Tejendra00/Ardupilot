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


    }
    
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

