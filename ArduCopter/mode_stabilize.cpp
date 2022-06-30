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


    }
    
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


