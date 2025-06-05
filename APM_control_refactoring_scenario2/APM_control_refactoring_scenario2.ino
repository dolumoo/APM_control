//===========================================//
//          REQUIRED LIBRAREIES              // 
//===========================================//
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>       
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
//===========================================//
//          HARDWARE DECLARATION             //
//===========================================//
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2; 
AP_InertialSensor_MPU6000 ins;
//===========================================//
//      USER-SPECIFIED DEFINES GLOBAL        //  
//===========================================//
#define num_RC_channels 8
#define num_Motor 4

#define Motor_1 0 //Num 1 motor
#define Motor_2 1 //Num 2 motor
#define Motor_3 2 //Num 3 motor
#define Motor_4 3 //Num 4 motor

#define roll  0
#define pitch 1
#define thro  2
#define yaw   3

//==========================================//
//     GLOBAL VARIABLES DECLARATION         //
//==========================================//
Vector3f gyro, accel;
float attitude[4];
float gyro_Val[4];

float error[4];

// command
float cmd[4];
float cmd_Mval[4] = {1500, 1500, 1000, 1500}; // minus value
float cmd_Dval[4] = {500, 500, 1000, 500};    // divide value
float cmd_Cmin[4] = {-1.0, -1.0, 0, -1.0};    // constrain value

// motor out
float out[4];
float scaled[4];

// low pass filter
float dt;
uint32_t current_time, prev_time;

uint16_t RC_pwm[num_RC_channels];
uint16_t Motor_pwm[num_Motor];

//RC_pwm initial value
uint16_t RC_pwm_prev[num_RC_channels]= {1500, 1500, 1000, 1500}; 

float print_interval;

//===========================================//
//         USER-SPECIFIED VARIABLES          //
//===========================================//
// control value
float maxVal[4] = {20, 20, 1, 3};
float alpha = 0.2; //Low pass filter gain value
float KP[4] = {0.006, 0.006, 0, 0};
float KD[4] = {0.002, 0.004, 0, 0};

float Kp_roll_angle = 3.5f;
float Kp_roll_rate = 0.0012f;
float Ki_roll_rate = 0.0001f;
float Kp_pitch_angle = 3.5f;
float Kp_pitch_rate = 0.0012f;
float Ki_pitch_rate = 0.0001f;
float Kp_yaw_rate = 0.0025f;
float Kd_yaw_rate = 0.000001f;
float Ki_yaw_rate = 0.003f;

// failsafe value
uint16_t fs[4] = {1500, 1500, 1000, 1500};

// scenario 1
bool is_armed = false;
bool is_shutdowned = false;

//
float time = 0;

namespace print{
 void PWM_RC();
 void PWM_Motor();
 void newline();
 void print_accel(Vector3f accel);
 void print_gyro(Vector3f gyro);
 void print_att(float _roll, float _pitch, float _yaw);
}

//===========================================//
//              VOID SETUP                   //
//===========================================//
void setup(){
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, NULL);
  ins.init_accel(NULL);
  set_dmp();  
  hal.rcout -> set_freq(0xF, 490);
  hal.rcout -> enable_mask(0xFF);
  arm_Motors();
}
//===========================================//
//               VOID LOOP                   //
//===========================================//
void loop(){  
  //calculating dt(delta time)
  prev_time = current_time;      
  current_time = hal.scheduler -> micros();      
  dt = (current_time - prev_time)/1000000.0;
  //end
  
  hal.rcin -> read(RC_pwm, num_RC_channels);
  
  get_IMU_data();
  get_Cmd();
  control_Law();
  control_Mixer();
  motors_Out();
  
  for(int i = 0; i < 4; i++){
    hal.rcout -> write(i, Motor_pwm[i]);
  }
}

//===========================================//
//                NAMESPACE                  //
//===========================================//
namespace print{
  void PWM_RC(){
    hal.console -> printf("RC3_pwm : %.2f\n", float(RC_pwm[2]));
    hal.console -> print("\n"); 
  }
  void PWM_Motor(){  
    for(int i = 0; i < 4; i++){
      hal.console -> printf("Motor%n_pwm : %.2f\n", i, float(Motor_pwm[i]));
    } 
  }
  void newline(){
    hal.console -> print("\n");
  }
  
  void print_accel(Vector3f accel){
    hal.console -> printf("Accel[m/s^2] : ");
    hal.console -> printf("ax : %.2f\t", accel.x);
    hal.console -> printf("ay : %.2f\t", accel.y);
    hal.console -> printf("az : %.2f\t", accel.z);
    hal.console -> print("\n");
  }
  
  void print_gyro(Vector3f gyro){
    hal.console -> printf("Gyro[deg/s] : ");
    hal.console -> printf("p : %.2f\t", gyro.x);
    hal.console -> printf("q : %.2f\t", gyro.y);
    hal.console -> printf("r : %.2f\t", gyro.z);
    hal.console -> print("\n");
  }
  
  void print_att(float _roll, float _pitch, float _yaw){
    hal.console -> printf("Attitude [deg] : ");
    hal.console -> printf("roll : %.2f\t", _roll);
    hal.console -> printf("pitch : %.2f\t", _pitch);
    hal.console -> printf("yaw : %.2f\t", _yaw);
    hal.console -> print("\n");
  }
}

//==========================================//
//               FUNCTIONS                  //
//==========================================//
void scenario(){
  int ROLL = 15;
  int PITCH = 15;
  int YAW = 1;
  
  float roll_cmd, pitch_cmd, yaw_rate_cmd;
 
  if (time < 5)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 5 && time < 10)
  {roll_cmd = - ROLL; pitch_cmd = 0; yaw_rate_cmd = 0;}
  else if (time > 10 && time < 15)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 15 && time < 20)
  {roll_cmd = ROLL;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 20 && time < 25)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 25 && time < 30)
  {roll_cmd = 0;   pitch_cmd = -PITCH;   yaw_rate_cmd = 0;}
  else if (time > 30 && time < 35)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 35 && time < 40)
  {roll_cmd = 0;   pitch_cmd = PITCH;   yaw_rate_cmd = 0;}
  else if (time > 40 && time < 45)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 45 && time < 50)
  {roll_cmd = -ROLL;   pitch_cmd = -PITCH;   yaw_rate_cmd = -YAW;}
  else if (time > 50 && time < 55)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  else if (time > 55 && time < 60)
  {roll_cmd = ROLL;   pitch_cmd = PITCH;   yaw_rate_cmd = YAW;}
  else if (time > 60 && time < 65)
  {roll_cmd = 0;   pitch_cmd = 0;   yaw_rate_cmd = 0;}
  if (time < 65){
   hal.console -> printf("%.3f  \t%.f  \t%.4f  \t%.f  \t%.4f  \t%.f  \t%.4f\n", time, roll_cmd, roll, pitch_cmd, pitch, yaw_rate_cmd, gyro.z);
  }
  time += dt;
  
    
  cmd[roll] = roll_cmd;
  cmd[pitch] = pitch_cmd;
  cmd[yaw] = yaw_rate_cmd;
}

void get_Cmd(){
  /*
  for(int i = 0; i<4; i++){
    RC_pwm[i] = (1-alpha)*RC_pwm_prev[i] + alpha*RC_pwm[i];
    RC_pwm_prev[i] = RC_pwm[i];
  }
  */
  
  fail_Safe();
  
  cmd[thro] = (RC_pwm[2] - 1000.0)/1000.0;
  scenario();
  
  for (int i = 0; i < 4; i++){
    cmd[i] = constrain(cmd[i], cmd_Cmin[i], 1) * maxVal[i];
  }
  
  if (RC_pwm[6] < 1500){
    if(is_shutdowned == false){
      is_armed = true;
    }
    else{
      if(RC_pwm[2] < 1050){
        is_armed = true;
        is_shutdowned = false;
      }
    }
  }
  else{
    is_armed = false;
    is_shutdowned = true;
  }
}

void fail_Safe(){
  if(RC_pwm[thro] > 2200 || RC_pwm[thro] < 950){
    for(int i = 0; i < 4; i++){
      RC_pwm[i] = fs[i];
    }
  }
}

float control_PID(int i){
  error[i] = cmd[i] - attitude[i];
  out[i] = error[i] * KP[i] - gyro_Val[i] * KD[i];
  return out[i];
}

/*
void control_Law(){
  out[thro]  = cmd[thro];
  out[roll]  = control_PID(roll);
  out[pitch] = control_PID(pitch);
  // out[yaw] =
  
  yaw_out = (yaw_rate_cmd-gyro.z)*yaw_rate_KP - gyro.z *yaw_rate_KD; 
  yaw_out = constrain(yaw_rate_cmd, -0.1, 0.1);
}
*/

void control_Law(){
  out[thro] = cmd[thro];
  
  //roll controller
  float roll_error, pitch_error, yaw_error;
  float yaw_error_derive, prev_yaw_error, yaw_int;
  static float roll_rate_d = 0.f;
  static float roll_rate_error = 0.f;
  static float roll_rate_error_i = 0.f;
  
  roll_error = cmd[roll] - attitude[roll];
  roll_rate_d = roll_error * Kp_roll_angle;
  roll_rate_error = roll_rate_d - gyro.x;
  roll_rate_error_i += roll_rate_error*dt;
  
  if(out[thro] < 0.1){
    roll_rate_error_i = 0;
  }
  
  roll_rate_error_i = constrain(roll_rate_error_i, -20, 20);
  out[roll] =  Kp_roll_rate * roll_rate_error + Ki_roll_rate * roll_rate_error_i;
  out[roll] = constrain(out[roll], -0.2, 0.2);
  
  //pitch controller
  static float pitch_rate_d = 0.f;
  static float pitch_rate_error = 0.f;
  static float pitch_rate_error_i = 0.f;
  
  pitch_error = cmd[pitch] - attitude[pitch];
  pitch_rate_d = pitch_error * Kp_pitch_angle;
  pitch_rate_error = pitch_rate_d - gyro.y;
  pitch_rate_error_i += pitch_rate_error*dt;
  
  if(out[thro] < 0.1){
    pitch_rate_error_i = 0;
  }
  
  pitch_rate_error_i = constrain(pitch_rate_error_i, -20, 20);
  out[pitch] =  Kp_pitch_rate * pitch_rate_error + Ki_pitch_rate * pitch_rate_error_i;
  out[pitch] = constrain(out[pitch], -0.2, 0.2);
  
  //yaw rate controller
  yaw_error = 2 * cmd[yaw] - gyro.z;
  yaw_error_derive = (yaw_error - prev_yaw_error)/ dt;
  prev_yaw_error = yaw_error;
  yaw_int += yaw_error * dt;
  
  if(out[thro] < 0.1){
    yaw_int = 0;
  }
  
  yaw_int = constrain(yaw_int, -10, 10);
  out[yaw] = yaw_error * Kp_yaw_rate + yaw_int * Ki_yaw_rate + yaw_error_derive * Kd_yaw_rate; 
  out[yaw] = constrain(out[yaw], -0.1, 0.1);
}


void control_Mixer(){
  for(int i = 0; i < 4; i++){
    int m = i % 2; 
    int n = (i+1) % 2;
    int p = (i == 1 ? -1 : 1);
    int q = (i == 2 ? -1 : 1);
        
    scaled[i] = out[thro] + p * m * out[roll] + q * n * out[pitch] ;//+ out[yaw];
    scaled[i] = constrain(scaled[i], 0.0, 1.0);
  }
  
  /*
  m1_scaled = thro_out + roll_out *  0 + pitch_out *  1 + yaw_out * -1;  0
  m2_scaled = thro_out + roll_out * -1 + pitch_out *  0 + yaw_out *  1;  1
  m3_scaled = thro_out + roll_out *  0 + pitch_out * -1 + yaw_out * -1;  2
  m4_scaled = thro_out + roll_out *  1 + pitch_out *  0 + yaw_out *  1;  3
  */
}

void motors_Out(){
  //  % -> PWM
  for(int i = 0; i < 4; i++){
    Motor_pwm[i] = 1000 + 1000 * scaled[i];
  }
  
  // safety
  if(RC_pwm[2] < 1050){ 
    for(int i = 0; i < 4; i++){
      Motor_pwm[i] = 1000;
    }
  }
  
  engine_Kill();
}

void get_IMU_data(){
  ins.update();
  
  gyro = ins.get_gyro();
  accel = ins.get_accel();
  ins.quaternion.to_euler(&attitude[roll], &attitude[pitch], &attitude[yaw]);
 
  gyro_Val[0] = ToDeg(gyro.x);
  gyro_Val[1] = ToDeg(gyro.y);
  gyro_Val[3]= ToDeg(gyro.z);
  
  attitude[roll] = ToDeg(attitude[roll]);
  attitude[pitch] = ToDeg(attitude[pitch]);
  attitude[yaw] = ToDeg(attitude[yaw]);
}

void set_dmp(){
  hal.console -> println("\nDMP Initialize startup...");
  hal.scheduler -> suspend_timer_procs();
  ins.dmp_init();
  ins.push_gyro_offsets_to_dmp();
  ins.push_accel_offsets_to_dmp();
  ins.dmp_set_bias_from_no_motion();
  hal.scheduler -> resume_timer_procs();
  hal.console -> println("\nComplete!");
}

void engine_Kill(){
  if(is_armed == false){
    Motor_pwm[0] = 1000;
    Motor_pwm[1] = 1000;
    Motor_pwm[2] = 1000;
    Motor_pwm[3] = 1000;
  }
}

void arm_Motors(){
  hal.rcin -> read(RC_pwm, num_RC_channels);
  if (RC_pwm[2] >1100){
    hal.console -> print("Arming denied : Throttle not at minimum.");
    while(true){}

  }
  
  for(int i = 0; i<= 500; i++){
    hal.rcout -> write(Motor_1, 1000);
    hal.rcout -> write(Motor_2, 1000);
    hal.rcout -> write(Motor_3, 1000);
    hal.rcout -> write(Motor_4, 1000);
    hal.scheduler -> delay(2);
  }
}

AP_HAL_MAIN(); 
