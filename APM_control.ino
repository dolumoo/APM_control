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
//change this to match the number of transmitter channels you have
#define num_RC_channels 8
#define num_Motor 4

#define Motor_1 0 //Num 1 motor
#define Motor_2 1 //Num 2 motor
#define Motor_3 2 //Num 3 motor
#define Motor_4 3 //Num 4 motor

//==========================================//
//     GLOBAL VARIABLES DECLARATION         //
//==========================================//
Vector3f gyro, accel;
float roll, pitch, yaw;
float roll_error, pitch_error;

float roll_cmd, pitch_cmd, yaw_rate_cmd, thro_cmd;
float roll_out, pitch_out, yaw_out, thro_out;
float m1_scaled, m2_scaled, m3_scaled, m4_scaled;
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
float maxThro = 1;        //
float maxRoll = 20.0;     //[deg]
float maxPitch = 20.0;    //[deg]
float maxYaw_rate = 3.0; //[deg/s]

float alpha = 0.2; //Low pass filter gain value
float roll_KP = 0.009;
float roll_KD = 0.001;
float pitch_KP = 0.009;
float pitch_KD = 0.0001;
float yaw_rate_KP = 0.006;
float yaw_rate_KD = 0.001;

//Set radio channels to default failsafe values in the event
//that bad reciever data is detected. Recommended defaults:

uint16_t RC_roll_fs = 1500;
uint16_t RC_pitch_fs = 1500;
uint16_t RC_throttle_fs = 1000;
uint16_t RC_yaw_fs = 1500;

bool is_armed = false;
bool is_shutdowned = false;

namespace print{
 void PWM_RC();
 void PWM_Motor();
 void newline();
 void print_accel(Vector3f accel);
 void print_gyro(Vector3f gyro);
 void print_att(float roll, float pitch, float yaw);
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

  hal.rcout -> write(Motor_1, Motor_pwm[0]);  
  hal.rcout -> write(Motor_2, Motor_pwm[1]);  
  hal.rcout -> write(Motor_3, Motor_pwm[2]);
  hal.rcout -> write(Motor_4, Motor_pwm[3]);  

  if(print_interval > 1){
    print::newline();
    print::print_accel(accel);
    print::print_gyro(gyro);
    print::print_att(roll, pitch, yaw);
    print_interval = 0; 
  } 
  
  print_interval += dt;
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
    hal.console -> printf("Motor1_pwm : %.2f\t", float(Motor_pwm[0])); 
    hal.console -> printf("Motor2_pwm : %.2f\t", float(Motor_pwm[1]));  
    hal.console -> printf("Motor3_pwm : %.2f\n", float(Motor_pwm[2]));
    hal.console -> printf("Motor4_pwm : %.2f\t", float(Motor_pwm[3])); 
    hal.console -> print("\n"); 
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
  
  void print_att(float roll, float pitch, float yaw){
    hal.console -> printf("Attitude [deg] : ");
    hal.console -> printf("roll : %.2f\t", roll);
    hal.console -> printf("pitch : %.2f\t", pitch);
    hal.console -> printf("yaw_rate : %.2f\t", yaw_rate_cmd);
    hal.console -> printf("yaw : %.2f\t", yaw);
    hal.console -> print("\n");
  }
}

//==========================================//
//               FUNCTIONS                  //
//==========================================//
void get_Cmd(){
  for(int i = 0; i<4; i++){
    RC_pwm[i] = (1-alpha)*RC_pwm_prev[i] + alpha*RC_pwm[i];
    RC_pwm_prev[i] = RC_pwm[i];
  }
  
  fail_Safe();
   
  thro_cmd = (RC_pwm[2] - 1000.0)/1000.0;
  roll_cmd = (RC_pwm[0] - 1500.0)/500.0; 
  pitch_cmd = (RC_pwm[1] - 1500.0)/500.0; 
  yaw_rate_cmd = (RC_pwm[3] - 1500.0)/500.0; 
  
  thro_cmd = constrain(thro_cmd, 0.0, 1.0)*maxThro; 
  roll_cmd = constrain(roll_cmd, -1.0, 1.0)*maxRoll; 
  pitch_cmd = constrain(pitch_cmd, -1.0, 1.0)*maxPitch;
  yaw_rate_cmd = constrain(yaw_rate_cmd, -1.0, 1.0)*maxYaw_rate;
  
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
    //engine killed
    is_armed = false;
    is_shutdowned = true;
  }
}

void fail_Safe(){
  float minVal = 950;
  float maxVal = 2200;
  
  if(RC_pwm[2] > maxVal || RC_pwm[2] < minVal){
    
    RC_pwm[0] = RC_roll_fs;
    RC_pwm[1] = RC_pitch_fs;
    RC_pwm[2] = RC_throttle_fs;
    RC_pwm[3] = RC_yaw_fs;
  }
}

void control_Law(){
  thro_out = thro_cmd;
  
  //roll controller
  roll_error = roll_cmd - roll;
  roll_out = roll_error * roll_KP - gyro.x*roll_KD;
  roll_out = constrain(roll_out, -0.3, 0.3);
  
  pitch_error = pitch_cmd - pitch;
  pitch_out = pitch_error * pitch_KP - gyro.y*roll_KD;
  pitch_out = constrain(pitch_out, -0.3, 0.3);
  
  yaw_out = (yaw_rate_cmd-gyro.z)*yaw_rate_KP - gyro.z *yaw_rate_KD; 
  yaw_out = constrain(yaw_rate_cmd, -0.1, 0.1);
}

void control_Mixer(){
  m1_scaled = thro_out + pitch_out - yaw_out;
  m2_scaled = thro_out - roll_out  + yaw_out;
  m3_scaled = thro_out - pitch_out - yaw_out;
  m4_scaled = thro_out + roll_out  + yaw_out;
  
  m1_scaled = constrain(m1_scaled, 0.0, 1.0);
  m2_scaled = constrain(m2_scaled, 0.0, 1.0); 
  m3_scaled = constrain(m3_scaled, 0.0, 1.0);
  m4_scaled = constrain(m4_scaled, 0.0, 1.0); 
}

void motors_Out(){
  //  % -> PWM
  Motor_pwm[0] = 1000 + 1000 * m1_scaled;
  Motor_pwm[1] = 1000 + 1000 * m2_scaled;
  Motor_pwm[2] = 1000 + 1000 * m3_scaled;
  Motor_pwm[3] = 1000 + 1000 * m4_scaled;
  
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
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
 
  gyro.x = ToDeg(gyro.x);
  gyro.y = ToDeg(gyro.y);
  gyro.z = ToDeg(gyro.z);
  
  roll = ToDeg(roll);
  pitch = ToDeg(pitch);
  yaw = ToDeg(yaw);
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
