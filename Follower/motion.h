#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"
#include "filter.h"
#include "LineSensors.h"

#define SPIN_THRESHOLD 0.05
#define TRACKING_TORLERANCE 8
#define NUM_SENSORS 5
#define SPEED_EST 30

Kinematics_c pose;
Motors_c motors;
PID_c pid_left;
PID_c pid_right;
PID_c pid_distance;
PID_c pid_direction;
PID_c  pid_turn;
Filter_c filter_left;
Filter_c filter_right;
IR_c ir;


class motion_c{
  private:

   unsigned long speed_ts = 0;
   unsigned long last_speed_ts = 0;
   unsigned long pid_speed_control_ts = 0;
   unsigned long last_pid_speed_control_ts = 0;
   long last_count_RIGHT;
   long last_count_LEFT;
   float left_pid_pwm;
   float right_pid_pwm;
   float MAX_SPEED = 4.f;
   bool stopping_init;
   unsigned long stopping_ts;
   unsigned long pose_est_ts;
   unsigned long last_pose_est_ts;
   unsigned long moving_ts;
   float raw_diff;
   float normalised_diff;
   bool moving_init;
   bool turn_finished;
   bool end_finished;
   float tracking_x_diff;
   float tracking_y_diff;
   float angle_diff;
   bool is_first_entry_calibration;
   float adjust_speed;
   float direction_speed;
   float turn_speed;
     
  public:
    float left_speed;
    float right_speed;
    float readings[NUM_SENSORS];

    // arrays for IR-sensor calibration
    unsigned long irsMAX[NUM_SENSORS];
    unsigned long irsMIN[NUM_SENSORS];
    unsigned long irsRANGE[NUM_SENSORS];
    
    motion_c(){
    }


    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      
      speed_ts = millis();
      last_speed_ts = millis();
      filter_left.initialise();
      filter_right.initialise();
      pid_speed_control_ts = millis();
      last_pid_speed_control_ts = millis();
      pose_est_ts = millis();
      last_pose_est_ts = millis();
      setupEncoder0();
      motors.initialise();
      setupEncoder1();
      pose.initialise(start_x, start_y, start_th);
      stopping_ts = millis();
      left_pid_pwm = 0.0;
      right_pid_pwm = 0.0;
      raw_diff = 0.0;
      normalised_diff = 0.0;
      turn_finished = false;
      moving_init = true;
      stopping_init = true;
      end_finished = false;
      moving_ts = millis();
      is_first_entry_calibration = true;
      pid_left.initialise(25, 0.075, 0.01);
      pid_right.initialise(25, 0.075, 0.01);
      pid_distance.initialise(50, 0.5, 0.2);
      pid_direction.initialise(1, 0.2, 0.2);
      pid_turn.initialise(1, 0.4, 0.2);
      reset_pid();
      
    }
    
    void speed_est(){
      //unit is rps
      speed_ts = millis();
      unsigned long elapsed = speed_ts - last_speed_ts;
      if (elapsed >= 30){
        long diff_r = count_RIGHT - last_count_RIGHT;
        long diff_l = count_LEFT  - last_count_LEFT;
        last_count_RIGHT = count_RIGHT;
        last_count_LEFT  = count_LEFT;

        float raw_r = (diff_r/358.3) * 1000 / elapsed;
        float raw_l = (diff_l/358.3) * 1000 / elapsed;

        right_speed = filter_right.filter(raw_r, 0.2);
        left_speed  = filter_left.filter(raw_l,  0.2);

        last_speed_ts = speed_ts;
      }
    }
    
    void reset_pid(){
      pid_left.reset();
      pid_right.reset();
      pid_distance.reset();
      pid_direction.reset();
      pid_turn.reset();
    }
    
    void pid_speed_control(float target_left_speed, float target_right_speed){
      pid_speed_control_ts = millis();
      if(pid_speed_control_ts - last_pid_speed_control_ts > 10){
      left_pid_pwm = pid_left.update(target_left_speed, left_speed, 14); 
      right_pid_pwm = pid_right.update(target_right_speed, right_speed);  
      last_pid_speed_control_ts = pid_speed_control_ts;
      }
      motors.setPWM(left_pid_pwm, right_pid_pwm);
    }
    
    void stopping(unsigned long duration){
      if(stopping_init){
        stopping_ts = millis() + duration;
        stopping_init = false;
      }
      motors.setPWM(0.0, 0.0);
    }

    bool check_stopping(){
      if(millis() > stopping_ts && !stopping_init){
        stopping_init = true;
        motors.setPWM(0.0, 0.0);
        return true;
      }
      return false;
    }
    void pose_est(){
      pose_est_ts = millis();
      if(pose_est_ts - last_pose_est_ts > 30){
        pose.update();
        //print_global_coordinates();
        last_pose_est_ts = pose_est_ts;
      }
    }

    void moving(float target_speed, unsigned long duration){  
      if(moving_init){
        moving_ts = millis() + duration;
        moving_init = false;
      }
      pid_speed_control(target_speed, target_speed);
    }

    bool check_moving(){
      if (millis() >= moving_ts){
        moving_init = true;
        return true;
      }
      else{
        return false;
      }
    }

    float normalised_angle_diff(float diff){
      if(diff > PI){
        diff -= 2 * PI;
      }
      else if(diff < -PI){
        diff += 2 * PI;
      }
      return diff;
    }

    float omega_conversion(float omega_rad){
      return omega_rad / (2 * PI * wheel_radius);
    }

    void differential_contol(float velocity, float omega_rad){
      float omega = omega_conversion(omega_rad);
      float v_right = velocity + omega*wheel_sep;
      float v_left = velocity - omega*wheel_sep;
      pid_speed_control(v_left, v_right);
    }

    bool check_differential_control(float angle){
      normalised_diff = normalised_angle_diff(angle - pose.theta);
      if(abs(normalised_diff) <= SPIN_THRESHOLD){
        reset_pid();
        return true;
      }
      else{
        return false;
      }
    }

    void calibrated_IR_Digital(){
      ir.calibrated_IR_Digital(irsMIN, irsRANGE);
      for(int i=0; i < NUM_SENSORS; i++){
        readings[i] = ir.filter_elapsed[i]; 
      }
    }

    void distance_adjust(){
      float accumulator = 0;
      for(int i = 1; i < NUM_SENSORS-1; i++){
        accumulator += ir.filter_elapsed[i];
      }
      float average = accumulator/3;
      adjust_speed = pid_distance.update(average, 0.07);
      if (adjust_speed < 0.75){
        adjust_speed = 0.75;
      }
      adjust_speed = motors.limit(adjust_speed, 0.85);
    }

    void direction_adjust(){
      float accumulator = 0;
      accumulator -= readings[1];
      accumulator += readings[3];

      if(accumulator < 0){
        //右拐，比例因子=accumulator
        accumulator = -0.02*accumulator;
        direction_speed = pid_direction.update(accumulator, 0);
        direction_speed = motors.limit(adjust_speed, 0.1);
        //pid_speed_control(adjust_speed + direction_speed, adjust_speed - direction_speed);
      }
      else if(accumulator > 0){
        //左拐，比例因子=accumulator
        accumulator = 0.02*accumulator;
        direction_speed = pid_direction.update(accumulator, 0);
        direction_speed = motors.limit(adjust_speed, 0.1);
        //pid_speed_control(adjust_speed - direction_speed, adjust_speed + direction_speed); 
      }
    }

    void turn_adjust(){
      float accumulator = 0;
      accumulator -= readings[0];
      accumulator += readings[4];

      if(accumulator < 0){
        //右拐，比例因子=accumulator
        accumulator = -0.05*accumulator;
        turn_speed = pid_turn.update(accumulator, 0);
        turn_speed = motors.limit(turn_speed, 0.1);
        pid_speed_control(adjust_speed + direction_speed + turn_speed, adjust_speed - direction_speed - turn_speed);
      }
      else if(accumulator > 0){
        //左拐，比例因子=accumulator
        accumulator = 0.05*accumulator;
        direction_speed = pid_turn.update(accumulator, 0);
        direction_speed = motors.limit(turn_speed, 0.4);
        pid_speed_control(adjust_speed - direction_speed - turn_speed, adjust_speed + direction_speed + turn_speed); 
      }
    }
    
};
