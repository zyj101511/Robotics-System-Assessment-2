#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"
#include "filter.h"
#include "Transmission.h"

#define SPIN_THRESHOLD 0.05
#define TRACKING_TORLERANCE 8
#define NUM_IRS 5

Kinematics_c pose;
Motors_c motors;
PID_c pid_left;
PID_c pid_right;
PID_c pid_spin;
PID_c pid_track; 
Filter_c filter_left;
Filter_c filter_right;



class motion_c{
  private:

   unsigned long speed_timestap = 0;
   unsigned long last_speed_timestap = 0;
   unsigned long pid_speed_control_ts = 0;
   unsigned long last_pid_speed_control_ts = 0;
   unsigned long calibration_ts = 0;
   long last_count_RIGHT;
   long last_count_LEFT;
   float left_pid_pwm;
   float right_pid_pwm;
   float MAX_SPEED = 0.1f;
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
   bool calibration_init;
     
  public:
    float left_speed;
    float right_speed;

    // arrays for IR-sensor calibration
    int irsMAX[NUM_IRS];
    int irsMIN[NUM_IRS];
    int irsRANGE[NUM_IRS];
    
    motion_c(){
    }


    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      setupTimer3();
      
      speed_timestap = millis();
      last_speed_timestap = millis();
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
      calibration_ts = millis();
      calibration_init = true;
      pid_left.initialise(90, 0.52, 0.12);
      pid_right.initialise(90, 0.5, 0.1);
      pid_spin.initialise(90, 0.2, 0.1);
      pid_track.initialise(90, 0.3, 0.0);
      reset_pid();
      
    }
  
    void speed_est(){
      float kBaseScale = (33.0 * PI) / (10 * 358.3);
      speed_timestap = millis();
      unsigned long elapsed = speed_timestap - last_speed_timestap;
      if (elapsed >= 10){
        long diff_r = count_RIGHT - last_count_RIGHT;
        long diff_l = count_LEFT  - last_count_LEFT;
        last_count_RIGHT = count_RIGHT;
        last_count_LEFT  = count_LEFT;

        float raw_r = diff_r * kBaseScale;
        float raw_l = diff_l * kBaseScale;

        right_speed = filter_right.filter(raw_r, 0.2);
        left_speed  = filter_left.filter(raw_l,  0.2);

        last_speed_timestap = speed_timestap;
      }
    }
    
    void reset_pid(){
      pid_left.reset();
      pid_right.reset();
      pid_spin.reset();
      pid_track.reset();
    }
    
    void pid_speed_control(float target_left_speed, float target_right_speed){
      pid_speed_control_ts = millis();
      if(pid_speed_control_ts - last_pid_speed_control_ts > 10){
      left_pid_pwm = pid_left.update(target_left_speed, left_speed); 
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
      if(pose_est_ts - last_pose_est_ts > 5){
        pose.update();
        //print_global_coordinates();
        last_pose_est_ts = pose_est_ts;
      }
    }
    
    void spin(float target_angle, float max_speed = 0.1){
      raw_diff = target_angle - pose.raw_theta;
      float demand_spin_speed = pid_spin.update(0, raw_diff);
      demand_spin_speed = motors.limit(demand_spin_speed, max_speed);
      pid_speed_control(demand_spin_speed, -demand_spin_speed);
    }

    bool check_spin(){
      if(abs(raw_diff) <= SPIN_THRESHOLD){
        stopping(100);
        reset_pid();
        return true;
      }
      else{
        return false;
      }
    }

    void turn(float target_angle){
      normalised_diff = target_angle - pose.theta;
      
      if(normalised_diff > PI){
        normalised_diff -= 2 * PI;
      }
      else if(normalised_diff < -PI){
        normalised_diff += 2 * PI;
      }
      float demand_turn_speed = pid_spin.update(0, normalised_diff);
      demand_turn_speed = motors.limit(demand_turn_speed, MAX_SPEED);
      pid_speed_control(demand_turn_speed, -demand_turn_speed);
    }

    bool check_turn(){
      if(abs(normalised_diff) <= SPIN_THRESHOLD){
        stopping(100);
        reset_pid();
        return true;
      }
      else{
        return false;
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

    void tracking(float target_x, float target_y){
      tracking_x_diff = target_x - pose.x;
      tracking_y_diff = target_y - pose.y;
      float target_angle = atan2(tracking_y_diff, tracking_x_diff);
      float distance = tracking_x_diff * tracking_x_diff;
      distance += tracking_y_diff * tracking_y_diff;
      distance = sqrt(distance);
      angle_diff = target_angle - pose.theta;
      if(angle_diff > PI){
        angle_diff -= 2 * PI;
      }
      else if(angle_diff < -PI){
        angle_diff += 2 * PI;
      }
      
      if (!turn_finished){
        turn(target_angle);
        turn_finished = check_turn();
      }
      else if(turn_finished && !end_finished){
        //Serial.println(end_finished);
        // stopping(5000);
        end_finished = true;
        reset_pid();
      }
      else if (turn_finished && end_finished){
        float turn_speed = pid_spin.update(0, angle_diff);
        float tracking_speed = pid_track.update(distance, 0);
        turn_speed = motors.limit(turn_speed, MAX_SPEED);
        tracking_speed = motors.limit(tracking_speed, MAX_SPEED);
        pid_speed_control(tracking_speed + 0.45*turn_speed, tracking_speed - 0.45*turn_speed);
      }
    }

    bool check_tracking(){
      //if(abs(normalised_diff) < SPIN_THRESHOLD){
        if(abs(tracking_x_diff) < TRACKING_TORLERANCE && abs(tracking_y_diff) < TRACKING_TORLERANCE){
          //motors.setPWM(0.0, 0.0);
          turn_finished = false;
          end_finished = false;
          reset_pid();
          return true;
        } 
      //}
      else{
        return false;
      }
    }
    void reset_tracking(){
      turn_finished = false;
      end_finished = false;
    }

    void resend(){
      sending = true;
    }
    
};
