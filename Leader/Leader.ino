
#include "motion.h"
#define NUM_TARGETS 6

motion_c motion;

enum STATES {CALIBRATION, TRACKING, WAIT, PUSH, RETURN};

STATES currentState;

float points_x[NUM_TARGETS] = {158, 344, 398, 420, 251, 62};
float points_y[NUM_TARGETS] = {85, -10, 108, 261, 208, 261};
int index;

unsigned long start_ts;
bool is_puck = false;
bool is_black = false;
bool finish_stop = false;
bool finish_tracking = false;
bool finish_calibration = false;
bool finish_move = false;
bool finish_return = false;
  
unsigned long timer_ts = millis();
unsigned long timer = millis();

void setup() {
  delay(2000);
  motion.initialise(0, 0, 0);
  index = 0;
  is_puck = false;
  is_black = false;
  finish_stop = false;
  finish_tracking = false;
  finish_calibration = false;
  finish_move = false;
  
  timer_ts = millis();
  timer = millis();

  currentState = CALIBRATION;
}

void loop() {
  timer = millis();
  motion.speed_est();
  motion.pose_est();
  if(timer - timer_ts <= 240000){
    switch(currentState){
      case CALIBRATION: calibration(); break;
      
      case TRACKING: tracking(); break;
      
      case WAIT: waiting();break;
      
      case PUSH: push(); break;
      
      case RETURN: returnhome(); break;
      
    }
  }

  else if(timer - timer_ts > 240000){ 
    motion.stopping(1000);
  }
}

void calibration(){
  if(!finish_calibration){
    finish_calibration = motion.calibration();
  }
  else{
    currentState = TRACKING;
  }
}

void tracking(){
  motion.tracking(points_x[index], points_y[index]);
  is_puck = motion.puck();
  if(!is_puck){
    finish_tracking = motion.check_tracking();
    if(finish_tracking){
      finish_tracking = false;
      if(index < 5){
        index += 1;
      }
      else if(index == 5){
        index = 0;
      }
      currentState = TRACKING;
    }
  }
  else if(is_puck){
    motion.reset_pid();
    currentState = PUSH;
  }
}

void push(){
  if(!finish_stop){
    motion.stopping(1000);
    finish_stop = motion.check_stopping();
  }
  else if(finish_stop && !is_black){
    motion.moving(0.8, 100000);
    is_black = motion.detect_black();
  }
  else if(finish_stop && is_black){
    finish_stop = false;
    is_black = false;
    motion.stopping(1000);
    motion.reset_pid();
    motion.reset_tracking();
    currentState = RETURN;
  }
}

void returnhome(){
  if(!finish_return){
    motion.tracking(0, 0);
    finish_return = motion.check_tracking();
  }
  else if(finish_return){
    motion.reset_pid();
    index = 0;
    finish_return = false;
    currentState = WAIT;
  }
}
void waiting(){
  if(!finish_stop){
    motion.stopping(4000);
    finish_stop = motion.check_stopping();
  }
  else if(finish_stop){
    motion.reset_pid();
    finish_stop = false;
    currentState = TRACKING;
  }
}
