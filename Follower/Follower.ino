#include "motion.h"

motion_c motion;

enum STATES {CALIBRATION, FOLLOW};
STATES currentState;

bool finish_calibration = false;

void setup() {
  delay(2000);
  motion.initialise(0, 0, 0);
  currentState = FOLLOW;
}

void loop() {
  motion.speed_est();
  motion.pose_est();
  motion.detect_light();
  switch(currentState){
    
  case CALIBRATION: calibration(); break;
      
  case FOLLOW: 
    motion.print_irs();
    break;

  
  }
}

void calibration(){
  if(!finish_calibration){
    finish_calibration = motion.calibration();
  }
  else{
    currentState = FOLLOW;
  }
}
