#include "motion.h"

motion_c motion;

enum STATES {FOLLOW};
STATES currentState;

bool finish_calibration = false;

void setup() {
  motion.initialise(0, 0, 0);
  pinMode(EMIT_PIN, INPUT);
  Serial.begin(9600);
  currentState = FOLLOW;
  motion.reset_pid();
}

void loop() {
  motion.speed_est();
  motion.pose_est();
  motion.calibrated_IR_Digital();
  
  switch(currentState){
      
  case FOLLOW:
    motion.distance_adjust(); 
    motion.direction_adjust();
    motion.turn_adjust();
    
    break;
  }
}

/*void calibration(){
  if(!finish_calibration){
    finish_calibration = motion.calibration();
  }
  else{
    currentState = FOLLOW;
    motion.reset_pid();
  }
}*/
