#include "motion.h"
#include "LineSensors.h"

motion_c motion;
IR_c ir;

enum STATES {CALIBRATION, FOLLOW};
STATES currentState;

bool finish_calibration = false;

void setup() {
  motion.initialise(0, 0, 0);
  pinMode(EMIT_PIN, INPUT);
  Serial.begin(9600);
  delay(1500);
  currentState = FOLLOW;
}

void loop() {
  motion.speed_est();
  motion.pose_est();
  ir.IR_Digital();
  switch(currentState){
    
  case CALIBRATION: break;
      
  case FOLLOW: 
    break;

  
  }
}

void calibration(){
  if(!finish_calibration){
    //finish_calibration = motion.calibration();
  }
  else{
    currentState = FOLLOW;
  }
}
