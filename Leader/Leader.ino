#include "motion.h"

motion_c motion;

enum STATES {LEAD};
STATES currentState;

bool finish_calibration = false;

void setup() {
  delay(2000);
  motion.initialise(0, 0, 0);
  currentState = LEAD;
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
}

void loop() {
  motion.speed_est();
  motion.pose_est();
  switch(currentState){
      
  case LEAD: 
    Serial.println(digitalRead(11));
    break;

  
  }
}
