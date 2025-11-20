#include "motion.h"

motion_c motion;

enum STATES {WAIT, FORWARD, TURN, FORWARD_2, STOP};
STATES currentState;

bool finish_calibration = false;

unsigned long last_send = 0;
unsigned long current_ts = 0;
unsigned long start_ts = 0;
unsigned long last_plot = 0;

void setup () {
  motion.initialise(0, 0, 0);
  
  //motion.init_transmission();
  
  currentState = FORWARD;
  pinMode (11, OUTPUT);
  digitalWrite (11, LOW);
  start_ts = millis();

}

void loop () {
  motion.speed_est();
  motion.pose_est();
  current_ts = millis();
  
  // Sending data every 50ms
  if(current_ts - last_send >= 100){
    motion.resend();
    last_send = current_ts;
  }

  switch (currentState){
    case WAIT:
      wait (start_ts);
      break;

    case FORWARD:
      forward ();
      break;

    case TURN:
      turn ();
      break;

    case FORWARD_2:
      forward_2 ();
      break;

    case STOP:
      stopping ();
      break;
      
  }
  if(millis() - last_plot > 50){
   last_plot = millis();
   Serial.print(motion.left_speed*10);
   Serial.print(",");
   Serial.println(motion.right_speed*10);
  }
}

void wait(unsigned long start){
  if(millis () - start > 2000){
    currentState = FORWARD;
    motion.reset_pid ();
  }
}

void forward (){
  motion.moving(-0.8, 3000);
  if(motion.check_moving ()){
    currentState = TURN;
  }
}

void turn (){
  motion.differential_contol (-0.8, 0.4);
  if(motion.check_differential_control(PI/2)){
    currentState = FORWARD_2;
  }
}

void forward_2 (){
  motion.moving (-0.8, 6000);
  if(motion.check_moving ()){
    currentState = STOP;
  }
}

void stopping(){
  motion.stopping(10);
}
 
  
