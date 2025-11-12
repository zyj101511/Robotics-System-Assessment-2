#include "motion.h"

motion_c motion;

enum STATES {LEAD};
STATES currentState;

bool finish_calibration = false;

unsigned long last_send = 0;
unsigned long current_ts = 0;

void setup() {
  motion.initialise(0, 0, 0);
  currentState = LEAD;
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
}

void loop() {
  Serial.println(digitalRead(11));
  current_ts = millis();
  if(current_ts - last_send >= 30){
    motion.resend();
    last_send = current_ts;
  }
}
