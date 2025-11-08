#ifndef _MOTORS_H
#define _MOTORS_H

// ===== Pololu 3Pi+ 32U4 default pins (Arduino numbering) =====
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15


#define FWD LOW
#define BWD HIGH

#define MAX_PWM 70.0f

class Motors_c {
  int clamp_pwm(int v){ if(v>MAX_PWM) return MAX_PWM; if(v<-MAX_PWM) return -MAX_PWM; return v; }
  public:
    Motors_c() {}

    void initialise() {
      pinMode(L_DIR, OUTPUT);
      pinMode(R_DIR, OUTPUT);
      pinMode(L_PWM, OUTPUT);
      pinMode(R_PWM, OUTPUT);

      // 安全初始状态
      digitalWrite(L_DIR, FWD);
      digitalWrite(R_DIR, FWD);
      
      analogWrite(L_PWM, 0);
      analogWrite(R_PWM, 0);
    }

    // 正数前进，负数后退
    void setPWM(float left, float right) {
      left = limit(left, MAX_PWM);
      right = limit(right, MAX_PWM);
      
      if (left < 0) {
        digitalWrite(L_DIR, BWD);
        left = abs(left);
      } else {
        digitalWrite(L_DIR, FWD);
      }

      if (right < 0) {
        digitalWrite(R_DIR, BWD);
        right = abs(right);
      } else {
        digitalWrite(R_DIR, FWD);
      }
      
      analogWrite( L_PWM, left );
      analogWrite( R_PWM, right );
    }

    float limit(float input, float max_val) {
      int sign;
      if(input >= 0){
        sign = 1;
      }
      else if(input < 0){
        sign = -1;
      }
      if (abs(input) > max_val) {
      return sign * max_val;
      }
      return input;
    }
};

#endif
