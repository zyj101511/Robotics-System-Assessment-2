#define EMIT_PIN 11
#define NUM_SENSORS 5
const int sensor_pins[NUM_SENSORS] = {A11, A0, A2, A3, A4};


class IR_c {

  
  private:
    enum IR_STATE {CHARGE, WAIT, DISCHARGE, CHECK};
    IR_STATE ir_state;
    unsigned long ts;
    unsigned long start_time;

    bool done[NUM_SENSORS];
    int done_count;
    unsigned long time_ts;
  
  public:

    unsigned long elapsed[NUM_SENSORS];
    float last_elapsed[NUM_SENSORS];
    float norm_elapsed[NUM_SENSORS];
    float filter_elapsed[NUM_SENSORS];
    
    IR_c(){
      ir_state = CHARGE;
      done_count = 0;
      for(int i=0;i<NUM_SENSORS;i++){
        elapsed[i] = 0;
        last_elapsed[i] = 0;
        norm_elapsed[i] = 0;
        done[i] = false;
      }
    }

    void calibrated_IR_Digital(unsigned long irsMIN[NUM_SENSORS], unsigned long irsRANGE[NUM_SENSORS]){
  
      switch(ir_state){
        case CHARGE:  // charging  
          for(int i=0;i<NUM_SENSORS;i++){
            pinMode(sensor_pins[i], OUTPUT);
            digitalWrite(sensor_pins[i], HIGH);
          }
          ts = micros();
          time_ts = millis();
          ir_state = WAIT;
          break;

        case WAIT:  // wait 30us
          if (micros() - ts >= 30){
            for(int i=0;i<NUM_SENSORS;i++){
              pinMode(sensor_pins[i], INPUT);
            }

            start_time = micros();
            for(int i=0;i<NUM_SENSORS;i++){
              done[i] = false;
              elapsed[i] = 0;
            }
            done_count = 0;

            ir_state = DISCHARGE;
          }
          break;

        case DISCHARGE: // check discharge
          for(int i=0;i<NUM_SENSORS;i++){
            if(!done[i] && digitalRead(sensor_pins[i]) == LOW){
              elapsed[i] = micros() - start_time;
              if (elapsed[i] >= 60000){
                elapsed[i] = 60000;
              }
              norm_elapsed[i] = float(elapsed[i] - 1000) / 60000;

              done[i] = true;
              done_count++;
            }
          }

          if (done_count == NUM_SENSORS){
            ir_state = CHECK;
          }
          break;

        case CHECK: // finished and print
        
          for(int i=0;i<NUM_SENSORS;i++){
            filter_elapsed[i] = (5 * norm_elapsed[i] + 5 * last_elapsed[i]) / 10;
          }
        
          /*for(int i=0;i<NUM_SENSORS;i++){
            Serial.print(filter_elapsed[i]);
            if(i < NUM_SENSORS - 1){
              Serial.print(",");
            }
          }
          Serial.println();*/
          //unsigned long time_stop = millis() - time_ts;
          //Serial.println(time_stop);
          for(int i=0;i<NUM_SENSORS;i++){
            last_elapsed[i] = norm_elapsed[i];
          }
          ir_state = CHARGE;  // restart cycle
          break;
      }
      
    }
};
