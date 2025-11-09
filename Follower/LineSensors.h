#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 5
#define EMIT_PIN 11
const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4};


// Class to operate the linesensors.
class LineSensors_c {
  
  public:

    float readings[ NUM_SENSORS ];  // for ADC 
    float minimum[ NUM_SENSORS ];
    float maximum[ NUM_SENSORS ];
    float scaling[ NUM_SENSORS ];

    float calibrated[NUM_SENSORS];
    bool readings_bool[ NUM_SENSORS ];

    bool is_charging;
    unsigned long charging_ts;
    unsigned long discharging_ts;
    bool discharging_done[NUM_SENSORS];

    // Constructor, must exist.
    LineSensors_c() {
    }


    void initialiseForADC() {

      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        pinMode( sensor_pins[sensor], INPUT_PULLUP );
      }
      
    } // End of initialiseForADC()

    void readSensorsADC() {

      initialiseForADC();

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        readings[sensor] = analogRead(sensor_pins[sensor]);
      }

    }

   
    void calcCalibratedADC(int sensorsMIN[NUM_SENSORS], int sensorsRANGE[NUM_SENSORS]){

      // Get latest readings (raw values)
      readSensorsADC();

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        calibrated[sensor] = (readings[sensor] - sensorsMIN[sensor]) / (float)sensorsRANGE[sensor];
        if(calibrated[sensor] > 0.7){
          readings_bool[sensor] = true;
        }
        else{
          readings_bool[sensor] = false;
        }
      }
      
    } // End of calcCalibratedADC()


        // Part of the Advanced Exercises for Labsheet 2
    void initialiseForDigital() {
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );
      is_charging = true;
      charging_ts = 0;
      discharging_ts = 0;
      for(int sensor = 0; sensor < NUM_SENSORS; sensor++){
        discharging_done[sensor] = false; 
      }
    } // End of initialiseForDigital()
    
    void readSensorsDigital() {
      for(int sensor = 0; sensor < NUM_SENSORS; sensor++){
        // start cjharging
        if(is_charging){
          pinMode(sensor_pins[sensor], OUTPUT);
          digitalWrite(sensor_pins[sensor], HIGH);
          charging_ts = micros();
          is_charging = !is_charging; 
        }
        // discharging
        if(!is_charging && micros() - charging_ts >= 20){
          discharging_ts = micros();
          pinMode(sensor_pins[sensor], INPUT);
          int elapsed;
          while(digitalRead(sensor_pins[sensor]) != LOW && elapsed <= 2000){
            elapsed = micros() - discharging_ts;
          }
          // unsigned long elapsed = micros() - discharge_start_ts;
          if(digitalRead(sensor_pins[sensor]) == LOW){  // finished or overtime
            readings[sensor] = elapsed;
          }
          else if(elapsed >= 2000){
            elapsed = 2000;
            readings[sensor] = elapsed;
          }
          is_charging = !is_charging; 
        }
        delayMicroseconds(30);
      }
      calcCalibratedDigital();

    } // End of readSensorsDigital()

    void calcCalibratedDigital(){
      for(int sensor = 0; sensor < NUM_SENSORS; sensor++){
        if(readings[sensor] < 1500){
          readings_bool[sensor] = false;  // white
        }
        else{
          readings_bool[sensor] = true;  // black
        }
      }
    }

};



#endif
