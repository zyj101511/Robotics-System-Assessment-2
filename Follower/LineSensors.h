#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 3
#define EMIT_PIN 11
const int sensor_pins[ NUM_SENSORS ] = { A0, A2, A3};


// Class to operate the linesensors.
class LineSensors_c {
  
  public:

    float readings[NUM_SENSORS];  // for ADC 
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

};



#endif
