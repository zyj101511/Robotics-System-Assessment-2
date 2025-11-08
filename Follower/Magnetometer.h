
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define NUM_AXIS 3

class Magnetometer_c {

  public:

    LIS3MDL mag;


    float readings[ NUM_AXIS ];
    float calibrated[NUM_AXIS];
    
    // Constructor, must exist.
    Magnetometer_c () {
    }

    void initialise() {

      Wire.begin();
      
      if (!mag.init()) {
        Serial.println("Failed to detect LIS3MDL magnetometer!");
        while (1); 
      }
      mag.enableDefault();
    } 


    void get_mag_readings() {
        mag.read();
        readings[0] = mag.m.x;
        readings[1] = mag.m.y;
        readings[2] = mag.m.z;

    } 

    void calcCalibrated(float axisOFFSET[NUM_AXIS], float axisSCALE[NUM_AXIS]){
      get_mag_readings();
      for(int axe = 0; axe < NUM_AXIS; axe++){
        calibrated[axe] = (readings[axe] - axisOFFSET[axe]) * axisSCALE[axe];
      }
    }

};

#endif
