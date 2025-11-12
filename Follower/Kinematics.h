// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include "Motors.h"  
#include <math.h>

// These two commands mean that this header file
// will attempt to use some global variables of
// the same name found in another header file.
// From encoders.h
extern volatile long count_RIGHT;
extern volatile long count_LEFT;

// Some global definitions concerning
// the robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Labsheet 4)
 float count_per_rev = 358.3;   // From documentation - correct.
 float wheel_radius  = 16.25;    // mm, confirmed by user    // mm, confirmed by user    // mm, could vary - calibrate.
 float wheel_sep     = 43;     // mm, confirmed by user     // mm, confirmed by user    // mm, from centre of robot to wheel centre 
                                     //     - could vary, calibrate

// Take the circumference of the wheel and divide by the 
// number of counts per revolution. This provides the mm
// travelled per encoder count.
 float mm_per_count  = ( 2.0 * wheel_radius * PI ) / count_per_rev;
 const float LEFT_WHEEL_CORRECT  = 1;
 const float RIGHT_WHEEL_CORRECT = 1;

// Class to track robot position.
class Kinematics_c {
  public:

    // Pose
    float x, y, theta, raw_theta;

    // To calculate the difference
    // in encoder counts for each
    // call to update()
    long last_LEFT;
    long last_RIGHT;
  
    // ructor, must exist.
    Kinematics_c() {

    } 

    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      last_RIGHT = count_RIGHT; // Initisalise last count to current count
      last_LEFT = count_LEFT; // Initisalise last count to current count
      x = start_x;
      y = start_y;
      theta = start_th;
    }
    
    // Here I have opted to use encoder counts rather than 
    // wheel velocity.  Either way will work.  
    // With velocity, the difference in time between updates 
    // is required (distance = speed / time ).
    // If we use the velocity, it means we have to do
    // extra computation just to get back to distance, which
    // we had in the first place (as change of encoder counts)
    void update( ) {
      
        long delta_LEFT;  // change in counts
        long delta_RIGHT;  // change in counts
        // float mean_delta;
         
        float x_contribution;   // linear translation
        float th_contribution;  // rotation

        // How many counts since last update()?
        delta_LEFT = count_LEFT - last_LEFT;
        delta_RIGHT = count_RIGHT - last_RIGHT;

        // Used last encoder values, so now update to
        // current for next iteration
        last_LEFT = count_LEFT;
        last_RIGHT = count_RIGHT;
        
        // Work out x contribution in local frame.
        float delta_left = delta_LEFT * LEFT_WHEEL_CORRECT;
        float delta_right = delta_RIGHT * RIGHT_WHEEL_CORRECT;

        x_contribution = (mm_per_count * (delta_left + delta_right)) / 2.0 ;

        // Work out rotation in local frame
        th_contribution = mm_per_count * (delta_right - delta_left);
        th_contribution /= (wheel_sep *2.0);


        // Update global frame by taking these
        // local contributions, projecting a point
        // and adding to global pose.
        x = x + x_contribution * cos( theta );
        y = y + x_contribution * sin( theta );
        theta = theta + th_contribution;
        raw_theta += th_contribution;

        if (theta > PI){
          theta -= 2 * PI;
        }
        else if (theta < -PI){
          theta += 2 * PI;
        }
        // Done!
    } // End of update()

}; // End of Kinematics_c class defintion

#endif
