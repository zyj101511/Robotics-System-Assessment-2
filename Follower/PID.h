
#ifndef _PID_H
#define _PID_H

class PID_c {
  public:

    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    // To store gains.
    float p_gain;
    float i_gain;
    float d_gain;

    // To determine time elapsed.
    unsigned long ms_last_t;
  
    // ructor, must exist.
    PID_c() {

    } 

    void initialise( float p, float i, float d ) {
       feedback = 0;
       last_error = 0;
       p_term = 0;
       i_term = 0;
       d_term = 0;
       i_sum = 0;

       p_gain = p;
       i_gain = i;
       d_gain = d;

       ms_last_t = millis();
    }
    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;
      ms_last_t = millis();
    }
    float update( float demand, float measurement ) {
      float error;
      unsigned long ms_now_t;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;

      // Grab time to calc elapsed time.
      ms_now_t = millis();
      ms_dt = ms_now_t - ms_last_t;

      // ms_last_t has been used, so update
      // it for the next call of this update.
      ms_last_t = ms_now_t;
      
      // typecasting the different of two
      // unsigned longs is safer.
      float_dt = (float)ms_dt;

      if( float_dt == 0.0 ) return feedback;

      // Calculate error signal.
      error = demand - measurement;
      last_error = error;

      // P term, nice and easy
      p_term = p_gain * error;

      // discrete integration
      i_sum = i_sum + (error * float_dt);

      // i_term.
      i_term = i_gain * i_sum;

      diff_error = (error - last_error) / float_dt;
      d_term = diff_error * d_gain;

      feedback = p_term + i_term + d_term;

      // done
      return feedback;
    }

}; // End of PID_c class definition



#endif
