#ifndef _FILTER_H
#define _FILTER_H


// this class is a Exponential Moving Average low pass filter
class Filter_c{

  private:
    float last_filtered;
  public:

    Filter_c() {
    }
    
    void initialise() {
      last_filtered = 0.0;
    }
    
    float filter(float original, float alpha){ 
      
      if (alpha < 0.0f || alpha > 1.0f) alpha = 0.5f;

      float filtered = alpha * original + (1.0f - alpha) * last_filtered;
  
      last_filtered = filtered;
      return filtered;
    }
};
#endif
