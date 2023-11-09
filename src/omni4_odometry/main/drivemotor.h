#ifndef drivemotor_h
#define drivemotor_h
#include "Arduino.h" 

class drivemotor{
  public:
    drivemotor(int m1a, int m2a, int m3a, int m4a, int m1b, int m2b, int m3b, int m4b);
    void writeMotor(float angle, int value, int omega);
    void kill();
   private:
    int _m1a;
    int _m2a;
    int _m3a;
    int _m4a;
    int _m1b;
    int _m2b;
    int _m3b;
    int _m4b;
    float _angle;
    int _value;
    int _omega; 
};
#endif
