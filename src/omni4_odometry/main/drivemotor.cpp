#include "Arduino.h"
#include "drivemotor.h"

drivemotor::drivemotor(int m1a, int m2a, int m3a, int m4a, int m1b, int m2b, int m3b, int m4b){
  _m1a = m1a;
  _m2a = m2a;
  _m3a = m3a;
  _m4a = m4a;
  _m1b = m1b;
  _m2b = m2b;
  _m3b = m3b;
  _m4b = m4b;
  pinMode(_m1a,OUTPUT);
  pinMode(_m2a,OUTPUT);
  pinMode(_m3a,OUTPUT);
  pinMode(_m4a,OUTPUT);
  pinMode(_m1b,OUTPUT);
  pinMode(_m2b,OUTPUT);
  pinMode(_m3b,OUTPUT);
  pinMode(_m4b,OUTPUT);
  Serial.println("PINNED");
  
}

void drivemotor::writeMotor(float angle, int value, int omega){
  _angle = angle;
  _value = value;
  _omega = omega;
  
  if(cos(_angle*DEG_TO_RAD - PI/4) + _omega/4 > 0){
      analogWrite(_m1a,_value * cos(_angle*DEG_TO_RAD - PI/4) + _omega/4);
      analogWrite(_m1b,0);
      Serial.print("1forward ");
      Serial.println(_value * cos(_angle*DEG_TO_RAD - PI/4) + _omega/4);
  } else if(cos(_angle*DEG_TO_RAD - PI/4) + _omega/4 < 0){
      analogWrite(_m1b,-(_value * cos(_angle*DEG_TO_RAD - PI/4) + _omega/4));
      analogWrite(_m1a,0);
      Serial.print("1backward ");
      Serial.println(-(_value * cos(_angle*DEG_TO_RAD - PI/4) + _omega/4));
  }

  if(cos(_angle*DEG_TO_RAD + PI/4) + _omega/4 > 0){
      analogWrite(_m2a,_value * cos(_angle*DEG_TO_RAD + PI/4) + _omega/4);
      analogWrite(_m2b,0);
      Serial.print("2forward ");
      Serial.println(_value * cos(_angle*DEG_TO_RAD + PI/4) + _omega/4);
  } else if(cos(_angle*DEG_TO_RAD + PI/4) + _omega/4 < 0){
      Serial.print("2backward ");
      Serial.println(-(_value * cos(_angle*DEG_TO_RAD + PI/4) + _omega/4));
      analogWrite(_m2b,-(_value * cos(_angle*DEG_TO_RAD + PI/4) + _omega/4));
      analogWrite(_m2a,0);
  }

  if(cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4 > 0){
      analogWrite(_m3a,_value * cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4);
      analogWrite(_m3b,0);
      Serial.print("3forward ");
      Serial.println(_value * cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4);
  } else if(cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4 < 0){
      analogWrite(_m3b,-(_value * cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4));
      analogWrite(_m3a,0);
      Serial.print("3backward ");
      Serial.println(-(_value * cos(5*PI/4 - _angle*DEG_TO_RAD) + _omega/4));
  }

  if(cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4 > 0){
      analogWrite(_m4a,_value * cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4);
      analogWrite(_m4b,0);
      Serial.print("4forward ");
      Serial.println(_value * cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4);
  } else if(cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4 < 0){
      analogWrite(_m4b,-(_value * cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4));
      analogWrite(_m4a,0);
      Serial.print("4backward ");
      Serial.println(-(_value * cos(3*PI/4 - _angle*DEG_TO_RAD) + _omega/4));
  }
}

void drivemotor::kill(){
      analogWrite(_m1a,0);
      analogWrite(_m1b,0);
      Serial.print("1kill ");
      Serial.println(0);

      analogWrite(_m2a,0);
      analogWrite(_m2b,0);
      Serial.print("2kill ");
      Serial.println(0);

      analogWrite(_m3a,0);
      analogWrite(_m3b,0);
      Serial.print("3kill ");
      Serial.println(0);

      analogWrite(_m4a,0);
      analogWrite(_m4b,0);
      Serial.print("4kill ");
      Serial.println(0);
}
