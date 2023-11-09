#ifndef _ODOMETRY_LIBRARY_H
#define _ODOMETRY_LIBRARY_H
#include <ros.h>
#include <std_msgs/Float64.h>


std_msgs::Float64 x_pos_msg;
std_msgs::Float64 y_pos_msg;
std_msgs::Float64 theta_pos_msg;
ros::Publisher x_pos("x_pos", &x_pos_msg);
ros::Publisher y_pos("y_pos", &y_pos_msg );
ros::Publisher theta_pos("theta_pos", &theta_pos_msg );

float x = 0;          // posisi x
float y = 0;          // posisi y
float theta = 0;      // orientasi
float delta_s = 0;    // jarak yang ditempuh
float delta_theta = 0;// perubahan orientasi

// Variabel encoder
const float PPR = 3600 * PI;
const float WHEEL_RADIUS = 3;  // dalam cm
const float WHEELBASE = 48;     // dalam cm
float xL_distance, y_distance, xR_distance;

const int encoderXL1 = 16;
const int encoderXL2 = 15;
const int encoderY1 = 25;
const int encoderY2 = 26;
const int encoderXR1 = 32;
const int encoderXR2 = 33;

volatile long counterXL = 0;
volatile long counterY = 0;
volatile long counterXR = 0;

volatile long last_counterXL = 0;
volatile long last_counterY = 0;
volatile long last_counterXR = 0;

// Function interrupt encoder
// void IRAM_ATTR countXL1(){
//   if (digitalRead(encoderXL1) == digitalRead(encoderXL2)){
//     counterXL++;
//   }else{
//     counterXL--;
//   }
// }

// void IRAM_ATTR countY1(){
//   if (digitalRead(encoderY1) == digitalRead(encoderY2)){
//     counterY++;
//   }else{
//     counterY--;
//   }
// }

// void IRAM_ATTR countXR1(){
//   if (digitalRead(encoderXR1) == digitalRead(encoderXR2)){
//     counterXR++;
//   }else{
//     counterXR--;
//   }
// }


class BaseOdo{
  // Variabel odometri
 

  public:
  void odometry(){
    // Hitung delta_s dan delta_theta
    xL_distance = 2 * PI * WHEEL_RADIUS * (counterXL - last_counterXL) / PPR;
    xR_distance = 2 * PI * WHEEL_RADIUS * (counterXR - last_counterXR) / PPR;
    y_distance = 2 * PI * WHEEL_RADIUS * (counterY - last_counterY) / PPR;
    delta_s = (xR_distance + xL_distance) / 2;
    delta_theta = (xR_distance - xL_distance) / WHEELBASE;

    // Hitung posisi dan orientasi
    x += delta_s * cos(theta + delta_theta / 2);
    y += y_distance;
    theta += delta_theta;

    x_pos_msg.data = x;
    y_pos_msg.data = y_distance;
    theta_pos_msg.data = theta;
  }
  void serialPrint(){
    // Print hasil odometri
    // Serial.print(" | countXL: ");Serial.print(counterXL);
    // Serial.print(" | countXR: ");Serial.print(counterXR);
    // Serial.print(" | countY: "); Serial.print(counterY);
    // Serial.print(" | distXL: "); Serial.print(xL_distance);
    // Serial.print(" | distXR: "); Serial.print(xR_distance);
    // Serial.print(" | distY: "); Serial.println(y_distance);
    // Serial.print(" | x: "); Serial.print(x);
    // Serial.print(" | y: "); Serial.print(y);
    // Serial.print(" | delta_s: "); Serial.print(delta_s);
    // Serial.print(" | delta_theta: "); Serial.println(delta_theta);
  }
};

BaseOdo odometry;
#endif