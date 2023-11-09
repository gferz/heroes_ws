#undef ESP32
#include "drivemotor.h"
#include "2_Odometry.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros.h>
#define ESP32

#define m1a 18
#define m1b 17
#define m2a 19
#define m2b 21
#define m3a 13                                 //Pendefinisian pin keempat motor
#define m3b 27
#define m4a 5
#define m4b 4

drivemotor Motor(m1a,m2a,m3a,m4a,m1b,m2b,m3b,m4b);

ros::NodeHandle  nh;

float x_input,y_input,z_input;


void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){
  // Serial.println("GOBLOKKKKKKKK");

  // odometry.moveRobot(100, 0);
  x_input = velocity_msg.linear.x * 100;
  y_input = velocity_msg.linear.y * 100;
  z_input = velocity_msg.angular.z * 100;

  Serial.println("Data From Keyboard");
  Serial.print("x : "); Serial.println(x_input);
  Serial.print("y : "); Serial.println(y_input);
  Serial.print("z : "); Serial.println(z_input);
  Serial.println("------------");

  float angle = atan2(y_input, x_input)*RAD_TO_DEG;
  int value = sqrt(sq(x_input) + sq(y_input));
  Motor.writeMotor(angle, value, z_input);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );



void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(x_pos);
  nh.advertise(y_pos);
  nh.advertise(theta_pos);
}

void loop() {
  // Motor.writeMotor(0, 50, 0);
  x_pos.publish(&x_pos_msg);
  y_pos.publish(&y_pos_msg);
  theta_pos.publish(&theta_pos_msg);
  odometry.odometry();
  odometry.serialPrint();
  nh.spinOnce();
}
