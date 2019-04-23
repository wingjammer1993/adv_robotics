#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"
// polulu stuff
#include "maestro.h"

int get_throttle_pw(float);
int get_steer_pw(float);

#define FREQ 10
#define QUEUE_SZ 10
#define THROTTLE_CH 2
#define STEER_CH 9

// Brake hard limit is 0.5 -> 6014
// For reverse just send negative inputs for linear.x
#define THROTTLE_UNARMED 6000 // 1560*4
#define THROTTLE_ARMED 6280 // 1570*4
#define THROTTLE_MAX 8000 // 2000*4

#define STEER_LEFT 7200 // 1800*4
#define STEER_NEUTRAL 6000 // 1500*4
#define STEER_RIGHT 4800 // 1200*4

Maestro maestro;

// linear.x = (0, 10)
// angular.x = (0, 10)
void bldc_ctrl_master_reciever(const geometry_msgs::Twist &msg) {
  int throttle_pw = get_throttle_pw(msg.linear.x);
  //int throttle_pw = 6280;
  int steer_pw = get_steer_pw(msg.angular.z);

  ROS_INFO_STREAM(
    "throttle [pw, level]: [" << throttle_pw << ", " << msg.linear.x << "]" <<
    "steer [pw, angle]: [" << steer_pw << ", " << msg.angular.z << "]" << std::endl;
  );

  maestro.setTarget(THROTTLE_CH, throttle_pw );
  maestro.setTarget(STEER_CH, steer_pw );
}

int get_throttle_pw(float level) {
  return (int)(THROTTLE_UNARMED + level*((THROTTLE_MAX - THROTTLE_UNARMED)/10));
}

int get_steer_pw(float level) {
  return (int)(STEER_LEFT + level*((STEER_RIGHT - STEER_LEFT)/(10-0)));
}

void init_maestro(Maestro &maestro){
  maestro.setSpeed(THROTTLE_CH, 100);
  maestro.setAcceleration(THROTTLE_CH, 255);
  maestro.setTarget(THROTTLE_CH, THROTTLE_UNARMED);

  maestro.setSpeed(STEER_CH, 100);
  maestro.setAcceleration(STEER_CH, 255);
  maestro.setTarget(STEER_CH, STEER_NEUTRAL);

}

int main(int argc, char **argv){
  init_maestro(maestro);

  ros::init(argc, argv, "lowlevel_ctrl_sub");
  ros::NodeHandle nh;

  ROS_INFO_STREAM(
    "subscriber initialized"
  );
  ROS_INFO_STREAM(
    "THROTTLE CHANNEL: " << THROTTLE_CH << ", "
    "STEER CHANNEL: " << STEER_CH << std::endl;
  );

  ros::Subscriber subscriber = nh.subscribe("cmd_vel", QUEUE_SZ, &bldc_ctrl_master_reciever);
  ros::spin();
}
