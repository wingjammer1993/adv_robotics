#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <stdlib.h>
#include <algorithm>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define FREQ 10
#define QUEUE_SZ 10

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

std::string TAG = "BLDC_CTRL_PUB";
ros::Publisher publisher;

int kfd = 0;
struct termios cooked, raw;

float MIN_VEL = 0.0;
float MAX_VEL = 10.0;

float MIN_ANG =-30.0;
float MAX_ANG = 30.0;

float linear_vel = MIN_VEL;
float angular_vel = 0;

float linear_acc = 0.5;
float angular_acc = 1;

int linear_, angular_;


int quit_handler() {
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = (MIN_ANG + MAX_ANG)/2;

  publisher.publish(twist);

  exit(0);

}


int main(int argc, char **argv){

  // init
  ros::init(argc, argv, "lowlevel_ctrl_pub");
  // Handler for this node in the ROS ecosystem.
  ros::NodeHandle nh;

  publisher = nh.advertise<geometry_msgs::Twist>(
    "cmd_vel",
    QUEUE_SZ
  );


  char c;
  bool dirty=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the car.");

  while(true){
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);


    switch(c) {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("SIGKILL");
        linear_ = -1;
        quit_handler();
        break;
    }

    geometry_msgs::Twist twist;
    linear_vel += linear_acc*linear_;
    angular_vel += angular_acc*angular_;


    if(linear_ == 1) {
      linear_vel = std::min(MAX_VEL, linear_vel);
    }else {
      linear_vel = std::max(MIN_VEL, linear_vel);
    }

    if(angular_ == 1) {
      angular_vel = std::min(MAX_ANG, angular_vel);
    }else{
      angular_vel = std::max(MIN_ANG, angular_vel);
    }
    twist.linear.x = linear_vel;
    twist.angular.z = angular_vel;
    std::cout << twist.linear.x << ' ' << twist.angular.z << std::endl;
    linear_ = 0;
    angular_ = 0;
    publisher.publish(twist);

  }
  return 0;
}
