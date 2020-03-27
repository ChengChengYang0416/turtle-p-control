#include "ros/ros.h"
#include <geometry_msgs/Twist.h> 
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "turtlesim/Pose.h"

#define gain_linear 0.2
#define gain_angular 1

geometry_msgs::Twist vel_msg;
turtlesim::Pose now_pos;

double current_pos_x;
double current_pos_y;
double current_theta;

char getch()
{
  int flags = fcntl(0, F_GETFL, 0);
  fcntl(0, F_SETFL, flags | O_NONBLOCK);

  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0) {
      perror("tcsetattr()");
  }
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0) {
      perror("tcsetattr ICANON");
  }
  if (read(0, &buf, 1) < 0) {
      //perror ("read()");
  }
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0) {
      perror ("tcsetattr ~ICANON");
  }
  return (buf);
}

void KeyboardControl()
{
  int c = getch();
  if (c != EOF)
  {
      switch (c)
      {
          case 119:   // move to x+ (w)
              vel_msg.linear.x = 3.0;
              break;

          case 120:   // move to x- (x)
              vel_msg.linear.x = -3.0;
              break;

          case 97:    // turn to z- (d)
              vel_msg.angular.z = 3.0;
              break;

          case 100:   // turn to z+ (a)
              vel_msg.angular.z = -3.0;
      }
  }
}

void current_position(const turtlesim::Pose::ConstPtr& current_pos)
{
  current_pos_x = current_pos->x;
  current_pos_y = current_pos->y;
  current_theta = current_pos->theta;
  ROS_INFO("turtle position : (%f, %f), theta : %f", current_pos_x, current_pos_y, current_theta);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "p_control");
  ros::NodeHandle n;

  // declare publisher
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

  // declare subscriber
  ros::Subscriber CurPose_sub = n.subscribe("/turtle1/pose", 5, current_position);

  // setting frequency as 100 Hz
  ros::Rate loop_rate(100);

  printf("p_control start\n");

  while (ros::ok()){

    // set desired position and angle
    double x_d = 2;
    double y_d = 2;

    // calculate the error
    double e_x = x_d - current_pos_x;
    double e_y = y_d - current_pos_y;
    double e_theta = atan2(e_y, e_x);

    // calculate control input
    double control_x = gain_linear*(sqrt(e_x*e_x+e_y*e_y));
    double control_theta = gain_angular*(e_theta - current_theta);

    vel_msg.linear.x = control_x;
    vel_msg.angular.z = control_theta;

    KeyboardControl();

    turtlesim_pub.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



