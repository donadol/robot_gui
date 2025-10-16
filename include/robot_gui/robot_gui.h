#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>

class RobotGUI {
public:
  RobotGUI();

  void run();

private:
  ros::Subscriber robot_info_sub_;
  robotinfo_msgs::RobotInfo10Fields robot_info_data_;
  std::string robot_info_topic_;

  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

  // Teleoperation members
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist twist_msg_;
  std::string cmd_vel_topic_;
  float linear_velocity_step_;
  float angular_velocity_step_;

  const std::string WINDOW_NAME = "Robot Control GUI";
};
