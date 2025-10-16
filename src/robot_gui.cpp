#include "robot_gui/robot_gui.h"

RobotGUI::RobotGUI() {
  // Initialize ROS node handle
  ros::NodeHandle nh;

  // Set topic name for robot info
  robot_info_topic_ = "robot_info";

  // Subscribe to robot_info topic
  robot_info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      robot_info_topic_, 10, &RobotGUI::robotInfoCallback, this);

  // Initialize teleoperation publisher
  cmd_vel_topic_ = "/cooper_1/cmd_vel";
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

  // Initialize velocity step sizes
  linear_velocity_step_ = 0.05; // m/s per button press
  angular_velocity_step_ = 0.1; // rad/s per button press

  // Initialize twist message to zero
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;

  ROS_INFO("Robot GUI initialized. Subscribed to /%s topic",
           robot_info_topic_.c_str());
  ROS_INFO("Publishing to %s topic", cmd_vel_topic_.c_str());
}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  // Store the received data
  robot_info_data_ = *msg;
  ROS_DEBUG("Robot info received");
}

void RobotGUI::run() {
  // Create the main window frame (width x height)
  cv::Mat frame = cv::Mat(700, 600, CV_8UC3);

  // Initialize OpenCV window and tell cvui to use it
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with background color (dark gray)
    frame = cv::Scalar(49, 52, 49);

    // Display title
    cvui::text(frame, 20, 20, "Robot Control GUI", 0.6, 0xFFFFFF);

    // Create window for General Info Area
    cvui::window(frame, 20, 50, 560, 220, "Info");

    // Display robot info data (up to 10 fields)
    int yPos = 75;
    int lineSpacing = 20;

    if (!robot_info_data_.data_field_01.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_01, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_02.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_02, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_03.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_03, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_04.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_04, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_05.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_05, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_06.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_06, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_07.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_07, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_08.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_08, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_09.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_09, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    if (!robot_info_data_.data_field_10.empty()) {
      cvui::text(frame, 30, yPos, robot_info_data_.data_field_10, 0.4,
                 0xCECECE);
      yPos += lineSpacing;
    }

    // Teleoperation Buttons Section
    // Forward button
    if (cvui::button(frame, 210, 290, 100, 40, "Forward")) {
      twist_msg_.linear.x += linear_velocity_step_;
      cmd_vel_pub_.publish(twist_msg_);
      ROS_DEBUG("Forward pressed: linear.x = %.2f", twist_msg_.linear.x);
    }

    // Left button
    if (cvui::button(frame, 80, 340, 100, 40, "Left")) {
      twist_msg_.angular.z += angular_velocity_step_;
      cmd_vel_pub_.publish(twist_msg_);
      ROS_DEBUG("Left pressed: angular.z = %.2f", twist_msg_.angular.z);
    }

    // Stop button
    if (cvui::button(frame, 210, 340, 100, 40, "Stop")) {
      twist_msg_.linear.x = 0.0;
      twist_msg_.angular.z = 0.0;
      cmd_vel_pub_.publish(twist_msg_);
      ROS_DEBUG("Stop pressed");
    }

    // Right button
    if (cvui::button(frame, 340, 340, 100, 40, "Right")) {
      twist_msg_.angular.z -= angular_velocity_step_;
      cmd_vel_pub_.publish(twist_msg_);
      ROS_DEBUG("Right pressed: angular.z = %.2f", twist_msg_.angular.z);
    }

    // Backward button
    if (cvui::button(frame, 210, 390, 100, 40, "Backward")) {
      twist_msg_.linear.x -= linear_velocity_step_;
      cmd_vel_pub_.publish(twist_msg_);
      ROS_DEBUG("Backward pressed: linear.x = %.2f", twist_msg_.linear.x);
    }

    // Display current velocities
    cvui::window(frame, 20, 450, 250, 40, "Linear velocity:");
    cvui::printf(frame, 30, 475, 0.4, 0xff0000, "%.2f m/sec",
                 twist_msg_.linear.x);

    cvui::window(frame, 290, 450, 250, 40, "Angular velocity:");
    cvui::printf(frame, 300, 475, 0.4, 0xff0000, "%.2f rad/sec",
                 twist_msg_.angular.z);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    // Spin as a single-threaded node to process callbacks
    ros::spinOnce();
  }
}
