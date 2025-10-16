#include "robot_gui/robot_gui.h"

RobotGUI::RobotGUI() {
  // Initialize ROS node handle
  ros::NodeHandle nh;

  // Set topic name for robot info
  robot_info_topic_ = "robot_info";

  // Subscribe to robot_info topic
  robot_info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      robot_info_topic_, 10, &RobotGUI::robotInfoCallback, this);

  ROS_INFO("Robot GUI initialized. Subscribed to /%s topic",
           robot_info_topic_.c_str());
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
