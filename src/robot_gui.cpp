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

  // Initialize odometry subscriber
  odom_topic_ = "/cooper_1/odom";
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 10,
                                               &RobotGUI::odomCallback, this);

  // Initialize distance service client
  distance_service_name_ = "/get_distance";
  distance_service_client_ =
      nh.serviceClient<std_srvs::Trigger>(distance_service_name_);
  distance_message_ = "";

  // Initialize reset distance service client
  reset_distance_service_name_ = "/reset_distance";
  reset_distance_service_client_ =
      nh.serviceClient<std_srvs::Empty>(reset_distance_service_name_);

  // Initialize laser scan subscriber
  laser_scan_topic_ = "/cooper_1/scan";
  laser_scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
      laser_scan_topic_, 10, &RobotGUI::laserScanCallback, this);

  ROS_INFO("Robot GUI initialized. Subscribed to /%s topic",
           robot_info_topic_.c_str());
  ROS_INFO("Publishing to %s topic", cmd_vel_topic_.c_str());
  ROS_INFO("Subscribed to %s topic", odom_topic_.c_str());
  ROS_INFO("Subscribed to %s topic", laser_scan_topic_.c_str());
  ROS_INFO("Service client created for %s", distance_service_name_.c_str());
  ROS_INFO("Service client created for %s",
           reset_distance_service_name_.c_str());
}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  // Store the received data
  robot_info_data_ = *msg;
  ROS_DEBUG("Robot info received");
}

void RobotGUI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // Store the received odometry data
  odom_data_ = *msg;
  ROS_DEBUG("Odometry received: x=%.2f, y=%.2f, z=%.2f",
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z);
}

void RobotGUI::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Store the received laser scan data
  laser_scan_data_ = *msg;
  ROS_DEBUG("Laser scan received: %zu points", msg->ranges.size());
}

void RobotGUI::drawLaserScan(cv::Mat &frame, int x, int y, int width,
                             int height) {
  // Calculate center point and scale
  int centerX = x + width / 2;
  int centerY = y + height / 2;
  float scale = (width / 2) / 10.0; // Scale: 10 meters = half width

  // Create a radar-style display for laser scan data
  if (laser_scan_data_.ranges.empty()) {
    // Show "waiting for data" message
    cvui::text(frame, x + 60, centerY, "Waiting for scan data...", 0.4,
               0xCECECE);
    return; // No data yet
  }

  // Draw background circle (scan range indicator)
  cv::circle(frame, cv::Point(centerX, centerY), width / 2 - 10,
             cv::Scalar(60, 60, 60), 1);
  cv::circle(frame, cv::Point(centerX, centerY), width / 4,
             cv::Scalar(50, 50, 50), 1);

  // Draw robot at center
  cv::circle(frame, cv::Point(centerX, centerY), 5, cv::Scalar(0, 255, 255),
             -1); // Yellow robot

  // Draw forward direction indicator
  cv::line(frame, cv::Point(centerX, centerY), cv::Point(centerX, centerY - 15),
           cv::Scalar(0, 255, 255), 2);

  // Process laser scan data
  for (size_t i = 0; i < laser_scan_data_.ranges.size(); i++) {
    float range = laser_scan_data_.ranges[i];

    // Skip invalid readings
    if (range < laser_scan_data_.range_min ||
        range > laser_scan_data_.range_max || std::isnan(range) ||
        std::isinf(range)) {
      continue;
    }

    // Calculate angle for this reading
    float angle =
        laser_scan_data_.angle_min + i * laser_scan_data_.angle_increment;

    // Convert polar to Cartesian coordinates
    // Note: In ROS, 0 angle is forward (along x-axis), but in display we want
    // forward to be up So we rotate by -90 degrees (subtract PI/2)
    float displayAngle = angle - M_PI / 2;
    float pointX = range * cos(displayAngle) * scale;
    float pointY = range * sin(displayAngle) * scale;

    // Convert to pixel coordinates
    int pixelX = centerX + static_cast<int>(pointX);
    int pixelY = centerY + static_cast<int>(pointY);

    // Color code by distance: Red (close) -> Yellow (medium) -> Green (far)
    cv::Scalar color;
    if (range < 1.0) {
      color = cv::Scalar(0, 0, 255); // Red - danger!
    } else if (range < 3.0) {
      color = cv::Scalar(0, 165, 255); // Orange - caution
    } else {
      color = cv::Scalar(0, 255, 0); // Green - safe
    }

    // Draw the point
    cv::circle(frame, cv::Point(pixelX, pixelY), 2, color, -1);
  }

  // Draw legend
  cvui::text(frame, x + 5, y + height - 20, "Red: <1m", 0.35, 0x0000FF);
  cvui::text(frame, x + 70, y + height - 20, "Orange: 1-3m", 0.35, 0x00A5FF);
  cvui::text(frame, x + 160, y + height - 20, "Green: >3m", 0.35, 0x00FF00);
}

void RobotGUI::run() {
  // Create the main window frame (width x height)
  cv::Mat frame = cv::Mat(750, 900, CV_8UC3);

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

    // Display robot position (odometry based)
    cvui::window(frame, 20, 510, 560, 120,
                 "Estimated robot position based off odometry");
    cvui::text(frame, 30, 540, "X", 0.5, 0xCECECE);
    cvui::printf(frame, 90, 560, 0.8, 0xCECECE, "%.0f",
                 odom_data_.pose.pose.position.x);

    cvui::text(frame, 210, 540, "Y", 0.5, 0xCECECE);
    cvui::printf(frame, 270, 560, 0.8, 0xCECECE, "%.0f",
                 odom_data_.pose.pose.position.y);

    cvui::text(frame, 390, 540, "Z", 0.5, 0xCECECE);
    cvui::printf(frame, 450, 560, 0.8, 0xCECECE, "%.0f",
                 odom_data_.pose.pose.position.z);

    // Distance Travelled Service Section
    cvui::window(frame, 20, 640, 380, 90, "Distance Travelled");

    // Call button
    if (cvui::button(frame, 30, 660, 100, 40, "Call")) {
      std_srvs::Trigger srv;
      if (distance_service_client_.call(srv)) {
        distance_message_ = srv.response.message;
        ROS_INFO("Distance service response: %s", distance_message_.c_str());
      } else {
        distance_message_ = "Service call failed";
        ROS_ERROR("Failed to call service %s", distance_service_name_.c_str());
      }
    }

    // Reset button
    if (cvui::button(frame, 150, 660, 100, 40, "Reset")) {
      std_srvs::Empty srv;
      if (reset_distance_service_client_.call(srv)) {
        distance_message_ = "0.00";
        ROS_INFO("Distance reset successfully");
      } else {
        ROS_ERROR("Failed to call service %s",
                  reset_distance_service_name_.c_str());
      }
    }

    // Display the distance
    cvui::text(frame, 30, 710, "Distance in meters:", 0.4, 0xCECECE);
    if (!distance_message_.empty()) {
      cvui::printf(frame, 200, 700, 1.0, 0xCECECE, "%s",
                   distance_message_.c_str());
    }

    // Laser Scan Visualization (Top-Down View)
    cvui::window(frame, 600, 50, 280, 280, "Laser Scan (Top View)");
    drawLaserScan(frame, 600, 50, 280, 280);

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
