#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "cvui/cvui.h"

#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

struct Button {
  std::string label;
  cv::Rect rect;
};

class RobotGuiNode {
public:
  RobotGuiNode()
      : nh_(),
        cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("/cooper_1/cmd_vel", 10)),
        get_distance_client_(nh_.serviceClient<std_srvs::Trigger>("/get_distance")),
        linear_x_(0.0),
        angular_z_(0.0),
        last_odom_x_(0.0),
        last_odom_y_(0.0),
        last_odom_z_(0.0),
        last_service_message_("Press 'Get Distance' to query"),
        window_name_("Robot GUI") {
    robot_info_sub_ = nh_.subscribe("robot_info", 10, &RobotGuiNode::robotInfoCb, this);
    odom_sub_ = nh_.subscribe("/cooper_1/odom", 10, &RobotGuiNode::odomCb, this);

    // CVUI
    cv::namedWindow(window_name_);
    cv::resizeWindow(window_name_, 900, 720);
    cvui::init(window_name_);

    int cx = 450; // center x of the teleop cluster
    int cy = 340; // center y of the teleop cluster
    int bw = 140, bh = 50, spacing = 5;
    // Forward
    buttons_.push_back({"Forward", cv::Rect(cx - bw/2, cy - (bh + spacing) * 1 - bh/2, bw, bh)});
    // Middle row: Left, Stop, Right
    buttons_.push_back({"Left",  cv::Rect(cx - (bw + spacing) - bw/2, cy - bh/2, bw, bh)});
    buttons_.push_back({"Stop",  cv::Rect(cx - bw/2,                      cy - bh/2, bw, bh)});
    buttons_.push_back({"Right", cv::Rect(cx + (bw + spacing) - bw/2,     cy - bh/2, bw, bh)});
    // Backward
    buttons_.push_back({"Backward", cv::Rect(cx - bw/2, cy + (bh + spacing) * 1 - bh/2, bw, bh)});

    // Distance button
    int x0 = 20; int y0 = 560; int w = 2 * bw; int h = bh;
    distance_button_ = {"Get Distance", cv::Rect(x0 + 10, y0 + 30, w, h)};
  }

  void spin() {
    ros::Rate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      draw();
      publishCmdVel();
      int key = cv::waitKey(1);
      if (key == 'q' || key == 27) {
        break;
      }
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber robot_info_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::ServiceClient get_distance_client_;

  std::vector<std::string> robot_info_lines_ = std::vector<std::string>(10, "");
  std::mutex data_mutex_;

  double linear_x_;
  double angular_z_;
  double last_odom_x_;
  double last_odom_y_;
  double last_odom_z_;
  std::string last_service_message_;

  std::string window_name_;
  std::vector<Button> buttons_;
  Button distance_button_;

  

  void robotInfoCb(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(data_mutex_);
    robot_info_lines_[0] = msg->data_field_01;
    robot_info_lines_[1] = msg->data_field_02;
    robot_info_lines_[2] = msg->data_field_03;
    robot_info_lines_[3] = msg->data_field_04;
    robot_info_lines_[4] = msg->data_field_05;
    robot_info_lines_[5] = msg->data_field_06;
    robot_info_lines_[6] = msg->data_field_07;
    robot_info_lines_[7] = msg->data_field_08;
    robot_info_lines_[8] = msg->data_field_09;
    robot_info_lines_[9] = msg->data_field_10;
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(data_mutex_);
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_z_ = msg->pose.pose.position.z;
  }

  void draw() {
    cv::Mat frame(720, 900, CV_8UC3, cv::Scalar(30, 30, 30));

    // General Info
    cv::Rect info_area(20, 20, 860, 200);
    cvui::window(frame, info_area.x, info_area.y, info_area.width, info_area.height, "General Info");
    int y = info_area.y + 40;
    std::vector<std::string> info_copy;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      info_copy = robot_info_lines_;
    }
    for (size_t i = 0; i < info_copy.size(); ++i) {
      if (info_copy[i].empty()) continue;
      cvui::text(frame, info_area.x + 10, y, info_copy[i], 0.5, cv::Scalar(180, 220, 180));
      y += 20;
    }

    // Teleoperation
    cv::Rect teleop_panel(20, 220, 860, 220);
    cvui::window(frame, teleop_panel.x, teleop_panel.y, teleop_panel.width, teleop_panel.height, "Teleoperation");
    const double lin_step = 0.05;    // m/s per click
    const double ang_step = 0.10;    // rad/s per click
    const double lin_max = 1.0;
    const double ang_max = 2.0;
    for (const auto& b : buttons_) {
      if (cvui::button(frame, b.rect.x, b.rect.y, b.rect.width, b.rect.height, b.label)) {
        if (b.label == "Forward") {
          linear_x_ = std::min(lin_max, linear_x_ + lin_step);
        } else if (b.label == "Backward") {
          linear_x_ = std::max(-lin_max, linear_x_ - lin_step);
        } else if (b.label == "Left") {
          angular_z_ = std::min(ang_max, angular_z_ + ang_step);
        } else if (b.label == "Right") {
          angular_z_ = std::max(-ang_max, angular_z_ - ang_step);
        } else if (b.label == "Stop") {
          linear_x_ = 0.0;
          angular_z_ = 0.0;
        }
      }
    }

    // Current Velocities
    cv::Rect vel_panel(20, 440, 420, 100);
    cvui::window(frame, vel_panel.x, vel_panel.y, vel_panel.width, vel_panel.height, "Current Velocities");
    char buf[128];
    snprintf(buf, sizeof(buf), "Linear X: %+0.2f m/s", linear_x_);
    cvui::text(frame, vel_panel.x + 10, vel_panel.y + 45, buf, 0.6, cv::Scalar(200, 200, 255));
    snprintf(buf, sizeof(buf), "Angular Z: %+0.2f rad/s", angular_z_);
    cvui::text(frame, vel_panel.x + 10, vel_panel.y + 80, buf, 0.6, cv::Scalar(200, 200, 255));

    // Robot Position
    cv::Rect odom_panel(460, 440, 420, 100);
    cvui::window(frame, odom_panel.x, odom_panel.y, odom_panel.width, odom_panel.height, "Robot Position");
    double ox, oy, oz;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      ox = last_odom_x_; oy = last_odom_y_; oz = last_odom_z_;
    }
    snprintf(buf, sizeof(buf), "x: %.3f", ox);
    cvui::text(frame, odom_panel.x + 10, odom_panel.y + 45, buf, 0.6, cv::Scalar(180, 255, 180));
    snprintf(buf, sizeof(buf), "y: %.3f", oy);
    cvui::text(frame, odom_panel.x + 150, odom_panel.y + 45, buf, 0.6, cv::Scalar(180, 255, 180));
    snprintf(buf, sizeof(buf), "z: %.3f", oz);
    cvui::text(frame, odom_panel.x + 290, odom_panel.y + 45, buf, 0.6, cv::Scalar(180, 255, 180));

    // Distance Service
    cv::Rect distance_panel(20, 560, 860, 140);
    cvui::window(frame, distance_panel.x, distance_panel.y, distance_panel.width, distance_panel.height, "Distance Travelled (meters)");
    if (cvui::button(frame, distance_button_.rect.x, distance_button_.rect.y, distance_button_.rect.width, distance_button_.rect.height, distance_button_.label)) {
      callDistanceService();
    }
    cvui::text(frame, distance_panel.x + 10, distance_panel.y + 110, last_service_message_, 0.6, cv::Scalar(255, 230, 180));

    cvui::update();
    cv::imshow(window_name_, frame);
  }

  void publishCmdVel() {
    geometry_msgs::Twist t;
    t.linear.x = linear_x_;
    t.angular.z = angular_z_;
    cmd_vel_pub_.publish(t);
  }

  void callDistanceService() {
    std_srvs::Trigger srv;
    if (get_distance_client_.call(srv)) {
      last_service_message_ = srv.response.message;
    } else {
      last_service_message_ = std::string("Service call failed at ") + ros::this_node::getName();
    }
  }
};

int main(int argc, char** argv) {
  // Troubleshooting X11 errors on remote/virtual displays
  setenv("GDK_DISABLE_XSHM", "1", 0);
  setenv("QT_X11_NO_MITSHM", "1", 0);
  ros::init(argc, argv, "robot_gui_node");
  RobotGuiNode gui;
  gui.spin();
  return 0;
}
