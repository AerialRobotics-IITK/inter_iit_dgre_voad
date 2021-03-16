#pragma once

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace interiit21::lander {

class LanderNode {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();
    void centreCallback(const detector_msgs::Centre& msg);
    void poseCallback(const detector_msgs::GlobalCoord& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    void land();

  private:
    ros::Subscriber pose_sub_;
    ros::Subscriber centre_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher setpoint_pub_;

    detector_msgs::Centre centre_;
    detector_msgs::GlobalCoord pose_;
    geometry_msgs::PoseStamped setpoint_;
    nav_msgs::Odometry odom_;

    ros::ServiceClient land_client_;

    cv::aruco::DetectorParameters detector_params;
    cv::aruco::Dictionary dictionary;
};

}  // namespace interiit21::lander
