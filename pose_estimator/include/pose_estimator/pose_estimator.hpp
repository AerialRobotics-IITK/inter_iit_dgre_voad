#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <pose_estimator/libpose_estimation.hpp>

namespace iarc2020::pose_estimation {

class PoseEstimatorNode {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

  private:
    Eigen::Vector3d calculateGlobCoord(const double& img_x, const double& img_y, const double& dist);
    void centreCallback(const detector_msgs::Centre& msg);
    void odomCallback(const nav_msgs::Odometry& msg);

    int image_height_;
    int image_width_;

    detector_msgs::Centre centre_coord_;
    detector_msgs::GlobalCoord global_coord_;
    detector_msgs::GlobalCoord front_coord_;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d glob_coord_;
    Eigen::Vector3d straight_vec_;

    ros::Subscriber centre_coord_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher glob_coord_pub_;
    ros::Publisher front_coord_pub_;

    PoseEstimator pose_est_;
};

}  // namespace iarc2020::pose_estimation
