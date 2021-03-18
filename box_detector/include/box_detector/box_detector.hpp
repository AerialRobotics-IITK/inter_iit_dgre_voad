#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
// #include <detector_msgs/Centre.h>
#include<nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


namespace interiit21::box_detection {

class BoxDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void findGlobalCoordinates();
    void detectArucoMarker();
    void odomCallback(const nav_msgs::Odometry& msg);
    void arrayToMatrixConversion();
    void landing();

    ros::Subscriber img_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher centre_pub_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;
    ros::Publisher pose_pub_;

    nav_msgs::Odometry odom_;
    // detector_msgs::Centre centre_;
    sensor_msgs::ImagePtr contour_msg;
    geometry_msgs::PoseStamped pose_;
    
    Eigen::Matrix3d cameraToQuadMatrix;
    Eigen::Matrix3d quadOrientationMatrix;
    Eigen::Vector3d translation_, camera_translation_vector_;
    Eigen::Vector3d global_coordinates_;
    Eigen::Vector2d centre_;

    cv::Mat img_;
    cv::Mat outputImage;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    int marker_size_ = 0.4;
    int index_;
    bool debug_;  
    bool marker_detected_;
    float camera_matrix[9];
    std::vector<float> camera_to_quad_matrix_, camera_translation_,camera_matrix_;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Vec3d> rvecs, tvecs;
};

}  // namespace interiit21::box_detection
