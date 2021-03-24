#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


namespace interiit21::box_detection {

class BoxDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void image1Callback(const sensor_msgs::ImageConstPtr& msg);
    void image2Callback(const sensor_msgs::ImageConstPtr& msg);
    void findGlobalCoordinates();
    void detectArucoMarker();
    void odomCallback(const nav_msgs::Odometry& msg);
    // void cameraInfoCallback(const sensor_msgs::CameraInfo& msg);
    void arrayToMatrixConversion();

    ros::Subscriber img1_sub_;
    ros::Subscriber img2_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher centre_pub_;
    ros::Publisher global_coord_pub_;
    ros::Publisher contour_pub_;

    nav_msgs::Odometry odom_;
    detector_msgs::Centre marker_centre_;
    detector_msgs::GlobalCoord global_coord_;
    sensor_msgs::ImagePtr contour_msg;
    geometry_msgs::PoseStamped pose_;
    sensor_msgs::CameraInfo camera_parameters_;
    
    Eigen::Matrix3d cameraToQuadMatrix;
    Eigen::Matrix3d quadOrientationMatrix;
    Eigen::Vector3d translation_, camera_translation_vector_;
    Eigen::Vector3d global_coordinates_;
    Eigen::Vector2d centre_;

    cv::Mat img_;
    cv::Mat img1_;
    cv::Mat img2_;
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
