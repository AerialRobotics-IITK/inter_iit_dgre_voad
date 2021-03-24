#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
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
    void image1Callback(const sensor_msgs::ImageConstPtr& msg);
    void image2Callback(const sensor_msgs::ImageConstPtr& msg);
    void findGlobalCoordinates();
    void detectArucoMarker();
    void odomCallback(const nav_msgs::Odometry& msg);
    // void cameraInfoCallback(const sensor_msgs::CameraInfo& msg);
    void arrayToMatrixConversion();

    ros::Subscriber img1_sub_;
    ros::Subscriber img2_sub_;
    ros::Subscriber odom_sub_;  // subscriber for the odometry
    ros::Subscriber camera_info_sub_;
    ros::Publisher centre_pub_;        // Publishes the centre of the detected marker
    ros::Publisher global_coord_pub_;  // Publishes the global coordinate of the detected marker
    ros::Publisher contour_pub_;       // Publishes the contour of the image

    nav_msgs::Odometry odom_;                  // Stores the odometry
    detector_msgs::Centre marker_centre_;      // Stores the centre of the detected marker
    detector_msgs::GlobalCoord global_coord_;  // Stores the global coordinate of the detected marker
    sensor_msgs::ImagePtr contour_msg;         // Stores the contour of the image
    geometry_msgs::PoseStamped pose_;
    sensor_msgs::CameraInfo camera_parameters_;

    Eigen::Matrix3d cameraToQuadMatrix;  // Rotation matrix to rotate vector from camera to quad frame
    Eigen::Matrix3d quadOrientationMatrix;
    Eigen::Vector3d translation_, camera_translation_vector_;
    Eigen::Vector3d global_coordinates_;  // Stores the global coordinates of the detected marker.
    Eigen::Vector2d centre_;              // Stores the centre of the detected marker.

    cv::Mat img_;  // Stores the image captured by the camera.
    cv::Mat img1_;
    cv::Mat img2_;
    cv::Mat outputImage;   // Makes a copy of the captured image for other computations.
    cv::Mat cameraMatrix;  // Stores the camera matrix
    cv::Mat distCoeffs;

    int marker_size_ = 0.4;  // Size of the detected marker
    int index_;              // Stores the index of the marker having ID 0 so that it can be accessed
    bool debug_;             // Using it to print some values for debugging purposes.
    bool marker_detected_;   // Checks whether the marker with ID 0 has been detected or not.
    float camera_matrix[9];  // Describes the mapping of a camera from 3D points in the world to 2D points
    std::vector<float> camera_to_quad_matrix_, camera_translation_, camera_matrix_;
    std::vector<int> markerIds;  // Stores all the markers detected in the world.
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Vec3d> rvecs, tvecs;  // tvecs store the coordinates of the marker in camera frame while rvecs stores the roll, pitch and yaw of the marker
                                          // with respect to the global frame
};

}  // namespace interiit21::box_detection