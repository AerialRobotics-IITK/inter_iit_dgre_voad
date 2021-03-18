#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <box_detector/libbox_detection.hpp>
#include <detector_msgs/Centre.h>
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
    void calculateCentre();
    void detectArucoMarker();
    void takeoff();

    cv::Mat img_;

    ros::Subscriber img_sub_;

    ros::Publisher centre_pub_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;

    BoxDetector detect_;
    double area_ = -1;
    double distance_ = -1;
    double scale_factor_ = 17424;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Mat outputImage;

    detector_msgs::Centre centre_;
};

}  // namespace interiit21::box_detection
