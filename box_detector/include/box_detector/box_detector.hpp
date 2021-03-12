#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <box_detector/libbox_detection.hpp>
#include <detector_msgs/Centre.h>

namespace interiit21::box_detection {

class BoxDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat img_;

    ros::Subscriber img_sub_;

    ros::Publisher centre_pub_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;

    BoxDetector detect_;

    detector_msgs::Centre centre_coord_;
};

}  // namespace interiit21::box_detection
