#include <box_detector/box_detector.hpp>

namespace interiit21::box_detection {

void BoxDetectorNode::init(ros::NodeHandle& nh) {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    int canny_lower, canny_upper, canny_ker;
    int min_contour_area;

    img_sub_ = nh.subscribe("image_raw", 1, &BoxDetectorNode::imageCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("h_min", h_min);
    nh_private.getParam("s_min", s_min);
    nh_private.getParam("v_min", v_min);
    nh_private.getParam("h_max", h_max);
    nh_private.getParam("s_max", s_max);
    nh_private.getParam("v_max", v_max);
    nh_private.getParam("canny_lower", canny_lower);
    nh_private.getParam("canny_upper", canny_upper);
    nh_private.getParam("canny_ker", canny_ker);
    nh_private.getParam("min_contour_area", min_contour_area);

    detect_.setHSVMin(h_min, s_min, v_min);
    detect_.setHSVMax(h_max, s_max, v_max);
    detect_.setCannyParams(canny_lower, canny_upper, canny_ker);
    detect_.setMinArea(min_contour_area);

    centre_pub_ = nh_private.advertise<detector_msgs::Centre>("centre_coord", 10);
    thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);

    centre_.x = -1;
    centre_.y = -1;
}

void BoxDetectorNode::run() {
    if (img_.empty()) {
        return;
    };
    
    detectArucoMarker();
    calculateCentre();
    centre_pub_.publish(centre_);
}

void BoxDetectorNode::detectArucoMarker() {
    if (img_.empty()) {
        return;
    };

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::aruco::detectMarkers(img_, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    outputImage = img_.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    // detect_.thresholdImage(img_);
    // detect_.findGoodContours();
    // detect_.drawContours(img_);
    // detect_.fitRect(img_);

    // std::pair<int, int> centre_pair = detect_.getCentre();
    // double distance = detect_.getDistance();
    // double area = detect_.getArea();
    // centre_coord_.x = centre_pair.first;
    // centre_coord_.y = centre_pair.second;
    // centre_coord_.d = (float) distance;
    // centre_coord_.a = (float) area;
    // centre_coord_.header.stamp = ros::Time::now();

    // sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    // sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();

    // thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);
    // centre_pub_.publish(centre_coord_);
}

void BoxDetectorNode::calculateCentre() {
    for (int i = 0; i < markerIds.size(); i++) {
        if (markerIds[i] != 0)
            continue;
        if (markerIds[i] == 0) {
            centre_.x = (markerCorners[i][0].x + markerCorners[i][1].x + markerCorners[i][2].x + markerCorners[i][3].x) / 4;
            centre_.y = (markerCorners[i][0].y + markerCorners[i][1].y + markerCorners[i][2].y + markerCorners[i][3].y) / 4;

            area_ = sqrt((markerCorners[i][1].x - markerCorners[i][0].x) * (markerCorners[i][1].x - markerCorners[i][0].x) +
                         (markerCorners[i][1].y - markerCorners[i][0].y) * (markerCorners[i][1].y - markerCorners[i][0].y)) *
                    sqrt((markerCorners[i][2].x - markerCorners[i][1].x) * (markerCorners[i][2].x - markerCorners[i][1].x) +
                         (markerCorners[i][2].y - markerCorners[i][1].y) * (markerCorners[i][2].y - markerCorners[i][1].y));

            distance_ = sqrt(scale_factor_ / area_);

            std::cout << (distance_ * distance_) * area_ << "\n"
                      << "\n";

            centre_.d = distance_;
            centre_.a = area_;

            std::cout << "Required Marker Detected"
                      << "\n"
                      << "\n";
            cv::circle(outputImage, cv::Point(centre_.x, centre_.y), 4.0, cv::Scalar(0, 0, 255), 1, 8);
            break;
        }
    }
}

void BoxDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr_;

    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_ = cv_ptr_->image;
}

}  // namespace interiit21::box_detection
