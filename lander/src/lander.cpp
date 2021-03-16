#include <lander/lander.hpp>

namespace interiit21::lander {

void LanderNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    pose_sub_ = nh.subscribe("pose", 10, &LanderNode::poseCallback, this);
    centre_sub_ = nh.subscribe("centre", 10, &LanderNode::centreCallback, this);
    odom_sub_ = nh.subscribe("odom", 10, &LanderNode::odomCallback, this);
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("setpoint", 10);
    // land_client_ = nh.serviceClient<>("land");

    // aruco
    dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
}

void LanderNode::run() {
}

void land() {
    //* Move the quad to the (x,y) of the detected white box
    setpoint_.pose.position.x = centre_.x;
    setpoint_.pose.position.y = centre_.y;
    setpoint_.pose.position.z = odom_.pose.position.position.z;

    //
}

void centreCallback(const detector_msgs::Centre& msg) {
    centre_ = msg;
}

void poseCallback(const detector_msgs::GlobalCoord& msg) {
    pose_ = msg;
}

void odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

}  // namespace interiit21::lander
