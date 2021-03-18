#pragma once

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace interiit21::explorer {

class ExplorerNode {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();
    void centreCallback(const detector_msgs::Centre& msg);
    void poseCallback(const detector_msgs::GlobalCoord& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    void stateCallback(const mavros_msgs::State& msg);
    void takeoff();
    void land();

  private:
    //*Variables
    detector_msgs::Centre centre_;
    detector_msgs::GlobalCoord pose_;
    geometry_msgs::PoseStamped setpoint_;
    nav_msgs::Odometry odom_;
    mavros_msgs::State current_state_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandTOL land_cmd_;
    mavros_msgs::CommandTOL takeoff_cmd_;

    //*Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber centre_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;

    //*Publishers
    ros::Publisher setpoint_pub_;

    //*ServiceClients
    ros::ServiceClient landing_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient takeoff_client_;
};

}  // namespace interiit21::explorer
