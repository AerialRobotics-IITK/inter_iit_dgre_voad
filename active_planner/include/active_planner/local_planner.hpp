#pragma once

#include <stack>
#include <unordered_map>

#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/tf.h>

#include <active_planner/frontier_evaluator.hpp>
#include <active_planner/path_finder.hpp>
#include <active_planner/visualizer.hpp>

namespace local_planner {

typedef mav_msgs::EigenTrajectoryPointVector Trajectory;

class LocalPlanner {
  public:
    LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    bool readyForExit() {
        return exit_;
    }

    bool isActive() {
        return active_;
    }

    void setConstantYaw(const double& yaw) {
        const_yaw_ = yaw;
    }

    void run();

  private:
    enum class YawPolicy { POINT_FACING, ANTICIPATE_VELOCITY, FOLLOW_VELOCITY, CONSTANT };

    inline std::string getHash(const Eigen::Vector3d& coord) {
        return std::to_string(int(coord.x() / voxel_size_)) + "," + std::to_string(int(coord.y() / voxel_size_));
    }

    static inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }
    geometry_msgs::Point convertEigenToGeometryMsg(const Eigen::Vector3d& point);

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odometry_ = *msg;
    }

    bool activateCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& resp) {
        active_ = req.data;
        resp.success = true;
        return true;
    }

    bool exitCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
        exit_ = true;
        return true;
    }

    Trajectory generateTrajectoryThroughWaypoints(const Path& waypoints);
    void applyYawToTrajectory(Trajectory& trajectory, const YawPolicy& policy = YawPolicy::POINT_FACING);
    bool checkForAbort(const uint i, Trajectory& trajectory);

    Eigen::Vector3d getBestFrontier();

    ros::ServiceServer activate_server_;
    ros::ServiceServer exit_server_;

    ros::Subscriber odom_sub_;
    ros::Publisher command_pub_;

    bool active_;
    bool verbose_;
    bool visualize_;
    bool exit_;

    double robot_radius_;
    double voxel_size_;
    double const_yaw_;

    nav_msgs::Odometry odometry_;

    Path frontier_path_;
    std::stack<Eigen::Vector3d> waypoint_queue_;
    std::vector<FrontierEvaluator::FrontierCenter> frontiers_;
    std::unordered_map<std::string, Eigen::Vector3d> visited_frontiers_;
    std::vector<FrontierEvaluator::FrontierCenter> frontier_cache_;

    Trajectory trajectory_;
    PathFinder pathfinder_;
    Visualizer visualizer_;
    FrontierEvaluator evaluator_;
};

}  // namespace local_planner