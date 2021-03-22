#include <active_planner/local_planner.hpp>

namespace local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : pathfinder_(nh, nh_private)
    , evaluator_(nh, nh_private)
    , active_(false)
    , exit_(false)
    , const_yaw_(0) {
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("sampling_dt", sampling_dt_);

    // pathfinder_ = PathFinder(new PathFinder(nh, nh_private, &server_));
    // evaluator_ = FrontierEvaluator(new FrontierEvaluator(nh, nh_private, &server_));

    odom_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    activate_server_ = nh.advertiseService("activate", &LocalPlanner::activateCallback, this);
    exit_server_ = nh.advertiseService("shutdown", &LocalPlanner::exitCallback, this);

    if (visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("trajectory");
    }
}

geometry_msgs::Point LocalPlanner::convertEigenToGeometryMsg(const Eigen::Vector3d& point) {
    geometry_msgs::Point point_msg;

    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();

    return point_msg;
}

Eigen::Vector3d LocalPlanner::getBestFrontier() {
    geometry_msgs::Point curr_pos = odometry_.pose.pose.position;
    double min_distance = DBL_MAX;
    Eigen::Vector3d best_f;
    for (auto frontier : frontiers_) {
        double distance = norm(curr_pos, convertEigenToGeometryMsg(frontier.center));
        if (min_distance > distance) {
            min_distance = distance;
            best_f = frontier.center;
        }
    }
    return best_f;
}

void LocalPlanner::run() {
    if (waypoint_queue_.empty()) {
        frontiers_.clear();
        evaluator_.findFrontiers();
        frontiers_ = evaluator_.getFrontiers();
        Eigen::Vector3d waypoint = getBestFrontier();
        waypoint_queue_.push(waypoint);
    } else {
        Eigen::Vector3d waypt = waypoint_queue_.front();
        waypoint_queue_.pop();

        mav_msgs::EigenOdometry start_odom;
        mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

        pathfinder_.findPath(start_odom.position_W, waypt);
        frontier_path_ = pathfinder_.getPath();
        trajectory_ = generateTrajectoryThroughWaypoints(frontier_path_);

        ros::Rate pub_rate(20);
        for (auto target : trajectory_) {
            geometry_msgs::PoseStamped setpt;
            mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(target, &setpt);
            command_pub_.publish(setpt);
            ros::spinOnce();
            bool feasible = true;

            while (ros::ok() && (norm(odometry_.pose.pose.position, convertEigenToGeometryMsg(target.position_W)) > voxel_size_)) {
                ros::spinOnce();
                mav_msgs::EigenOdometry start_odom;
                mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

                if (pathfinder_.isLineInCollision(start_odom.position_W, target.position_W)) {
                    ROS_WARN("Aborting current trajectory...");
                    break;
                }
                pub_rate.sleep();
            }

            if (!feasible) {
                trajectory_.clear();
                frontier_path_.clear();
                break;
            }
        }
    }
}

Trajectory LocalPlanner::generateTrajectoryThroughWaypoints(const Path& waypoints) {
    Trajectory traj;
    if (waypoints.empty()) {
        ROS_INFO("Path is empty!");
        // status_thread_ = std::async(std::launch::async, &LocalPlanner::setStatus, this, PlanStatus::FAILURE);
        return traj;
    }

    for (auto& point : waypoints) {
        mav_msgs::EigenTrajectoryPoint traj_pt;
        traj_pt.position_W = point;
        traj.push_back(traj_pt);
    }

    // if (pathfinder_.getPathLength(waypoints) < 0.05) {
    //     applyYawToTrajectory(traj);
    //     if (visualize_) {
    //         visualizer_.visualizeTrajectory("trajectory", traj, "map", Visualizer::ColorType::BLACK, 0.2);
    //     }
    //     return traj;
    // }

    // mav_trajectory_generation::Trajectory gen_traj;
    // ros::spinOnce();

    // TODO: Preserve velocities when switching waypoints
    // bool success = smoother_.getTrajectoryBetweenWaypoints(eigen_waypts, &gen_traj);
    // if (success) {
    // mav_trajectory_generation::sampleWholeTrajectory(gen_traj, sampling_dt_, &traj);
    // } else {
    // }

    applyYawToTrajectory(traj);
    if (visualize_) {
        visualizer_.visualizeTrajectory("trajectory", traj, "map", Visualizer::ColorType::BLACK, 0.2);
    }
    return traj;
}

void LocalPlanner::applyYawToTrajectory(Trajectory& trajectory, const YawPolicy& policy) {
    if (trajectory.size() < 2) {
        return;
    }
    double last_yaw = trajectory.front().getYaw();

    if (policy == YawPolicy::POINT_FACING) {
        for (auto i = 0; i < trajectory.size() - 1; i++) {
            Eigen::Vector3d heading = trajectory[i + 1].position_W - trajectory[i].position_W;
            double desired_yaw = 0.0;
            if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
                desired_yaw = std::atan2(heading.y(), heading.x());
            } else {
                desired_yaw = last_yaw;
            }
            trajectory[i].setFromYaw(desired_yaw);
            last_yaw = desired_yaw;
        }
    } else if (policy == YawPolicy::FOLLOW_VELOCITY) {
        const double minVelocityNorm = 0.1;
        for (auto i = 0; i < trajectory.size(); i++) {
            Eigen::Vector3d velocity_xy = trajectory[i].velocity_W;
            velocity_xy.z() = 0;
            if (velocity_xy.norm() > minVelocityNorm) {
                double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            } else {
                double desired_yaw = last_yaw;
                auto j = i + 1;
                while (j < trajectory.size() && velocity_xy.norm() < minVelocityNorm) {
                    velocity_xy = trajectory[j].velocity_W;
                    velocity_xy.z() = 0;
                    j++;
                }
                if (velocity_xy.norm() > minVelocityNorm) {
                    desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                }
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            }
        }
    } else if (policy == YawPolicy::ANTICIPATE_VELOCITY) {
        const double minVelocityNorm = 0.1;
        double initial_yaw = last_yaw;
        for (auto i = trajectory.size(); i > 0; i--) {
            Eigen::Vector3d velocity_xy = trajectory[i].velocity_W;
            velocity_xy.z() = 0;
            if (velocity_xy.norm() > minVelocityNorm) {
                double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            } else {
                double desired_yaw = last_yaw;
                for (auto j = i; j > 0 && velocity_xy.norm() < minVelocityNorm; j--) {
                    velocity_xy = trajectory[j - 1].velocity_W;
                    velocity_xy.z() = 0;
                }
                if (velocity_xy.norm() > minVelocityNorm) {
                    desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                }
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            }
        }
        trajectory[0].setFromYaw(initial_yaw);
    } else if (policy == YawPolicy::CONSTANT) {
        for (auto i = 0; i < trajectory.size(); i++) {
            trajectory[i].setFromYaw(const_yaw_);
        }
    }
}

}  // namespace local_planner