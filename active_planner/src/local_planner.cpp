#include <active_planner/local_planner.hpp>

namespace local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : pathfinder_(nh, nh_private)
    , evaluator_(nh, nh_private)
    , active_(false)
    , exit_(false)
    , verbose_(false)
    , const_yaw_(3.14) {
    // Load Parameters
    nh_private.getParam("visualize_planner", visualize_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("verbose_planner", verbose_);

    odom_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    activate_server_ = nh.advertiseService("activate", &LocalPlanner::activateCallback, this);
    exit_server_ = nh.advertiseService("shutdown", &LocalPlanner::exitCallback, this);

    // Register parameters for visualization
    if (visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("occupied_path");
        visualizer_.createPublisher("free_path");
        visualizer_.createPublisher("trajectory");
    }
}

// Helper function to convert Eigen::Vector3d to geometry point
geometry_msgs::Point LocalPlanner::convertEigenToGeometryMsg(const Eigen::Vector3d& point) {
    geometry_msgs::Point point_msg;

    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();

    return point_msg;
}

// Evaluate frontiers
Eigen::Vector3d LocalPlanner::getBestFrontier() {
    geometry_msgs::Point curr_pos = odometry_.pose.pose.position;
    double yaw = mav_msgs::yawFromQuaternion(mav_msgs::quaternionFromMsg(odometry_.pose.pose.orientation));
    double max_distance = DBL_MIN;
    Eigen::Vector3d best_f = Eigen::Vector3d::Zero();
    for (auto frontier : frontiers_) {
        Eigen::Vector3d vector = (frontier.center - Eigen::Vector3d(curr_pos.x, curr_pos.y, curr_pos.z));
        // Frontier with max distance along yaw is best
        double distance = std::cos(yaw) * vector.x() + std::sin(yaw) * vector.y() - std::fabs(vector.z()); 
        if (distance > max_distance) {
            // Fetch only new frontiers
            if (visited_frontiers_.find(getHash(frontier.center)) == visited_frontiers_.end()) {
                max_distance = distance;
                best_f = frontier.center;
            }
        }
    }

    // Cache all frontiers
    for (auto& frontier : frontiers_) {
        frontier_cache_.push_back(frontier);
    }
    return best_f;
}

// Get frontier from cache if required
Eigen::Vector3d LocalPlanner::getBestFrontierFromCache() {
    geometry_msgs::Point curr_pos = odometry_.pose.pose.position;
    double yaw = mav_msgs::yawFromQuaternion(mav_msgs::quaternionFromMsg(odometry_.pose.pose.orientation));
    double max_distance = DBL_MIN;

    Eigen::Vector3d best_f = Eigen::Vector3d::Zero();

    if (verbose_) {
        ROS_WARN("Fetching frontiers from cache.");
    }

    for (auto frontier : frontier_cache_) {
        Eigen::Vector3d vector = (frontier.center - Eigen::Vector3d(curr_pos.x, curr_pos.y, curr_pos.z));
        double distance = std::cos(yaw) * vector.x() + std::sin(yaw) * vector.y() - std::fabs(vector.z());
        if (distance > max_distance) {
            if (visited_frontiers_.find(getHash(frontier.center)) == visited_frontiers_.end()) {
                max_distance = distance;
                best_f = frontier.center;
            }
        }
    }
    return best_f;
}

// Main function
void LocalPlanner::run() {
    frontiers_.clear();
    trajectory_.clear();
    frontier_path_.clear(); 

    if (waypoint_queue_.empty()) {
        evaluator_.findFrontiers();
        frontiers_ = evaluator_.getFrontiers();

        if (verbose_){
            ROS_INFO_STREAM("Found " << frontiers_.size() << " frontiers");
        }

        Eigen::Vector3d waypoint = getBestFrontier();  // Fetch best frontier
        if(verbose_){
            ROS_WARN_STREAM(waypoint.norm());
        }

        // No good frontier found
        if(waypoint.norm() < voxel_size_){
            if (verbose_) {
                ROS_WARN("No explorable frontier! Spinning around once.");
                visited_frontiers_[getHash(waypoint)] = waypoint;
            }

            geometry_msgs::PoseStamped turn_msg;
            turn_msg.header.stamp = ros::Time::now();

            geometry_msgs::Quaternion orig = odometry_.pose.pose.orientation;
            double orig_z = odometry_.pose.pose.position.z;
            turn_msg.pose.position = odometry_.pose.pose.position;
            double yaw = mav_msgs::yawFromQuaternion(mav_msgs::quaternionFromMsg(orig));
            auto new_yaw = mav_msgs::quaternionFromYaw(yaw + M_PI * 0.8);
            turn_msg.pose.orientation.x = new_yaw.x();
            turn_msg.pose.orientation.y = new_yaw.y();
            turn_msg.pose.orientation.z = new_yaw.z();
            turn_msg.pose.orientation.w = new_yaw.w();
            turn_msg.pose.position.z = 4.0;

            command_pub_.publish(turn_msg);

            ros::Duration(4.0).sleep();
            ros::spinOnce();

            new_yaw = mav_msgs::quaternionFromYaw(yaw - M_PI * 0.8);
            turn_msg.pose.orientation.x = new_yaw.x();
            turn_msg.pose.orientation.y = new_yaw.y();
            turn_msg.pose.orientation.z = new_yaw.z();
            turn_msg.pose.orientation.w = new_yaw.w();
            turn_msg.pose.position.z = 1.0;
            command_pub_.publish(turn_msg);

            ros::Duration(4.0).sleep();
            ros::spinOnce();

            turn_msg.pose.orientation = orig;
            turn_msg.pose.position.z = orig_z;
            command_pub_.publish(turn_msg);
            ros::Duration(2.0).sleep();
            ros::spinOnce();

        } else {
            
            if (verbose_) {
                ROS_INFO_STREAM("Pursuing new frontier: \n" << waypoint);
            }
            waypoint_queue_.push(waypoint);
        }
    } else {  // Process first waypoint
        Eigen::Vector3d waypt = waypoint_queue_.top();
        waypoint_queue_.pop();

        ros::spinOnce();
        mav_msgs::EigenOdometry start_odom;
        mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

        frontier_path_.clear();
        trajectory_.clear();

        // Find path to waypoint
        pathfinder_.findPath(start_odom.position_W, waypt);
        frontier_path_ = pathfinder_.getPath();
        trajectory_ = generateTrajectoryThroughWaypoints(frontier_path_);

        if (verbose_) {
            ROS_INFO_STREAM("Generated " << trajectory_.size() << " waypoints");
        }

        // Mark all frontiers as seen
        visited_frontiers_[getHash(waypt)] = waypt;
        for (auto frontier : frontiers_) {
            visited_frontiers_[getHash(frontier.center)] = frontier.center;
        }

        if (trajectory_.empty()) {
            if(verbose_){
                ROS_INFO("Current frontier not feasible.");
                return;
            }
        }

        // Track trajectory to current waypoint
        ros::Rate pub_rate(20);
        for (auto i = 0; i < trajectory_.size(); i++) {
            auto target = trajectory_[i];
            geometry_msgs::PoseStamped setpt;

            double curr_yaw = mav_msgs::yawFromQuaternion(target.orientation_W_B);
            target.orientation_W_B = mav_msgs::quaternionFromYaw(curr_yaw - M_PI * 0.85);  // Account for initila yaw offset

            mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(target, &setpt);
            command_pub_.publish(setpt);

            if (verbose_) {
                ROS_INFO("Published next waypoint!");
            }

            ros::spinOnce();
            bool feasible = true;

            // Wait to reach to waypoint
            while (ros::ok() && (norm(odometry_.pose.pose.position, convertEigenToGeometryMsg(target.position_W)) > robot_radius_) && !exit_) {
                ros::spinOnce();
                mav_msgs::EigenOdometry start_odom;
                mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

                if (checkForAbort(i, trajectory_)) {
                    if (verbose_) {
                        ROS_WARN("Aborting current trajectory...");
                    }

                    // Check if replan is possible
                    pathfinder_.findPath(start_odom.position_W, waypt);
                    if (pathfinder_.getPath().size() > 0) {
                        waypoint_queue_.push(waypt);
                    }

                    geometry_msgs::PoseStamped stop_pt;
                    ros::spinOnce();

                    stop_pt.pose = odometry_.pose.pose;
                    stop_pt.header.stamp = ros::Time::now();
                    command_pub_.publish(stop_pt);

                    feasible = false;
                    trajectory_.clear();
                    frontier_path_.clear();

                    // Fetch a new waypoint right here
                    Eigen::Vector3d waypoint = getBestFrontier();
                    if(waypoint.norm() > voxel_size_){
                        if (verbose_) {
                            ROS_INFO_STREAM("Pursuing new frontier: \n" << waypoint);
                        }
                        waypoint_queue_.push(waypoint);
                    }

                    break;
                }

                pub_rate.sleep();
            }

            // Discard current frontier
            if (!feasible) {
                trajectory_.clear();
                frontier_path_.clear();
                ros::spinOnce();
                
                if(verbose_){
                    ROS_INFO("Aborted. Looking for next frontier");
                }
                
                visited_frontiers_[getHash(waypt)] = waypt;
                break;
            }
        }

        if(verbose_){
            ROS_INFO_STREAM("Looking for next frontier");
        }
    }
}

// Check if current trajectory gets an obstacle in between
bool LocalPlanner::checkForAbort(const uint i, Trajectory& trajectory){
    bool need_abort = false;
    uint occupied = 0;

    std::vector<Eigen::Vector3d> free_points, occ_points;

    // Lookahead for 4 waypoints
    double distance = 0.0;
    for (auto j = i; j < (i+4) && j < trajectory.size(); j++) {
        if (pathfinder_.getMapDistance(trajectory[j].position_W, distance) && distance < robot_radius_) {
            occupied++;
            if (visualize_) {
                occ_points.push_back(trajectory[j].position_W);
            }
        } else if (visualize_) {
            free_points.push_back(trajectory[j].position_W);
        }
    }

    need_abort = (occupied > 0);  // At least one point is occupied - abort

    if (visualize_) {
        visualizer_.visualizePoints("occupied_path", occ_points, "map", Visualizer::ColorType::RED, 1);
        visualizer_.visualizePoints("free_path", free_points, "map", Visualizer::ColorType::GREEN, 0.5);
    }

    return need_abort;
}

// Build trajectory
Trajectory LocalPlanner::generateTrajectoryThroughWaypoints(const Path& waypoints) {
    Trajectory traj;
    if (waypoints.empty()) {
        return traj;
    }

    for (auto i = 0; i < waypoints.size(); i++) {
        mav_msgs::EigenTrajectoryPoint traj_pt;
        traj_pt.position_W = waypoints[i];
        traj.push_back(traj_pt);
    }

    applyYawToTrajectory(traj);
    if (visualize_) {
        visualizer_.visualizeTrajectory("trajectory", traj, "map", Visualizer::ColorType::BLACK, 0.2);
    }
    return traj;
}

// Apply yaw to trajectory waypoints as per policy
void LocalPlanner::applyYawToTrajectory(Trajectory& trajectory, const YawPolicy& policy) {
    if (trajectory.size() < 2) {
        return;
    }
    double last_yaw = mav_msgs::yawFromQuaternion(mav_msgs::quaternionFromMsg(odometry_.pose.pose.orientation));

    if (policy == YawPolicy::POINT_FACING) {
        for (auto i = 0; i < trajectory.size() - 1; i++) {
            Eigen::Vector3d heading = (trajectory[i+1].position_W - trajectory[i].position_W).normalized();
            Eigen::Vector3d plane_heading(heading.x(), heading.y(), 0);
            double desired_yaw = 0.0;
            if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
                desired_yaw = std::acos(heading.dot(plane_heading.normalized()));
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