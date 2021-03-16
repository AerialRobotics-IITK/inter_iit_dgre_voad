#include <explorer/explorer.hpp>

namespace interiit21::explorer {

void ExplorerNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    
    pose_sub_ = nh.subscribe("pose", 10, &ExplorerNode::poseCallback, this);
    centre_sub_ = nh.subscribe("centre_coord", 10, &ExplorerNode::centreCallback, this);
    odom_sub_ = nh.subscribe("odom", 10, &ExplorerNode::odomCallback, this);
    state_sub_ = nh.subscribe("mavros/state", 10, &ExplorerNode::stateCallback, this);

    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("setpoint", 10);

    landing_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ROS_WARN("Waiting for state publish");
    while (ros::ok() && state_sub_.getNumPublishers() == 0) {
        ros::Duration(0.2).sleep();
    }
}

void ExplorerNode::run() {
    takeoff();
}

void ExplorerNode::takeoff() {
    ros::Rate rate(20);
	ROS_INFO("Taking off!");

	ROS_WARN("Waiting for connection..");
	while (ros::ok() && current_state_.connected == false) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to MAVROS..");

	setpoint_.pose.position.x = 0;
	setpoint_.pose.position.y = 0;
	setpoint_.pose.position.z = 2.5;

	for (int i = 100; ros::ok() && i > 0; --i) {
		setpoint_pub_.publish(setpoint_);
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN("Published setpoint 100 times!");

	offb_set_mode_.request.custom_mode = "GUIDED";

	arm_cmd_.request.value = true;

	while (!arm_cmd_.response.success) {
		arming_client_.call(arm_cmd_);
		ROS_INFO_ONCE("Arming...");
	}

	ros::Time last_request = ros::Time::now();

	while (ros::ok()) {
		if (odom_.pose.pose.position.z >= 1.8) {
			offb_set_mode_.request.custom_mode = "AUTO.LOITER";
			set_mode_client_.call(offb_set_mode_);
			ROS_INFO_ONCE("AUTO.LOITER Triggered");
			break;
		}

		if (current_state_.mode != "GUIDED" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if (set_mode_client_.call(offb_set_mode_)) {
				if (offb_set_mode_.response.mode_sent) ROS_WARN("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if (arming_client_.call(arm_cmd_)) {
					if (arm_cmd_.response.success) ROS_WARN("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		setpoint_pub_.publish(setpoint_);
		ros::spinOnce();
		rate.sleep();
	}
}

void ExplorerNode::land() {
    //* Move the quad to the (x,y) of the detected white box
    setpoint_.pose.position.x = pose_.x;
    setpoint_.pose.position.y = pose_.y;
    setpoint_.pose.position.z = odom_.pose.pose.position.z;
    setpoint_pub_.publish(setpoint_);

    landing_client_.call(land_cmd_);
}

void ExplorerNode::centreCallback(const detector_msgs::Centre& msg) {
    centre_ = msg;
}

void ExplorerNode::poseCallback(const detector_msgs::GlobalCoord& msg) {
    pose_ = msg;
}

void ExplorerNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

void ExplorerNode::stateCallback(const mavros_msgs::State& msg) {
    current_state_ = msg;
};

}  // namespace interiit21::explorer