#include <pose_estimator/libpose_estimation.hpp>

namespace iarc2020::pose_estimation {

void PoseEstimator::init() {
    // clang-format off
    cam_matrix_ << 415.21159208661874, 0, 424.5,
                   0, 415.21159208661874, 240.5,
                   0, 0, 1;

    cam_to_quad_ << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    // clang-format on

    img_vec_ = Eigen::Vector3d(0, 0, 1);
    t_cam_ = Eigen::Vector3d(0, 0, 0.02);
}

void PoseEstimator::getDistance(const float& dist) {
    // clang-format off
    scale_up_ << dist, 0, 0,
                 0, dist, 0,
                 0, 0, dist;
    // clang-format on
}

void PoseEstimator::setImgVec(const float& x, const float& y) {
    img_vec_(0) = x;
    img_vec_(1) = y;
}

void PoseEstimator::setQuaternion(const nav_msgs::Odometry& odom) {
    geometry_msgs::Quaternion odom_quat = odom.pose.pose.orientation;
    tf::Quaternion quat = tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
    Eigen::Quaterniond eigen_quat = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
    quad_to_glob_ = eigen_quat.normalized().toRotationMatrix();
}

void PoseEstimator::CamToQuad() {
    quad_coord_ = cam_to_quad_ * scale_up_ * cam_matrix_.inverse() * img_vec_ + t_cam_;
}

void PoseEstimator::QuadToGlob(const nav_msgs::Odometry& odom) {
    glob_coord_ = quad_to_glob_ * quad_coord_;

    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
}

Eigen::Vector3d PoseEstimator::getGlobCoord() {
    return glob_coord_;
}

void PoseEstimator::setCamToQuadMatrix(const std::vector<double>& mat) {
    cam_to_quad_ = Eigen::Matrix3d(mat.data()).transpose();
}

void PoseEstimator::setCamMatrix(const std::vector<double>& mat) {
    cam_matrix_ = Eigen::Matrix3d(mat.data()).transpose();
}

void PoseEstimator::setTCamMatrix(const std::vector<double>& mat) {
    t_cam_ = Eigen::Vector3d(mat.data());
}

void PoseEstimator::setVerbosity(const bool& flag) {
    verbose_ = flag;
}

}  // namespace iarc2020::pose_estimation
