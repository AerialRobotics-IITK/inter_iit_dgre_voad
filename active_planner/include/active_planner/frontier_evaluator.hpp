#pragma once

#include <Eigen/Core>
#include <active_planner/visualizer.hpp>
#include <glog/logging.h>
#include <unordered_map>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>

namespace local_planner {

typedef rviz_visualizer::Visualizer Visualizer;

enum class VoxelState { OCCUPIED, FREE, UNKNOWN };

class FrontierEvaluator {
  public:
    FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void findFrontiers();

    typedef struct Frontier {
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d center;
    } Frontier;
    typedef struct FrontierCenter {
        size_t size;
        Eigen::Vector3d center;
        FrontierCenter(const Frontier& f) {
            size = f.points.size();
            center = f.center;
        }
    } FrontierCenter;

    std::vector<FrontierCenter> getFrontiers();

    VoxelState getVoxelState(const Eigen::Vector3d& point);
    bool getVoxelDistance(const Eigen::Vector3d& point, double& distance);
    bool getVoxelWeight(const Eigen::Vector3d& point, double& weight);

    void visualizeVoxelStates();
    void visualizeFrontierPoints();
    void visualizeFrontierCenters();

  private:
    inline std::string getHash(const Eigen::Vector3d& coord) {
        return std::to_string(int(coord.x() / voxel_size_)) + "," + std::to_string(int(coord.y() / voxel_size_));
    }
    inline bool inHeightRange(const double& height) {
        return ((slice_level_ - height) < lower_range_) && ((height - slice_level_) < upper_range_);
    }

    bool isFrontierVoxel(const Eigen::Vector3d& voxel);

    void findNeighbours(const std::string& key, Frontier& frontier);

    std::vector<Frontier> frontiers_;

    std::vector<Eigen::Vector3d> neighbor_voxels_;
    std::vector<Eigen::Vector3d> planar_neighbor_voxels_;

    Visualizer visualizer_;
    voxblox::EsdfServer esdf_server_;

    std::unordered_map<std::string, Eigen::Vector3d> hash_map_;

    double voxel_size_;
    double block_size_;
    double checking_dist_;
    double occupancy_distance_;
    double slice_level_;
    double upper_range_;
    double lower_range_;
    double min_frontier_size_;
    double robot_radius_;

    bool accurate_frontiers_;
    bool visualize_;

    std::string frame_id_;
};
}  // namespace local_planner