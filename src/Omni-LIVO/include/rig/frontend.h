#pragma once
#include "rig/core.h"
#include "voxel_map.h"
#include "vio.h"
#include <opencv2/core.hpp>
#include <memory>
namespace rig {
  struct RigPoint {
    Vec3 point=Vec3::Zero();
    double time=0;
    int lidar_id=0;
    float intensity=0;
    Mat3 covariance=Mat3::Zero();
  };
  struct CameraObservation {
    int id=0;
    double time=0;
    cv::Mat image;
    Pose relative;
  };
  struct FusionBatch {
    double time=0;
    std::vector<RigPoint> points;
    std::vector<CameraObservation> cameras;
  };
  class GeometryFrontend {
    public:
    explicit GeometryFrontend(const SensorRegistry& config);
    NormalEquations equations(const std::vector<RigPoint>& points,const JointFilter& filter,bool record);
    void insert(const std::vector<RigPoint>& points,const JointFilter& filter,double time);
    void clear();
    std::vector<Vec3> mapPoints() const;
    Vec3 normalAt(const Vec3& world) const;
    std::map<int,uint64_t> matched,inserted;
    double degeneracy=1;
    size_t peak_voxels=0;
    private:
    const SensorRegistry& config_;
    VoxelMapConfig voxel_config_{
    };
    std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> unused_;
    std::unique_ptr<VoxelMapManager> map_;
    VOXEL_LOCATION key(const Vec3& p) const;
  };
  struct PhotoReference {
    int camera=0;
    Vec3 world=Vec3::Zero(),normal=Vec3::UnitZ();
    Pose camera_pose;
    cv::Mat image;
    Eigen::Vector2d pixel=Eigen::Vector2d::Zero();
    double exposure=0,time=0;
  };
  struct CameraContext {
    CameraConfig config;
    std::unique_ptr<vk::AbstractCamera> model;
    std::vector<PhotoReference> references;
    uint64_t images=0,observations=0,rejected=0;
    double variance=100;
  };
  class VisualFrontend {
    public:
    explicit VisualFrontend(const SensorRegistry& config);
    NormalEquations equations(const std::vector<CameraObservation>& images,const JointFilter& filter,bool record);
    void seed(const std::vector<CameraObservation>& images,const std::vector<RigPoint>& points,
    const JointFilter& filter,const GeometryFrontend& geometry);
    void clear();
    std::map<int,CameraContext> cameras;
    uint64_t cross_observations=0;
    static bool pixel(const cv::Mat& image,double u,double v,double& value);
    static Eigen::Matrix<double,2,6> projectionJacobian(const Pose& body,const Pose& extrinsic,const Vec3& world,const vk::AbstractCamera& camera);
    private:
    const SensorRegistry& config_;
    VIOManager warp_;
    // Reuse Omni-LIVO's affine photometric warp implementation.
  };
} // namespace rig
