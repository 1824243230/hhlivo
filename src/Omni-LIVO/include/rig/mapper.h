#pragma once
#include "rig/frontend.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rosbag/bag.h>
#include <fstream>
#include <mutex>
#include <memory>
namespace rig {
  struct LidarPacket {
    int id=0;
    double begin=0,end=0;
    std::vector<RigPoint> points;
    size_t consumed=0;
  };
  class RigMapper {
    public:
    RigMapper(SensorRegistry config,const std::string& output,bool online=false);
    void imu(const sensor_msgs::Imu& message,int id,uint64_t arrival);
    void lidar(const sensor_msgs::PointCloud2& message,int id,uint64_t arrival);
    void camera(const sensor_msgs::Image& message,int id,uint64_t arrival);
    void advance(uint64_t arrival,bool eof=false);
    void finish();
    void publishWith(ros::NodeHandle& nh);
    private:
    SensorRegistry config_;
    JointFilter filter_;
    TrajectoryBuffer trajectory_;
    GeometryFrontend geometry_;
    VisualFrontend visual_;
    SensorHealthMonitor health_;
    std::map<int,std::deque<ImuSample>> imus_;
    std::map<int,std::deque<LidarPacket>> lidars_;
    std::map<int,std::deque<CameraObservation>> cameras_;
    std::map<int,double> lidar_coverage_;
    std::map<int,uint64_t> lidar_points_,consumed_,deskewed_,unused_,camera_batches_;
    uint64_t origin_=0,last_arrival_=0,updates_=0,resets_=0,geometry_rows_=0,visual_rows_=0;
    uint64_t geometry_updates_=0,visual_updates_=0;
    double next_=0,last_update_=0,last_publish_=-1,max_compute_=0,total_compute_=0;
    bool online_=false,finished_=false,tracking_=false,ever_initialized_=false;
    std::string output_;
    std::ofstream trajectory_file_,diagnostics_file_;
    rosbag::Bag output_bag_;
    ros::Publisher odom_pub_,cloud_pub_,diagnostics_pub_,path_pub_;
    nav_msgs::Path display_path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::mutex mutex_;
    double time(uint64_t stamp,double offset=0);
    const ImuConfig& imuConfig(int id) const;
    bool sample(int id,double time,ImuSample& output) const;
    bool initialize(double watermark,bool eof);
    bool propagate(double target);
    FusionBatch batch(double target);
    void estimate(FusionBatch& batch);
    void output(const FusionBatch& batch,double computation);
    void reset();
    void summary();
  };
} // namespace rig
