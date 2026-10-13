#include "rig/core.h"
#include <yaml-cpp/yaml.h>
#include <set>
#include <stdexcept>
namespace rig {
  template<class T> void option(const YAML::Node& n,const char* key,T& value) {
    if(n[key]) value=n[key].as<T>();
  }
  static void sensor(const YAML::Node& n,SensorConfig& c) {
    c.id=n["id"].as<int>();
    c.topic=n["topic"].as<std::string>();
    option(n,"enabled",c.enabled);
    option(n,"time_offset",c.time_offset);
    if(n["rotation"]) {
      auto v=n["rotation"].as<std::vector<double>>();
      if(v.size()!=9)throw std::runtime_error("rotation requires 9 values");
      for(int k=0; k<9; ++k)c.extrinsic.R(k/3,k%3)=v[k];
    }
    if(n["translation"]) {
      auto v=n["translation"].as<std::vector<double>>();
      if(v.size()!=3)throw std::runtime_error("translation requires 3 values");
      for(int k=0; k<3; ++k)c.extrinsic.p(k)=v[k];
    }
    if(n["extrinsic_std"]) {
      auto v=n["extrinsic_std"].as<std::vector<double>>();
      if(v.size()!=6)throw std::runtime_error("extrinsic_std requires 6 values");
      for(int k=0; k<6; ++k)c.extrinsic_cov(k,k)=v[k]*v[k];
    }
  }
  SensorRegistry SensorRegistry::load(const std::string& file) {
    auto n=YAML::LoadFile(file);
    SensorRegistry c;
    option(n,"calibration_mode",c.calibration_mode);
    for(const auto& v:n["sensors"]["imus"]) {
      ImuConfig x;
      sensor(v,x);
      option(v,"gyro_noise",x.gyro_noise);
      option(v,"accel_noise",x.accel_noise);
      option(v,"gyro_walk",x.gyro_walk);
      option(v,"accel_walk",x.accel_walk);
      option(v,"gyro_limit",x.gyro_limit);
      option(v,"accel_limit",x.accel_limit);
      c.imus.push_back(x);
    }
    for(const auto& v:n["sensors"]["lidars"]) {
      LidarConfig x;
      sensor(v,x);
      option(v,"max_points",x.max_points);
      option(v,"blind",x.blind);
      option(v,"max_range",x.max_range);
      option(v,"range_noise",x.range_noise);
      option(v,"beam_noise",x.beam_noise);
      c.lidars.push_back(x);
    }
    for(const auto& v:n["sensors"]["cameras"]) {
      CameraConfig x;
      sensor(v,x);
      option(v,"width",x.width);
      option(v,"height",x.height);
      option(v,"scale",x.scale);
      option(v,"fx",x.fx);
      option(v,"fy",x.fy);
      option(v,"cx",x.cx);
      option(v,"cy",x.cy);
      option(v,"model",x.model);
      option(v,"distortion",x.distortion);
      c.cameras.push_back(x);
    }
    auto p=n["processing"];
#define OPT(k) option(p,#k,c.k)
    OPT(threads);
    OPT(max_match_points);
    OPT(visual_points);
    OPT(camera_min);
    OPT(camera_max);
    OPT(patch_size);
    OPT(iterations);
    OPT(max_voxels);
    OPT(max_visual_points);
    OPT(update_period);
    OPT(wait_budget);
    OPT(sensor_timeout);
    OPT(init_duration);
    OPT(voxel_size);
    OPT(map_radius);
    OPT(photo_variance);
    OPT(photo_gate);
    OPT(rigid_rotation_noise);
    OPT(rigid_position_noise);
    OPT(geometry);
    OPT(visual);
    OPT(uncertainty);
    OPT(cross_camera);
#undef OPT
    c.validate();
    return c;
  }
  void SensorRegistry::validate() const {
    if(calibration_mode!="placeholder" && calibration_mode!="validated")throw std::runtime_error("invalid calibration_mode");
    std::set<std::string> topics;
    auto check=[&](const auto& list){
      std::set<int> ids;
      for(const auto& s:list){
        if(s.id<0||!ids.insert(s.id).second||s.topic.empty()||!topics.insert(s.topic).second)throw std::runtime_error("invalid/duplicate sensor id or topic");
        if(!s.extrinsic.R.allFinite()||!s.extrinsic.p.allFinite()||!std::isfinite(s.time_offset)||
        (s.extrinsic.R.transpose()*s.extrinsic.R-Mat3::Identity()).norm()>1e-5||std::abs(s.extrinsic.R.determinant()-1)>1e-5)throw std::runtime_error("invalid T_B_sensor rotation/translation");
      }
    };
    check(imus);
    check(lidars);
    check(cameras);
    if(std::none_of(imus.begin(),imus.end(),[](const auto& c){
      return c.enabled;
    })||std::none_of(lidars.begin(),lidars.end(),[](const auto& c){
      return c.enabled;
    }))throw std::runtime_error("at least one IMU and LiDAR required");
    for(const auto& c:imus) if(c.gyro_noise<=0||c.accel_noise<=0||c.gyro_walk<0||c.accel_walk<0)throw std::runtime_error("invalid IMU noise");
    for(const auto& c:lidars) if(c.max_points<1||c.blind<0||c.max_range<=c.blind||c.range_noise<=0||c.beam_noise<0)throw std::runtime_error("invalid LiDAR parameters");
    for(const auto& c:cameras) if(c.width<32||c.height<32||c.scale<=0||c.scale>1||c.fx<=0||c.fy<=0||
    (c.model!="pinhole"&&c.model!="equidistant")||c.distortion.size()<4)throw std::runtime_error("invalid camera parameters");
    if(threads<1||threads>32||update_period<=0||wait_budget<0||sensor_timeout<=0||init_duration<0||voxel_size<=0||map_radius<=0||iterations<1||max_voxels<1||max_visual_points<1||max_match_points<1||camera_min<0||camera_max<camera_min||visual_points<1||patch_size<2||patch_size%2||photo_variance<=0||photo_gate<=0||rigid_rotation_noise<=0||rigid_position_noise<=0)throw std::runtime_error("invalid processing parameters");
  }
} // namespace rig
