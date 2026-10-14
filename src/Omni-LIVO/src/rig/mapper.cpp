#include "rig/mapper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <filesystem>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
namespace rig {
  namespace {
    template<class List> const SensorConfig& findSensor(const List& list,int id) {
      for(const auto& c:list)if(c.id==id)return c;
      throw std::runtime_error("unknown sensor id");
    }
    sensor_msgs::PointCloud2 cloud(const std::vector<Vec3>& points,uint64_t stamp) {
      sensor_msgs::PointCloud2 msg;
      msg.header.stamp.fromNSec(stamp);
      msg.header.frame_id="rig_world";
      sensor_msgs::PointCloud2Modifier mod(msg);
      mod.setPointCloud2FieldsByString(1,"xyz");
      mod.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
      for(const auto& p:points){
        *x=p.x();
        *y=p.y();
        *z=p.z();
        ++x;
        ++y;
        ++z;
      }return msg;
    }
  }
  RigMapper::RigMapper(SensorRegistry config,const std::string& output,bool online):config_(std::move(config)),filter_(config_),geometry_(config_),visual_(config_),online_(online),output_(output) {
    config_.validate();
    std::filesystem::create_directories(output_);
    if(std::filesystem::exists(output_+"/trajectory.txt"))throw std::runtime_error("output already exists; choose a fresh directory");
    trajectory_file_.open(output_+"/trajectory.txt");
    diagnostics_file_.open(output_+"/frames.csv");
    if(!trajectory_file_||!diagnostics_file_)throw std::runtime_error("cannot open output files");
    diagnostics_file_ << "time,points,cameras,active_imus,state_dimension,geometry_rows,visual_rows,degeneracy,compute_ms\n";
    output_bag_.open(output_+"/estimates.bag",rosbag::bagmode::Write);
    std::cout << "rig_mapping calibration_mode="
    << config_.calibration_mode
    << " imus="
    << config_.imus.size()
    << " lidars="
    << config_.lidars.size()
    << " cameras="
    << config_.cameras.size()
    << std::endl;
  }
  void RigMapper::publishWith(ros::NodeHandle& nh) {
    odom_pub_=nh.advertise<nav_msgs::Odometry>("/rig/odometry",10);
    cloud_pub_=nh.advertise<sensor_msgs::PointCloud2>("/rig/local_map",1,true);
    diagnostics_pub_=nh.advertise<std_msgs::String>("/rig/diagnostics",10);
    path_pub_=nh.advertise<nav_msgs::Path>("/rig/path",1,true);
    display_path_.header.frame_id="rig_world";
    // Construct ROS broadcasters only after ros::init, in the online entry point.
    tf_broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>();
    static_tf_broadcaster_=std::make_unique<tf2_ros::StaticTransformBroadcaster>();
    std::vector<geometry_msgs::TransformStamped> transforms;
    const auto append=[&](const auto& sensors,const std::string& kind){
      for(const auto& c:sensors)if(c.enabled){
        geometry_msgs::TransformStamped transform;
        transform.header.stamp=ros::Time::now();
        transform.header.frame_id="rig_body";
        transform.child_frame_id=kind+"_"+std::to_string(c.id);
        transform.transform.translation.x=c.extrinsic.p.x();
        transform.transform.translation.y=c.extrinsic.p.y();
        transform.transform.translation.z=c.extrinsic.p.z();
        Eigen::Quaterniond rotation(c.extrinsic.R);
        rotation.normalize();
        transform.transform.rotation.x=rotation.x();
        transform.transform.rotation.y=rotation.y();
        transform.transform.rotation.z=rotation.z();
        transform.transform.rotation.w=rotation.w();
        transforms.push_back(transform);
      }
    };
    append(config_.imus,"imu");
    append(config_.lidars,"lidar");
    append(config_.cameras,"camera");
    static_tf_broadcaster_->sendTransform(transforms);
  }
  double RigMapper::time(uint64_t stamp,double offset) {
    if(!origin_)origin_=stamp;
    return double(int64_t(stamp)-int64_t(origin_))*1e-9+offset;
  }
  const ImuConfig& RigMapper::imuConfig(int id) const {
    for(const auto& c:config_.imus)if(c.id==id)return c;
    throw std::runtime_error("unknown IMU");
  }
  void RigMapper::imu(const sensor_msgs::Imu& m,int id,uint64_t arrival) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& c=imuConfig(id);
    if(!c.enabled)return;
    if(last_arrival_&&arrival+1000000000ULL<last_arrival_)reset();
    last_arrival_=arrival;
    ImuSample s{
      id,time(m.header.stamp.toNSec(),c.time_offset),{
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z
      },{
        m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z
      }
    };
    bool valid=s.accel.allFinite()&&s.gyro.allFinite()&&s.accel.norm()<=c.accel_limit&&s.gyro.norm()<=c.gyro_limit;
    if(!health_.observe(SensorHealthMonitor::key("imu",id),s.time,time(arrival),valid))return;
    auto& q=imus_[id];
    q.push_back(s);
    while(q.size()>1000){
      q.pop_front();
      ++health_.sensors[SensorHealthMonitor::key("imu",id)].stale;
    }
  }
  void RigMapper::camera(const sensor_msgs::Image& m,int id,uint64_t arrival) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& c=findSensor(config_.cameras,id);
    if(!c.enabled)return;
    if(last_arrival_&&arrival+1000000000ULL<last_arrival_)reset();
    last_arrival_=arrival;
    double t=time(m.header.stamp.toNSec(),c.time_offset);
    bool valid=m.width>0&&m.height>0&&!m.data.empty();
    if(!health_.observe(SensorHealthMonitor::key("camera",id),t,time(arrival),valid))return;
    const auto& ctx=visual_.cameras.at(id);
    cv::Mat gray=cv_bridge::toCvCopy(m,"mono8")->image;
    cv::Mat resized;
    cv::resize(gray,resized,cv::Size(ctx.model->width(),ctx.model->height()),0,0,cv::INTER_AREA);
    cameras_[id].push_back({
      id,t,resized,Pose()
    });
    while(cameras_[id].size()>10){
      cameras_[id].pop_front();
      ++health_.sensors[SensorHealthMonitor::key("camera",id)].stale;
    }
  }
  void RigMapper::lidar(const sensor_msgs::PointCloud2& m,int id,uint64_t arrival) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& sensor=findSensor(config_.lidars,id);
    if(!sensor.enabled)return;
    if(last_arrival_&&arrival+1000000000ULL<last_arrival_)reset();
    last_arrival_=arrival;
    const LidarConfig* config=nullptr;
    for(const auto& c:config_.lidars)if(c.id==id)config=&c;
    double t=time(m.header.stamp.toNSec(),sensor.time_offset);
    bool valid=!m.is_bigendian&&m.height==1&&m.point_step>=24&&m.data.size()==size_t(m.row_step)*m.height&&m.row_step>=m.width*m.point_step;
    std::map<std::string,uint32_t> fields;
    for(const auto& f:m.fields)fields[f.name]=f.offset;
    for(const auto& name:{
      "x","y","z","intensity","offset_time","ring"
    })if(!fields.count(name))valid=false;
    for(const auto& f:m.fields)if(f.offset+(f.name=="ring"?2:4)>m.point_step)valid=false;
    if(!health_.observe(SensorHealthMonitor::key("lidar",id),t,time(arrival),valid))return;
    LidarPacket packet;
    packet.id=id;
    packet.begin=t;
    packet.end=t;
    packet.points.reserve(config->max_points);
    size_t stride=std::max(size_t(1),size_t(m.width)/size_t(config->max_points));
    std::unordered_set<VOXEL_LOCATION> spatial;
    for(size_t i=0; i<m.width; ++i) {
      const uint8_t* data=m.data.data()+i*m.point_step;
      uint32_t ns;
      std::memcpy(&ns,data+fields["offset_time"],4);
      double pt=t+ns*1e-9;
      packet.end=std::max(packet.end,pt);
      if(i%stride || packet.points.size()>=size_t(config->max_points))continue;
      float x,y,z,intensity;
      std::memcpy(&x,data+fields["x"],4);
      std::memcpy(&y,data+fields["y"],4);
      std::memcpy(&z,data+fields["z"],4);
      std::memcpy(&intensity,data+fields["intensity"],4);
      Vec3 p(x,y,z);
      if(!p.allFinite()||p.norm()<config->blind||p.norm()>config->max_range)continue;
      VOXEL_LOCATION key(int64_t(std::floor(x/0.1)),int64_t(std::floor(y/0.1)),int64_t(std::floor(z/0.1)));
      if(!spatial.insert(key).second)continue;
      packet.points.push_back({
        p,pt,id,intensity,Mat3::Zero()
      });
    }
    std::stable_sort(packet.points.begin(),packet.points.end(),[](const auto& a,const auto& b){
      return a.time<b.time;
    });
    lidar_points_[id]+=packet.points.size();
    lidar_coverage_[id]=packet.end;
    auto& q=lidars_[id];
    q.push_back(std::move(packet));
    while(q.size()>20){
      unused_[id]+=q.front().points.size()-q.front().consumed;
      q.pop_front();
    }
  }
  bool RigMapper::sample(int id,double t,ImuSample& out) const {
    auto found=imus_.find(id);
    if(found==imus_.end()||found->second.empty())return false;
    const auto& q=found->second;
    if(t<q.front().time-1e-7||t>q.back().time+1e-7)return false;
    auto b=std::lower_bound(q.begin(),q.end(),t,[](const auto& a,double t){
      return a.time<t;
    });
    if(b==q.end()){
      out=q.back();
      return true;
    }if(b==q.begin()){
      out=*b;
      return true;
    }
    auto a=b-1;
    double gap=b->time-a->time;
    if(gap>0.05)return false;
    double u=(t-a->time)/gap;
    out={
      id,t,(1-u)*a->accel+u*b->accel,(1-u)*a->gyro+u*b->gyro
    };
    return true;
  }
  bool RigMapper::initialize(double watermark,bool eof) {
    if(imus_.empty())return false;
    double start=1e30;
    for(const auto& p:imus_)if(!p.second.empty())start=std::min(start,p.second.front().time);
    double end=start+config_.init_duration;
    if(watermark<end&&!eof)return false;
    std::map<int,ImuSample> mean;
    bool all=true;
    for(const auto& c:config_.imus)if(c.enabled){
      auto it=imus_.find(c.id);
      if(it==imus_.end()||it->second.empty()||it->second.back().time<end){
        all=false;
        continue;
      }
      ImuSample m;
      m.id=c.id;
      int count=0;
      for(const auto& s:it->second)if(s.time<=end){
        m.accel+=s.accel;
        m.gyro+=s.gyro;
        ++count;
      }
      if(count>=3){
        m.accel/=count;
        m.gyro/=count;
        mean[c.id]=m;
      }
    }
    if((!all&&watermark<end+config_.wait_budget&&!eof)||mean.empty())return false;
    filter_.initialize(end,mean);
    trajectory_.append({
      end,filter_.bodyPose(),filter_.bodyVelocity(),filter_.bodyCovariance()
    });
    next_=end+config_.update_period;
    last_update_=end;
    tracking_=true;
    ever_initialized_=true;
    std::cout << "Initialized at "
    << end
    << " s, state_dimension="
    << filter_.state.layout.size()
    << std::endl;
    return true;
  }
  bool RigMapper::propagate(double target) {
    while(!trajectory_.knots.empty()&&trajectory_.knots.back().time>filter_.state.time+1e-8)trajectory_.knots.pop_back();
    while(filter_.state.time<target-1e-8) {
      double end=std::min(target,filter_.state.time+0.01),mid=(end+filter_.state.time)/2;
      std::map<int,ImuSample> input;
      std::vector<int> missing;
      for(int id:filter_.state.layout.imu_ids){
        ImuSample s;
        if(!sample(id,mid,s))missing.push_back(id);
        else input[id]=s;
      }
      if(input.empty())return false;
      for(int id:missing){
        auto& h=health_.sensors[SensorHealthMonitor::key("imu",id)];
        h.isolated=true;
        h.consecutive_good=0;
        filter_.removeImu(id);
      }
      filter_.propagate(end,input);
      filter_.constrainRig();
      trajectory_.append({
        end,filter_.bodyPose(),filter_.bodyVelocity(),filter_.bodyCovariance()
      });
    }
    JointFilter lookahead=filter_;
    for(int k=0; k<2; ++k){
      double end=lookahead.state.time+0.01;
      std::map<int,ImuSample> input;
      bool ready=true;
      for(int id:lookahead.state.layout.imu_ids){
        ImuSample s;
        if(!sample(id,end-0.005,s)){
          ready=false;
          break;
        }input[id]=s;
      }
      if(!ready)break;
      lookahead.propagate(end,input);
      trajectory_.append({
        end,lookahead.bodyPose(),lookahead.bodyVelocity(),lookahead.bodyCovariance()
      });
    }
    return true;
  }
  FusionBatch RigMapper::batch(double target) {
    FusionBatch batch;
    batch.time=target;
    Pose reference;
    trajectory_.query(target,reference);
    for(const auto& c:config_.lidars)if(c.enabled){
      auto& q=lidars_[c.id];
      int count=0;
      while(!q.empty()&&q.front().begin<=target){
        auto& p=q.front();
        while(p.consumed<p.points.size()&&p.points[p.consumed].time<=target+1e-9){
          auto pt=p.points[p.consumed++];
          ++consumed_[c.id];
          if(pt.time<last_update_-1e-8||count>=c.max_points||batch.points.size()>=size_t(config_.max_match_points)){
            ++unused_[c.id];
            continue;
          }
          Pose acquisition;
          if(!trajectory_.query(pt.time,acquisition)){
            ++unused_[c.id];
            continue;
          }
          Pose transform=reference.inverse()*acquisition*c.extrinsic;
          Vec3 original=pt.point;
          pt.point=transform.apply(original);
          if(config_.uncertainty)pt.covariance=transform.R*trajectory_.pointCovariance(original,target-pt.time,c)*transform.R.transpose();
          else pt.covariance=Mat3::Identity()*c.range_noise*c.range_noise;
          batch.points.push_back(pt);
          ++deskewed_[c.id];
          ++count;
        }
        if(p.consumed==p.points.size()&&p.end<=target+1e-8)q.pop_front();
        else break;
      }
    }
    for(const auto& c:config_.cameras)if(c.enabled){
      auto& q=cameras_[c.id];
      CameraObservation best;
      bool got=false;
      while(!q.empty()&&q.front().time<=target+0.001){
        auto obs=q.front();
        q.pop_front();
        if(obs.time>last_update_&&target-obs.time<=0.075){
          best=std::move(obs);
          got=true;
        }
      }
      if(got){
        Pose captured;
        if(trajectory_.query(best.time,captured)){
          best.relative=reference.inverse()*captured;
          batch.cameras.push_back(std::move(best));
          ++camera_batches_[c.id];
        }
      }
    }return batch;
  }
  void RigMapper::estimate(FusionBatch& batch) {
    auto update=[&](bool visual){
      JointState prior=filter_.state;
      Eigen::MatrixXd post;
      Eigen::VectorXd d;
      bool accepted=false;
      for(int i=0; i<config_.iterations; ++i){
        auto eq=visual?visual_.equations(batch.cameras,filter_,false):geometry_.equations(batch.points,filter_,false);
        if(eq.rows<6)break;
        JointState before=filter_.state;
        Eigen::MatrixXd candidate;
        Eigen::VectorXd step;
        if(!filter_.iteration(prior,eq,candidate,step))break;
        auto after=visual?visual_.equations(batch.cameras,filter_,false):geometry_.equations(batch.points,filter_,false);
        if(after.rows<6||after.error/std::max(size_t(1),after.rows)>eq.error/eq.rows*1.1+0.01){
          filter_.state=before;
          break;
        }
        post=candidate;
        d=step;
        accepted=true;
        if(step.norm()<1e-5)break;
      }
      if(accepted){
        filter_.commit(post,filter_.state.difference(prior));
        auto recorded=visual?visual_.equations(batch.cameras,filter_,true):geometry_.equations(batch.points,filter_,true);
        if(visual){
          visual_rows_+=recorded.rows;
          ++visual_updates_;
        }else{
          geometry_rows_+=recorded.rows;
          ++geometry_updates_;
        }
      }
    };
    Pose before=filter_.bodyPose();
    if(config_.geometry&&!batch.points.empty())update(false);
    if(config_.visual&&!batch.cameras.empty())update(true);
    trajectory_.correctFrom(last_update_,before,filter_.bodyPose());
    if(config_.geometry)geometry_.insert(batch.points,filter_,batch.time);
    if(config_.visual)visual_.seed(batch.cameras,batch.points,filter_,geometry_);
    if(!filter_.state.finite())throw std::runtime_error("non-finite joint estimator state");
  }
  void RigMapper::output(const FusionBatch& batch,double computation) {
    if(batch.time<=last_publish_)throw std::runtime_error("non-monotonic odometry");
    last_publish_=batch.time;
    ++updates_;
    Pose b=filter_.bodyPose();
    Vec3 v=filter_.bodyVelocity();
    Eigen::Quaterniond q(b.R);
    uint64_t ns=origin_+uint64_t(std::llround(batch.time*1e9));
    trajectory_file_ << std::fixed
    << std::setprecision(9)
    << ns/1e9
    << " "
    << b.p.transpose()
    << " "
    << q.x()
    << " "
    << q.y()
    << " "
    << q.z()
    << " "
    << q.w()
    << "\n";
    nav_msgs::Odometry odom;
    odom.header.stamp.fromNSec(ns);
    odom.header.frame_id="rig_world";
    odom.child_frame_id="rig_body";
    odom.pose.pose.position.x=b.p.x();
    odom.pose.pose.position.y=b.p.y();
    odom.pose.pose.position.z=b.p.z();
    odom.pose.pose.orientation.x=q.x();
    odom.pose.pose.orientation.y=q.y();
    odom.pose.pose.orientation.z=q.z();
    odom.pose.pose.orientation.w=q.w();
    Mat6 basis=Mat6::Identity();
    basis.topLeftCorner<3,3>()=b.R;
    Mat6 cov=basis*filter_.bodyCovariance()*basis.transpose();
    for(int i=0; i<6; ++i)for(int j=0; j<6; ++j)odom.pose.covariance[i*6+j]=cov((i<3?i+3:i-3),(j<3?j+3:j-3));
    Vec3 vb=b.R.transpose()*v;
    odom.twist.twist.linear.x=vb.x();
    odom.twist.twist.linear.y=vb.y();
    odom.twist.twist.linear.z=vb.z();
    Vec3 angular=filter_.bodyAngularVelocity();
    odom.twist.twist.angular.x=angular.x();
    odom.twist.twist.angular.y=angular.y();
    odom.twist.twist.angular.z=angular.z();
    Mat6 twist=filter_.bodyTwistCovariance();
    for(int i=0; i<6; ++i)for(int j=0; j<6; ++j)odom.twist.covariance[i*6+j]=twist(i,j);
    output_bag_.write("/rig/odometry",odom.header.stamp,odom);
    if(online_&&ros::ok()){
      odom_pub_.publish(odom);
      geometry_msgs::TransformStamped transform;
      transform.header=odom.header;
      transform.child_frame_id=odom.child_frame_id;
      transform.transform.translation.x=b.p.x();
      transform.transform.translation.y=b.p.y();
      transform.transform.translation.z=b.p.z();
      transform.transform.rotation=odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
      geometry_msgs::PoseStamped pose;
      pose.header=odom.header;
      pose.pose=odom.pose.pose;
      display_path_.header=odom.header;
      // Keep visualization memory and publication bandwidth bounded on long runs.
      if(display_path_.poses.size()>=2000)display_path_.poses.erase(display_path_.poses.begin());
      display_path_.poses.push_back(pose);
      path_pub_.publish(display_path_);
    }
    diagnostics_file_ << std::setprecision(9)
    << batch.time
    << ","
    << batch.points.size()
    << ","
    << batch.cameras.size()
    << ","
    << filter_.state.imus.size()
    << ","
    << filter_.state.layout.size()
    << ","
    << geometry_rows_
    << ","
    << visual_rows_
    << ","
    << geometry_.degeneracy
    << ","
    << computation
    << "\n";
    total_compute_+=computation;
    max_compute_=std::max(max_compute_,computation);
    if(online_&&ros::ok()&&(updates_%10==0||display_path_.poses.size()==1)){
      cloud_pub_.publish(cloud(geometry_.mapPoints(),ns));
      std_msgs::String s;
      s.data="tracking; state_dimension="+std::to_string(filter_.state.layout.size());
      diagnostics_pub_.publish(s);
    }
    if(updates_%20==0)std::cout << "frame="
    << updates_
    << " time="
    << batch.time
    << " points="
    << batch.points.size()
    << " cameras="
    << batch.cameras.size()
    << " lio_rows="
    << geometry_rows_
    << " vio_rows="
    << visual_rows_
    << " ms="
    << computation
    << std::endl;
  }
  void RigMapper::advance(uint64_t arrival,bool eof) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!origin_)return;
    if(last_arrival_&&arrival+1000000000ULL<last_arrival_)reset();
    last_arrival_=arrival;
    double watermark=time(arrival);
    for(auto& p:health_.sensors)if(p.second.last_time>=0&&watermark-p.second.last_time>config_.sensor_timeout&&!p.second.isolated){
      p.second.isolated=true;
      p.second.consecutive_good=0;
    }
    if(!filter_.state.initialized){
      if(ever_initialized_ || !initialize(watermark,eof))return;
    }
    while(next_<=watermark+1e-8){
      std::vector<int> remove;
      for(int id:filter_.state.layout.imu_ids){
        auto& h=health_.sensors[SensorHealthMonitor::key("imu",id)];
        if(h.isolated||next_-h.last_time>config_.sensor_timeout)remove.push_back(id);
      }
      for(int id:remove){
        auto& h=health_.sensors[SensorHealthMonitor::key("imu",id)];
        h.isolated=true;
        h.consecutive_good=0;
        filter_.removeImu(id);
      }
      if(!filter_.state.initialized){
        tracking_=false;
        break;
      }
      for(const auto& c:config_.imus)if(c.enabled&&!filter_.state.imus.count(c.id)){
        auto& h=health_.sensors[SensorHealthMonitor::key("imu",c.id)];
        ImuSample s;
        if(!h.isolated&&h.consecutive_good>=5&&sample(c.id,filter_.state.time+0.005,s))filter_.restoreImu(c.id);
      }
      bool ready=true;
      double cover=1e30;
      for(int id:filter_.state.layout.imu_ids){
        auto& q=imus_[id];
        if(q.empty()){
          ready=false;
          break;
        }cover=std::min(cover,q.back().time);
      }
      if(cover+1e-8<next_+(eof?0:0.02))ready=false;
      bool lidar_ready=true;
      for(const auto& c:config_.lidars)if(c.enabled&&!health_.sensors[SensorHealthMonitor::key("lidar",c.id)].isolated&&lidar_coverage_[c.id]<next_)lidar_ready=false;
      if(!ready){
        if(watermark<next_+config_.wait_budget&&!eof)break;
        double required=next_+(eof?0:0.02);
        bool any=false;
        for(int id:filter_.state.layout.imu_ids)if(!imus_[id].empty()&&imus_[id].back().time+1e-8>=required)any=true;
        if(any){
          auto ids=filter_.state.layout.imu_ids;
          for(int id:ids)if(imus_[id].empty()||imus_[id].back().time+1e-8<required){
            auto& h=health_.sensors[SensorHealthMonitor::key("imu",id)];
            h.isolated=true;
            h.consecutive_good=0;
            filter_.removeImu(id);
          }continue;
        }
        if(eof)break;
        next_+=config_.update_period;
        tracking_=false;
        continue;
      }
      if(!lidar_ready&&watermark<next_+config_.wait_budget&&!eof)break;
      auto begin=std::chrono::steady_clock::now();
      if(!propagate(next_)){
        next_+=config_.update_period;
        tracking_=false;
        continue;
      }
      auto group=batch(next_);
      estimate(group);
      tracking_=true;
      double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      output(group,ms);
      last_update_=next_;
      next_+=config_.update_period;
      for(auto& p:imus_)while(p.second.size()>2&&p.second[1].time<filter_.state.time-0.05)p.second.pop_front();
    }
  }
  void RigMapper::reset(){
    // A new clock epoch is a separate trajectory, never silently stitched to the old map.
    ++resets_;
    trajectory_file_.close();
    trajectory_file_.open(output_+"/trajectory_"+std::to_string(resets_)+".txt");
    output_bag_.close();
    output_bag_.open(output_+"/estimates_"+std::to_string(resets_)+".bag",rosbag::bagmode::Write);
    geometry_.clear();
    visual_.clear();
    trajectory_.clear();
    display_path_.poses.clear();
    if(online_&&ros::ok()){
      display_path_.header.stamp=ros::Time::now();
      path_pub_.publish(display_path_);
      cloud_pub_.publish(cloud({},display_path_.header.stamp.toNSec()));
    }
    imus_.clear();
    lidars_.clear();
    cameras_.clear();
    lidar_coverage_.clear();
    filter_.state=JointState();
    health_.sensors.clear();
    last_publish_=-1;
    tracking_=false;
    ever_initialized_=false;
    origin_=0;
    last_arrival_=0;
  }
  void RigMapper::summary() {
    struct rusage usage{
    };
    getrusage(RUSAGE_SELF,&usage);
    double minimum_covariance=0;
    if(filter_.state.covariance.size()){
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(filter_.state.covariance);
      minimum_covariance=eigen.eigenvalues().minCoeff();
    }
    std::ofstream f(output_+"/summary.json");
    f << "{\n\"calibration_mode\":\""
    << config_.calibration_mode
    << "\",\n\"updates\":"
    << updates_
    << ",\n\"tracking\":"
    << (tracking_?"true":"false")
    << ",\n\"state_dimension\":"
    << filter_.state.layout.size()
    << ",\n\"geometry_updates\":"
    << geometry_updates_
    << ",\n\"visual_updates\":"
    << visual_updates_
    << ",\n\"geometry_rows\":"
    << geometry_rows_
    << ",\n\"visual_rows\":"
    << visual_rows_
    << ",\n\"rigid_updates\":"
    << filter_.rigid_updates
    << ",\n\"rejected_updates\":"
    << filter_.rejected_updates
    << ",\n\"cross_camera_observations\":"
    << visual_.cross_observations
    << ",\n\"spline_queries\":"
    << trajectory_.spline_queries
    << ",\n\"fallback_queries\":"
    << trajectory_.fallback_queries
    << ",\n\"rejected_trajectory_queries\":"
    << trajectory_.rejected_queries
    << ",\n\"resets\":"
    << resets_
    << ",\n\"map_points\":"
    << geometry_.mapPoints().size()
    << ",\n\"peak_voxels\":"
    << geometry_.peak_voxels
    << ",\n\"peak_rss_mb\":"
    << usage.ru_maxrss/1024.0
    << ",\n\"minimum_covariance_eigenvalue\":"
    << minimum_covariance
    << ",\n\"mean_compute_ms\":"
    << total_compute_/std::max(uint64_t(1),updates_)
    << ",\n\"max_compute_ms\":"
    << max_compute_
    << ",\n\"sensors\":{\n";
    bool first=true;
    for(const auto& p:health_.sensors){
      if(!first)f << ",\n";
      first=false;
      const auto& h=p.second;
      f << "\""
      << p.first
      << "\":{\"received\":"
      << h.received
      << ",\"rejected\":"
      << h.rejected
      << ",\"duplicates\":"
      << h.duplicates
      << ",\"stale\":"
      << h.stale
      << ",\"recoveries\":"
      << h.recoveries
      << ",\"isolated\":"
      << (h.isolated?"true":"false")
      << "}";
    }
    f << "\n},\n\"lidars\":{";
    first=true;
    for(const auto& c:config_.lidars){
      if(!first)f << ",";
      first=false;
      int id=c.id;
      f << "\""
      << id
      << "\":{\"sampled\":"
      << lidar_points_[id]
      << ",\"consumed\":"
      << consumed_[id]
      << ",\"deskewed\":"
      << deskewed_[id]
      << ",\"unused\":"
      << unused_[id]
      << ",\"matched\":"
      << geometry_.matched[id]
      << ",\"inserted\":"
      << geometry_.inserted[id]
      << "}";
    }
    f << "},\n\"imus\":{";
    first=true;
    for(const auto& c:config_.imus){
      if(!first)f << ",";
      first=false;
      f << "\""
      << c.id
      << "\":{\"propagations\":"
      << filter_.propagated[c.id]
      << "}";
    }
    f << "},\n\"cameras\":{";
    first=true;
    for(const auto& p:visual_.cameras){
      if(!first)f << ",";
      first=false;
      f << "\""
      << p.first
      << "\":{\"batches\":"
      << camera_batches_[p.first]
      << ",\"images\":"
      << p.second.images
      << ",\"observations\":"
      << p.second.observations
      << ",\"rejected\":"
      << p.second.rejected
      << ",\"references\":"
      << p.second.references.size()
      << "}";
    }f << "}\n}\n";
  }
  void RigMapper::finish(){
    uint64_t final_time=0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(finished_)return;
      final_time=last_arrival_;
      // rosbag --clock may stop just before the final message header time.
      // Use received measurements to close that sub-millisecond clock gap,
      // with the same bounded wait budget used during normal scheduling.
      double measured=0;
      for(const auto& p:imus_)if(!p.second.empty())measured=std::max(measured,p.second.back().time);
      for(const auto& p:lidar_coverage_)measured=std::max(measured,p.second);
      for(const auto& p:cameras_)if(!p.second.empty())measured=std::max(measured,p.second.back().time);
      if(origin_){
        uint64_t measured_time=origin_+uint64_t(std::llround(measured*1e9));
        uint64_t limit=final_time+uint64_t(config_.wait_budget*1e9);
        final_time=std::max(final_time,std::min(measured_time,limit));
      }
    }
    // Drain the final bracketed batch before closing the result files, including
    // when a ROS player stops without supplying the usual spline lookahead.
    if(final_time)advance(final_time,true);
    std::lock_guard<std::mutex> lock(mutex_);
    if(finished_)return;
    finished_=true;
    auto points=geometry_.mapPoints();
    std::ofstream pcd(output_+"/map.pcd");
    pcd << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
    << points.size()
    << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
    << points.size()
    << "\nDATA ascii\n";
    for(const auto& p:points)pcd << p.transpose()
    << "\n";
    uint64_t stamp=origin_+uint64_t(std::max(0.0,last_publish_)*1e9);
    auto map=cloud(points,stamp);
    output_bag_.write("/rig/local_map",map.header.stamp,map);
    output_bag_.close();
    trajectory_file_.flush();
    diagnostics_file_.flush();
    summary();
    std::cout << "Finished: "
    << updates_
    << " odometry updates, "
    << points.size()
    << " map points. Results: "
    << output_
    << std::endl;
  }
} // namespace rig
