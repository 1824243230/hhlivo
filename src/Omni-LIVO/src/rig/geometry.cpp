#include "rig/frontend.h"
#include <algorithm>
namespace rig {
  GeometryFrontend::GeometryFrontend(const SensorRegistry& c):config_(c) {
    voxel_config_.max_voxel_size_=c.voxel_size;
    voxel_config_.max_layer_=2;
    voxel_config_.max_iterations_=c.iterations;
    voxel_config_.layer_init_num_={
      5,5,5
    };
    voxel_config_.max_points_num_=60;
    voxel_config_.planner_threshold_=0.0025;
    voxel_config_.beam_err_=0.05;
    voxel_config_.dept_err_=0.03;
    voxel_config_.sigma_num_=4;
    voxel_config_.map_sliding_en_=true;
    voxel_config_.half_map_size_=c.map_radius;
    voxel_config_.sliding_thresh_=2;
    voxel_config_.max_visual_voxels_=c.max_voxels;
    map_=std::make_unique<VoxelMapManager>(voxel_config_,unused_);
    map_->extR_.setIdentity();
    map_->extT_.setZero();
  }
  VOXEL_LOCATION GeometryFrontend::key(const Vec3& p) const {
    return {
      int64_t(std::floor(p.x()/config_.voxel_size)),int64_t(std::floor(p.y()/config_.voxel_size)),int64_t(std::floor(p.z()/config_.voxel_size))
    };
  }
  NormalEquations GeometryFrontend::equations(const std::vector<RigPoint>& points,const JointFilter& f,bool record) {
    NormalEquations eq(f.state.layout.size());
    Pose body=f.bodyPose();
    auto J=f.bodyJacobian();
    Mat6 body_covariance=f.bodyCovariance();
    Mat3 info=Mat3::Zero();
    std::vector<Eigen::RowVectorXd> hs(points.size());
    std::vector<double> rs(points.size()),vs(points.size());
    std::vector<Vec3> normals(points.size());
    std::vector<uint8_t> valid(points.size(),0);
#pragma omp parallel for num_threads(config_.threads) schedule(static)
    for(int index=0; index<static_cast<int>(points.size()); ++index) {
      const auto& p=points[index];
      Vec3 world=body.apply(p.point);
      auto k=key(world);
      PointToPlane best{
      };
      double bestprob=0;
      bool found=false;
      pointWithVar pv;
      pv.point_b=p.point;
      pv.point_w=world;
      pv.body_var=p.covariance;
      pv.var=body.R*p.covariance*body.R.transpose();
      // Association includes prior uncertainty, while measurement R excludes that prior.
      Eigen::Matrix<double,3,6> JP;
      JP  << -body.R*skew(p.point),Mat3::Identity();
      pv.var+=JP*body_covariance*JP.transpose();
      for(int x=-1; x<=1; ++x)for(int y=-1; y<=1; ++y)for(int z=-1; z<=1; ++z) {
        auto it=map_->voxel_map_.find({
          k.x+x,k.y+y,k.z+z
        });
        if(it==map_->voxel_map_.end())continue;
        bool success=false;
        double prob=0;
        PointToPlane plane{
        };
        map_->build_single_residual(pv,it->second,0,success,prob,plane);
        if(success&&prob>bestprob){
          found=true;
          best=plane;
          bestprob=prob;
        }
      }
      if(!found)continue;
      double r=best.normal_.dot(world-best.center_);
      Eigen::Matrix<double,1,6> pn;
      pn  << (world-best.center_).transpose(),-best.normal_.transpose();
      double v=0.0004+(best.normal_.transpose()*body.R*p.covariance*body.R.transpose()*best.normal_)(0,0)+(pn*best.plane_var_*pn.transpose())(0,0);
      if(!std::isfinite(v)||v<=0||std::abs(r)>std::min(1.0,4*std::sqrt(v)))continue;
      hs[index]=JointFilter::planeJacobian(body,p.point,best.normal_)*J;
      rs[index]=r;
      vs[index]=v;
      normals[index]=best.normal_;
      valid[index]=1;
    }
    size_t count=0;
    for(size_t i=0; i<points.size(); ++i)if(valid[i]){
      info+=normals[i]*normals[i].transpose()/vs[i];
      ++count;
      if(record)++matched[points[i].lidar_id];
    }
    if(count) {
      Eigen::SelfAdjointEigenSolver<Mat3> eig(info);
      double ratio=eig.eigenvalues()(0)/std::max(eig.eigenvalues()(2),1e-12);
      degeneracy=std::clamp(ratio/0.2,0.1,1.0);
      for(size_t i=0; i<hs.size(); ++i)if(valid[i])eq.add(hs[i],rs[i],vs[i]/degeneracy);
    }return eq;
  }
  void GeometryFrontend::insert(const std::vector<RigPoint>& points,const JointFilter& f,double time) {
    Pose body=f.bodyPose();
    Mat6 body_covariance=f.bodyCovariance();
    std::vector<pointWithVar> pv;
    pv.reserve(points.size());
    for(const auto& p:points){
      pointWithVar v;
      v.point_b=p.point;
      v.point_w=body.apply(p.point);
      v.body_var=p.covariance;
      Eigen::Matrix<double,3,6> J;
      J  << -body.R*skew(p.point),Mat3::Identity();
      v.var=body.R*p.covariance*body.R.transpose()+J*body_covariance*J.transpose();
      pv.push_back(v);
      ++inserted[p.lidar_id];
    }
    if(!pv.empty())map_->UpdateVoxelMap(pv,time);
    auto& m=map_->voxel_map_;
    for(auto it=m.begin(); it!=m.end(); ) {
      Vec3 center(it->second->voxel_center_[0],it->second->voxel_center_[1],it->second->voxel_center_[2]);
      if((center-body.p).norm()>config_.map_radius){
        delete it->second;
        it=m.erase(it);
      }else++it;
    }
    if(m.size()>size_t(config_.max_voxels)) {
      std::vector<std::pair<double,VOXEL_LOCATION>> distances;
      for(const auto& p:m){
        Vec3 c(p.second->voxel_center_[0],p.second->voxel_center_[1],p.second->voxel_center_[2]);
        distances.push_back({
          (c-body.p).squaredNorm(),p.first
        });
      }
      std::sort(distances.begin(),distances.end(),[](const auto& a,const auto& b){
        return a.first>b.first;
      });
      size_t remove=m.size()-config_.max_voxels;
      for(size_t i=0; i<remove; ++i){
        auto it=m.find(distances[i].second);
        delete it->second;
        m.erase(it);
      }
    }peak_voxels=std::max(peak_voxels,m.size());
  }
  void GeometryFrontend::clear(){
    for(auto& p:map_->voxel_map_)delete p.second;
    map_->voxel_map_.clear();
  }
  std::vector<Vec3> GeometryFrontend::mapPoints() const {
    std::vector<Vec3> out;
    for(const auto& p:map_->voxel_map_) {
      const auto& plane=*p.second->plane_ptr_;
      if(plane.is_init_)out.push_back(plane.center_);
      else if(!p.second->temp_points_.empty())out.push_back(p.second->temp_points_.front().point_w);
    }return out;
  }
  Vec3 GeometryFrontend::normalAt(const Vec3& p) const {
    auto it=map_->voxel_map_.find(key(p));
    if(it!=map_->voxel_map_.end()&&it->second->plane_ptr_->is_plane_)return it->second->plane_ptr_->normal_;
    return Vec3::Zero();
  }
} // namespace rig
