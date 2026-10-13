#include "rig/frontend.h"
#include <vikit/equidistant_camera.h>
#include <opencv2/imgproc.hpp>
#include <set>
namespace rig {
  VisualFrontend::VisualFrontend(const SensorRegistry& c):config_(c) {
    warp_.patch_size_total=c.patch_size*c.patch_size;
    for(const auto& p:c.cameras) if(p.enabled) {
      CameraContext ctx;
      ctx.config=p;
      ctx.variance=c.photo_variance;
      if(p.model=="equidistant")ctx.model=std::make_unique<vk::EquidistantCamera>(p.width,p.height,p.scale,p.fx,p.fy,p.cx,p.cy,p.distortion[0],p.distortion[1],p.distortion[2],p.distortion[3]);
      else ctx.model=std::make_unique<vk::PinholeCamera>(p.width,p.height,p.scale,p.fx,p.fy,p.cx,p.cy,p.distortion[0],p.distortion[1],p.distortion[2],p.distortion[3]);
      cameras.emplace(p.id,std::move(ctx));
    }
  }
  bool VisualFrontend::pixel(const cv::Mat& img,double u,double v,double& value) {
    if(!std::isfinite(u)||!std::isfinite(v)||u<1||v<1||u>=img.cols-2||v>=img.rows-2||img.type()!=CV_8UC1)return false;
    int x=int(u),y=int(v);
    double a=u-x,b=v-y;
    const auto* p=img.ptr<uint8_t>(y);
    const auto* q=img.ptr<uint8_t>(y+1);
    value=(1-a)*(1-b)*p[x]+a*(1-b)*p[x+1]+(1-a)*b*q[x]+a*b*q[x+1];
    return true;
  }
  Eigen::Matrix<double,2,6> VisualFrontend::projectionJacobian(const Pose& body,const Pose& ext,const Vec3& world,const vk::AbstractCamera& cam) {
    Vec3 pb=body.R.transpose()*(world-body.p),pc=ext.inverse().apply(pb);
    Eigen::Matrix<double,2,3> J;
    for(int k=0; k<3; ++k){
      Vec3 a=pc,b=pc;
      a(k)+=1e-5;
      b(k)-=1e-5;
      J.col(k)=(cam.world2cam(a)-cam.world2cam(b))/2e-5;
    }
    Eigen::Matrix<double,3,6> G;
    G  << ext.R.transpose()*skew(pb),-ext.R.transpose()*body.R.transpose();
    return J*G;
  }
  NormalEquations VisualFrontend::equations(const std::vector<CameraObservation>& images,const JointFilter& f,bool record) {
    NormalEquations result(f.state.layout.size());
    Pose body=f.bodyPose();
    auto bodyJ=f.bodyJacobian();
    std::vector<NormalEquations> per_camera;
    for(size_t i=0; i<images.size(); ++i)per_camera.emplace_back(f.state.layout.size());
    std::vector<uint64_t> cross(images.size(),0);
    int budget=std::min(config_.camera_max,std::max(config_.camera_min,config_.visual_points/std::max(1,int(images.size()))));
#pragma omp parallel for num_threads(config_.threads) schedule(static)
    for(int index=0; index<static_cast<int>(images.size()); ++index) {
      const auto& obs=images[index];
      auto& eq=per_camera[index];
      auto& ctx=cameras.at(obs.id);
      const auto& cam=*ctx.model;
      Pose ext=obs.relative*ctx.config.extrinsic,cp=body*ext;
      Pose cw=cp.inverse();
      double gain=std::exp(f.state.exposure.at(obs.id));
      int used=0;
      double mse=0;
      size_t pixels=0;
      std::vector<const PhotoReference*> candidates;
      for(const auto& r:ctx.references)candidates.push_back(&r);
      if(config_.cross_camera)for(const auto& c:cameras)if(c.first!=obs.id)for(const auto& r:c.second.references)candidates.push_back(&r);
      std::set<std::tuple<int,int,int>> unique;
      for(const auto* ref:candidates) {
        if(used>=budget)break;
        if(obs.time-ref->time>2||obs.time<=ref->time+1e-7)continue;
        Vec3 pc=cw.apply(ref->world);
        if(pc.z()<0.3)continue;
        auto px=cam.world2cam(pc);
        int half=config_.patch_size/2;
        if(!cam.isInFrame(px.cast<int>(),half+3))continue;
        auto cell=std::make_tuple(int(ref->world.x()*20),int(ref->world.y()*20),int(ref->world.z()*20));
        if(!unique.insert(cell).second)continue;
        const auto& source=*cameras.at(ref->camera).model;
        Eigen::Matrix2d A;
        Vec3 n=ref->normal;
        double plane=n.dot(ref->world);
        bool good=true;
        for(int axis=0; axis<2; ++axis){
          Eigen::Vector2d q=ref->pixel;
          q(axis)+=half;
          Vec3 ray=ref->camera_pose.R*source.cam2world(q);
          double denom=n.dot(ray);
          if(std::abs(denom)<1e-6){
            good=false;
            break;
          }double depth=(plane-n.dot(ref->camera_pose.p))/denom;
          Vec3 target=cw.apply(ref->camera_pose.p+depth*ray);
          if(depth<=0||target.z()<=0){
            good=false;
            break;
          }
          A.col(axis)=(cam.world2cam(target)-px)/half;
        }
        if(!good||!A.allFinite()||std::abs(A.determinant())<0.05||std::abs(A.determinant())>20)continue;
        Eigen::Matrix2d inverse=A.inverse();
        for(int y=-half; y<half; ++y)for(int x=-half; x<half; ++x){
          Eigen::Vector2d q=ref->pixel+inverse*Eigen::Vector2d(x,y);
          if(q.x()<1||q.y()<1||q.x()>=ref->image.cols-2||q.y()>=ref->image.rows-2)good=false;
        }
        if(!good)continue;
        std::vector<float> patch(config_.patch_size*config_.patch_size,0);
        warp_.warpAffine(A,ref->image,ref->pixel,0,0,0,half,patch.data());
        Eigen::MatrixXd JP=projectionJacobian(body,ext,ref->world,cam)*bodyJ;
        std::vector<Eigen::RowVectorXd> hs;
        std::vector<double> residuals;
        double squared=0;
        for(int y=-half; y<half; ++y)for(int x=-half; x<half; ++x) {
          double val,left,right,up,down;
          double u=px.x()+x,v=px.y()+y;
          if(!pixel(obs.image,u,v,val)||!pixel(obs.image,u-1,v,left)||!pixel(obs.image,u+1,v,right)||!pixel(obs.image,u,v-1,up)||!pixel(obs.image,u,v+1,down)){
            good=false;
            break;
          }
          double r=gain*val-std::exp(ref->exposure)*patch[(y+half)*config_.patch_size+x+half];
          Eigen::RowVectorXd H=gain*(Eigen::RowVector2d((right-left)/2,(down-up)/2)*JP);
          H(f.state.layout.exposure(obs.id))+=gain*val;
          hs.push_back(H);
          residuals.push_back(r);
          squared+=r*r;
        }
        if(!good||hs.empty()||std::sqrt(squared/hs.size())>config_.photo_gate){
          if(record)++ctx.rejected;
          continue;
        }
        for(size_t k=0; k<hs.size(); ++k)eq.add(hs[k],residuals[k],ctx.variance);
        ++used;
        pixels+=hs.size();
        mse+=squared;
        if(record){
          ++ctx.observations;
          if(ref->camera!=obs.id)++cross[index];
        }
      }
      if(record&&pixels)ctx.variance=std::clamp(0.8*ctx.variance+0.2*mse/pixels,config_.photo_variance,10*config_.photo_variance);
    }
    // Deterministic reduction: worker scheduling never changes summation order.
    for(size_t i=0; i<per_camera.size(); ++i){
      result.merge(per_camera[i]);
      cross_observations+=cross[i];
    }
    return result;
  }
  void VisualFrontend::seed(const std::vector<CameraObservation>& images,const std::vector<RigPoint>& points,const JointFilter& f,const GeometryFrontend& geom) {
    Pose body=f.bodyPose();
    int budget=std::min(config_.camera_max,std::max(config_.camera_min,config_.visual_points/std::max(1,int(images.size()))));
    for(const auto& obs:images) {
      auto& ctx=cameras.at(obs.id);
      ++ctx.images;
      Pose cp=body*obs.relative*ctx.config.extrinsic,cw=cp.inverse();
      std::map<std::pair<int,int>,std::pair<double,PhotoReference>> selected;
      for(const auto& p:points) {
        Vec3 world=body.apply(p.point),pc=cw.apply(world);
        if(pc.z()<0.3)continue;
        auto px=ctx.model->world2cam(pc);
        if(!ctx.model->isInFrame(px.cast<int>(),config_.patch_size+3))continue;
        double l,r,u,d;
        if(!pixel(obs.image,px.x()-1,px.y(),l)||!pixel(obs.image,px.x()+1,px.y(),r)||!pixel(obs.image,px.x(),px.y()-1,u)||!pixel(obs.image,px.x(),px.y()+1,d))continue;
        double score=(r-l)*(r-l)+(d-u)*(d-u);
        if(score<4)continue;
        auto key=std::make_pair(int(px.x()/24),int(px.y()/24));
        if(selected.count(key)&&selected.at(key).first>=score)continue;
        PhotoReference ref;
        ref.camera=obs.id;
        ref.world=world;
        ref.pixel=px;
        ref.image=obs.image;
        ref.camera_pose=cp;
        ref.exposure=f.state.exposure.at(obs.id);
        ref.time=obs.time;
        ref.normal=geom.normalAt(world);
        if(ref.normal.norm()<0.5)ref.normal=(world-cp.p).normalized();
        selected[key]={
          score,ref
        };
      }
      std::vector<std::pair<double,PhotoReference>> ranked;
      for(auto& p:selected)ranked.push_back(std::move(p.second));
      std::sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){
        return a.first>b.first;
      });
      if(!ranked.empty()){
        ctx.references.clear();
        for(int i=0; i<std::min(budget,int(ranked.size())); ++i)ctx.references.push_back(std::move(ranked[i].second));
      }
    }
    size_t count=0;
    for(auto& c:cameras){
      auto& r=c.second.references;
      r.erase(std::remove_if(r.begin(),r.end(),[&](const auto& p){
        return f.state.time-p.time>2;
      }),r.end());
      count+=r.size();
    }
    if(count>size_t(config_.max_visual_points))for(auto& c:cameras)c.second.references.resize(std::min(c.second.references.size(),size_t(config_.max_visual_points/cameras.size())));
  }
  void VisualFrontend::clear(){
    for(auto& c:cameras)c.second.references.clear();
  }
} // namespace rig
