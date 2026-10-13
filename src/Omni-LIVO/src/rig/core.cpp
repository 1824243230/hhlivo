#include "rig/core.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rig {
  Mat3 skew(const Vec3& v) {
    Mat3 m;
    m  << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;
    return m;
  }
  Mat3 expSO3(const Vec3& v) {
    const double a = v.norm();
    if (a < 1e-8) return Mat3::Identity() + skew(v) + 0.5 * skew(v) * skew(v);
    return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
  }
  Vec3 logSO3(const Mat3& R) {
    Eigen::AngleAxisd a(R);
    return a.axis() * a.angle();
  }
  namespace {
    Eigen::Matrix<double,6,1> logSE3(const Pose& pose) {
      Vec3 w=logSO3(pose.R);
      double a=w.norm();
      Mat3 W=skew(w);
      double coefficient=a<1e-4 ? 1.0/12+a*a/720 : (1-0.5*a/std::tan(a/2))/(a*a);
      Eigen::Matrix<double,6,1> xi;
      xi  << (Mat3::Identity()-0.5*W+coefficient*W*W)*pose.p,w;
      return xi;
    }
    Pose expSE3(const Eigen::Matrix<double,6,1>& xi) {
      Vec3 w=xi.tail<3>();
      double a=w.norm();
      Mat3 W=skew(w);
      double b=a<1e-4 ? 0.5-a*a/24 : (1-std::cos(a))/(a*a);
      double c=a<1e-4 ? 1.0/6-a*a/120 : (a-std::sin(a))/(a*a*a);
      return {
        expSO3(w),(Mat3::Identity()+b*W+c*W*W)*xi.head<3>()
      };
    }
  }
  int StateLayout::imu(int id) const {
    auto i = std::find(imu_ids.begin(), imu_ids.end(), id);
    if (i == imu_ids.end()) throw std::out_of_range("inactive IMU");
    return 15 * int(i - imu_ids.begin());
  }
  int StateLayout::exposure(int id) const {
    auto i = std::find(camera_ids.begin(), camera_ids.end(), id);
    if (i == camera_ids.end()) throw std::out_of_range("unknown camera");
    return gravity() + 3 + int(i - camera_ids.begin());
  }
  Eigen::VectorXd JointState::difference(const JointState& prior) const {
    Eigen::VectorXd d = Eigen::VectorXd::Zero(layout.size());
    for (int id : layout.imu_ids) {
      int o = layout.imu(id);
      const auto& a = imus.at(id);
      const auto& b = prior.imus.at(id);
      d.segment<3>(o) = logSO3(b.pose.R.transpose() * a.pose.R);
      d.segment<3>(o+3) = a.pose.p-b.pose.p;
      d.segment<3>(o+6) = a.velocity-b.velocity;
      d.segment<3>(o+9) = a.bg-b.bg;
      d.segment<3>(o+12) = a.ba-b.ba;
    }
    d.segment<3>(layout.gravity()) = gravity-prior.gravity;
    for (int id : layout.camera_ids) d(layout.exposure(id)) = exposure.at(id)-prior.exposure.at(id);
    return d;
  }
  void JointState::inject(const Eigen::VectorXd& d) {
    if (d.size() != layout.size() || !d.allFinite()) throw std::runtime_error("invalid state increment");
    for (int id : layout.imu_ids) {
      int o = layout.imu(id);
      auto& s = imus.at(id);
      s.pose.R *= expSO3(d.segment<3>(o));
      s.pose.p += d.segment<3>(o+3);
      s.velocity += d.segment<3>(o+6);
      s.bg += d.segment<3>(o+9);
      s.ba += d.segment<3>(o+12);
    }
    gravity += d.segment<3>(layout.gravity());
    for (int id : layout.camera_ids) exposure[id] = std::clamp(exposure[id] + d(layout.exposure(id)), -4.0, 4.0);
  }
  bool JointState::finite() const {
    if (!gravity.allFinite() || !covariance.allFinite()) return false;
    for (const auto& p : imus) if (!p.second.pose.R.allFinite() || !p.second.pose.p.allFinite() ||
    !p.second.velocity.allFinite() || !p.second.bg.allFinite() || !p.second.ba.allFinite()) return false;
    return true;
  }
  void NormalEquations::add(const Eigen::RowVectorXd& H, double r, double v, double huber) {
    if (!H.allFinite() || !std::isfinite(r) || !std::isfinite(v) || v <= 0) return;
    double nr = std::abs(r) / std::sqrt(v);
    double w = std::min(1.0, huber / std::max(nr, 1e-12)) / v;
    A.noalias() += w * H.transpose() * H;
    b.noalias() += w * H.transpose() * r;
    error += r*r/v;
    ++rows;
  }
  void NormalEquations::merge(const NormalEquations& o) {
    A += o.A;
    b += o.b;
    rows += o.rows;
    error += o.error;
  }
  const ImuConfig& JointFilter::imuConfig(int id) const {
    for (const auto& c : config_.imus) if (c.id == id) return c;
    throw std::out_of_range("IMU configuration");
  }
  void JointFilter::initialize(double t, const std::map<int, ImuSample>& mean) {
    if (mean.empty()) throw std::runtime_error("no usable IMU for initialization");
    state = JointState();
    base_id = mean.begin()->first;
    Vec3 a_body = imuConfig(base_id).extrinsic.R * mean.at(base_id).accel;
    if (a_body.norm() < 1) throw std::runtime_error("IMU acceleration norm too small");
    Pose body{
      Eigen::Quaterniond::FromTwoVectors(a_body.normalized(), Vec3::UnitZ()).toRotationMatrix(), Vec3::Zero()
    };
    for (const auto& p : mean) {
      const auto& c = imuConfig(p.first);
      ImuState s;
      s.pose = body*c.extrinsic;
      s.bg = p.second.gyro;
      s.ba = p.second.accel - s.pose.R.transpose()*Vec3(0,0,9.81);
      state.imus[p.first] = s;
      state.layout.imu_ids.push_back(p.first);
      last_gyro_[p.first] = p.second.gyro;
    }
    for (const auto& c : config_.cameras) if (c.enabled) {
      state.layout.camera_ids.push_back(c.id);
      state.exposure[c.id] = 0;
    }
    state.covariance = Eigen::MatrixXd::Identity(state.layout.size(), state.layout.size()) * 0.01;
    // Initial rig poses are derived from one body pose, so they are correlated.
    for (int a : state.layout.imu_ids) for (int b : state.layout.imu_ids) if (a != b) {
      int i=state.layout.imu(a),j=state.layout.imu(b);
      state.covariance.block<3,3>(i,j) = 0.009 * imuConfig(a).extrinsic.R.transpose()*imuConfig(b).extrinsic.R;
      state.covariance.block<6,6>(i+3,j+3) = 0.009 * Eigen::Matrix<double,6,6>::Identity();
    }
    state.time=t;
    state.initialized=true;
  }
  Pose JointFilter::bodyPose() const {
    return state.imus.at(base_id).pose * imuConfig(base_id).extrinsic.inverse();
  }
  Vec3 JointFilter::bodyVelocity() const {
    const auto& s=state.imus.at(base_id);
    const auto& e=imuConfig(base_id).extrinsic;
    Vec3 w=e.R*(last_gyro_.at(base_id)-s.bg);
    return s.velocity-bodyPose().R*w.cross(e.p);
  }
  Vec3 JointFilter::bodyAngularVelocity() const {
    return imuConfig(base_id).extrinsic.R*(last_gyro_.at(base_id)-state.imus.at(base_id).bg);
  }
  Mat6 JointFilter::bodyTwistCovariance() const {
    const auto& c=imuConfig(base_id);
    const auto& s=state.imus.at(base_id);
    int o=state.layout.imu(base_id);
    Eigen::MatrixXd J=Eigen::MatrixXd::Zero(6,state.layout.size());
    Mat3 R=bodyPose().R;
    J.block<3,3>(0,o)=skew(R.transpose()*s.velocity)*c.extrinsic.R;
    J.block<3,3>(0,o+6)=R.transpose();
    J.block<3,3>(0,o+9)=-skew(c.extrinsic.p)*c.extrinsic.R;
    J.block<3,3>(3,o+9)=-c.extrinsic.R;
    Eigen::Matrix<double,6,3> noise;
    noise  << skew(c.extrinsic.p)*c.extrinsic.R,c.extrinsic.R;
    return J*state.covariance*J.transpose()+noise*noise.transpose()*(c.gyro_noise*c.gyro_noise/0.01);
  }
  Eigen::MatrixXd JointFilter::bodyJacobian() const {
    Eigen::MatrixXd J=Eigen::MatrixXd::Zero(6,state.layout.size());
    int o=state.layout.imu(base_id);
    const auto& e=imuConfig(base_id).extrinsic;
    J.block<3,3>(0,o)=e.R;
    J.block<3,3>(3,o)=bodyPose().R*skew(e.p)*e.R;
    J.block<3,3>(3,o+3).setIdentity();
    return J;
  }
  Mat6 JointFilter::bodyCovariance() const {
    auto J=bodyJacobian();
    return J*state.covariance*J.transpose();
  }
  void JointFilter::propagate(double end, const std::map<int, ImuSample>& in) {
    double dt=end-state.time;
    if (dt<=0) return;
    if (dt>0.031) throw std::runtime_error("unbracketed IMU propagation interval");
    int n=state.layout.size();
    Eigen::MatrixXd F=Eigen::MatrixXd::Identity(n,n),Q=Eigen::MatrixXd::Zero(n,n);
    for (int id : state.layout.imu_ids) {
      const auto& m=in.at(id);
      const auto& c=imuConfig(id);
      auto& s=state.imus.at(id);
      int o=state.layout.imu(id);
      Vec3 w=m.gyro-s.bg, a=m.accel-s.ba;
      Mat3 R=s.pose.R;
      Vec3 world_acc=R*expSO3(w*dt*0.5)*a+state.gravity;
      s.pose.p += s.velocity*dt+0.5*world_acc*dt*dt;
      s.velocity+=world_acc*dt;
      s.pose.R=R*expSO3(w*dt);
      F.block<3,3>(o,o)=expSO3(-w*dt);
      F.block<3,3>(o,o+9)=-Mat3::Identity()*dt;
      F.block<3,3>(o+3,o+6)=Mat3::Identity()*dt;
      F.block<3,3>(o+3,o)=-0.5*R*skew(a)*dt*dt;
      F.block<3,3>(o+3,o+12)=-0.5*R*dt*dt;
      F.block<3,3>(o+3,state.layout.gravity())=0.5*Mat3::Identity()*dt*dt;
      F.block<3,3>(o+6,o)=-R*skew(a)*dt;
      F.block<3,3>(o+6,o+12)=-R*dt;
      F.block<3,3>(o+6,state.layout.gravity())=Mat3::Identity()*dt;
      Q.block<3,3>(o,o).diagonal().setConstant(c.gyro_noise*c.gyro_noise*dt);
      Q.block<3,3>(o+6,o+6).diagonal().setConstant(c.accel_noise*c.accel_noise*dt);
      Q.block<3,3>(o+3,o+3).diagonal().setConstant(c.accel_noise*c.accel_noise*dt*dt*dt/3);
      Q.block<3,3>(o+3,o+6).diagonal().setConstant(c.accel_noise*c.accel_noise*dt*dt/2);
      Q.block<3,3>(o+6,o+3)=Q.block<3,3>(o+3,o+6);
      Q.block<3,3>(o+9,o+9).diagonal().setConstant(c.gyro_walk*c.gyro_walk*dt);
      Q.block<3,3>(o+12,o+12).diagonal().setConstant(c.accel_walk*c.accel_walk*dt);
      last_gyro_[id]=m.gyro;
      ++propagated[id];
    }
    for (int id:state.layout.camera_ids) Q(state.layout.exposure(id),state.layout.exposure(id))=0.0001*dt;
    state.covariance=F*state.covariance*F.transpose()+Q;
    state.covariance=0.5*(state.covariance+state.covariance.transpose()).eval();
    state.time=end;
  }
  NormalEquations JointFilter::rigidEquations() const {
    NormalEquations eq(state.layout.size());
    if (state.imus.size()<2) return eq;
    const auto& a=state.imus.at(base_id);
    const auto& ea=imuConfig(base_id).extrinsic;
    int oa=state.layout.imu(base_id);
    for (int id:state.layout.imu_ids) if (id!=base_id) {
      const auto& b=state.imus.at(id);
      Pose e=ea.inverse()*imuConfig(id).extrinsic;
      int ob=state.layout.imu(id);
      Vec3 rR=logSO3((a.pose.R*e.R).transpose()*b.pose.R);
      Vec3 rp=b.pose.p-a.pose.p-a.pose.R*e.p;
      Eigen::MatrixXd H=Eigen::MatrixXd::Zero(6,state.layout.size());
      // Small-angle Jacobian is exact at the rigid configuration; large misalignment is gated.
      H.block<3,3>(0,oa)=-e.R.transpose();
      H.block<3,3>(0,ob).setIdentity();
      H.block<3,3>(3,oa)=a.pose.R*skew(e.p);
      H.block<3,3>(3,oa+3)=-Mat3::Identity();
      H.block<3,3>(3,ob+3).setIdentity();
      if (rR.norm()>0.5 || rp.norm()>2) continue;
      for(int k=0; k<3; ++k) {
        eq.add(H.row(k),rR(k),std::pow(config_.rigid_rotation_noise,2));
        eq.add(H.row(k+3),rp(k),std::pow(config_.rigid_position_noise,2));
      }
    } return eq;
  }
  bool JointFilter::iteration(const JointState& prior,const NormalEquations& eq,Eigen::MatrixXd& post,Eigen::VectorXd& d) {
    if(eq.rows==0) return false;
    const int n=state.layout.size();
    Eigen::LDLT<Eigen::MatrixXd> ldlt(prior.covariance);
    if(ldlt.info()!=Eigen::Success || !ldlt.isPositive()) {
      ++rejected_updates;
      return false;
    }
    Eigen::MatrixXd info=ldlt.solve(Eigen::MatrixXd::Identity(n,n));
    Eigen::LDLT<Eigen::MatrixXd> solve(info+eq.A);
    if(solve.info()!=Eigen::Success || !solve.isPositive()) {
      ++rejected_updates;
      return false;
    }
    d=solve.solve(-eq.b-info*state.difference(prior));
    for(int id:state.layout.imu_ids) {
      int o=state.layout.imu(id);
      if(d.segment<3>(o).norm()>0.5 || d.segment<3>(o+3).norm()>2) {
        ++rejected_updates;
        return false;
      }
    }
    if(!d.allFinite()) {
      ++rejected_updates;
      return false;
    }
    post=solve.solve(Eigen::MatrixXd::Identity(n,n));
    state.inject(d);
    return true;
  }
  void JointFilter::commit(const Eigen::MatrixXd& p,const Eigen::VectorXd& d) {
    Eigen::MatrixXd J=Eigen::MatrixXd::Identity(state.layout.size(),state.layout.size());
    for(int id:state.layout.imu_ids) {
      int o=state.layout.imu(id);
      J.block<3,3>(o,o)=Mat3::Identity()-0.5*skew(d.segment<3>(o));
    }
    state.covariance=J*p*J.transpose();
    state.covariance=0.5*(state.covariance+state.covariance.transpose()).eval();
    state.covariance.diagonal().array()+=1e-12;
  }
  void JointFilter::constrainRig() {
    JointState prior=state;
    auto eq=rigidEquations();
    Eigen::MatrixXd post;
    Eigen::VectorXd d;
    if(iteration(prior,eq,post,d)) {
      commit(post,d);
      ++rigid_updates;
    }
  }
  void JointFilter::removeImu(int id) {
    if(!state.imus.count(id)) return;
    if(state.imus.size()==1) {
      state.initialized=false;
      return;
    }
    StateLayout old=state.layout;
    state.layout.imu_ids.erase(std::remove(state.layout.imu_ids.begin(),state.layout.imu_ids.end(),id),state.layout.imu_ids.end());
    std::vector<int> indices;
    for(int k:state.layout.imu_ids) for(int j=0; j<15; ++j) indices.push_back(old.imu(k)+j);
    for(int j=old.gravity(); j<old.size(); ++j) indices.push_back(j);
    Eigen::MatrixXd p(indices.size(),indices.size());
    for(size_t i=0; i<indices.size(); ++i) for(size_t j=0; j<indices.size(); ++j) p(i,j)=state.covariance(indices[i],indices[j]);
    state.covariance=p;
    state.imus.erase(id);
    if(base_id==id) base_id=state.layout.imu_ids.front();
  }
  void JointFilter::restoreImu(int id) {
    if(state.imus.count(id) || !state.initialized) return;
    StateLayout old=state.layout;
    Pose body=bodyPose();
    Vec3 vel=bodyVelocity();
    const auto& c=imuConfig(id);
    ImuState s;
    s.pose=body*c.extrinsic;
    s.velocity=vel;
    state.layout.imu_ids.push_back(id);
    state.imus[id]=s;
    last_gyro_[id]=Vec3::Zero();
    Eigen::MatrixXd M=Eigen::MatrixXd::Zero(state.layout.size(),old.size());
    for(int k:old.imu_ids) M.block(state.layout.imu(k),old.imu(k),15,15).setIdentity();
    M.block(state.layout.gravity(),old.gravity(),old.size()-old.gravity(),old.size()-old.gravity()).setIdentity();
    int o=state.layout.imu(id),b=old.imu(base_id);
    Pose e=imuConfig(base_id).extrinsic.inverse()*c.extrinsic;
    M.block<3,3>(o,b)=e.R.transpose();
    M.block<3,3>(o+3,b)=-state.imus.at(base_id).pose.R*skew(e.p);
    M.block<3,3>(o+3,b+3).setIdentity();
    M.block<3,3>(o+6,b+6).setIdentity();
    state.covariance=M*state.covariance*M.transpose();
    state.covariance.block(o,o,15,15).diagonal().array()+=0.01;
  }
  Eigen::Matrix<double,1,6> JointFilter::planeJacobian(const Pose& pose,const Vec3& p,const Vec3& n) {
    Eigen::Matrix<double,1,6> H;
    H.head<3>()=-n.transpose()*pose.R*skew(p);
    H.tail<3>()=n.transpose();
    return H;
  }
  void TrajectoryBuffer::append(const TrajectoryKnot& k) {
    if(!knots.empty() && k.time<knots.back().time-1e-9) throw std::runtime_error("trajectory time reversal");
    if(!knots.empty() && std::abs(k.time-knots.back().time)<1e-9) knots.back()=k;
    else knots.push_back(k);
    while(knots.size()>2 && knots.front().time<k.time-2) knots.pop_front();
  }
  bool TrajectoryBuffer::query(double t,Pose& pose) {
    if(knots.empty() || t<knots.front().time-1e-7 || t>knots.back().time+1e-7) {
      ++rejected_queries;
      return false;
    }
    auto it=std::lower_bound(knots.begin(),knots.end(),t,[](const auto& k,double v){
      return k.time<v;
    });
    if(it==knots.end()) {
      pose=knots.back().pose;
      return true;
    }
    if(it==knots.begin()) {
      pose=it->pose;
      return true;
    }
    size_t b=it-knots.begin(),a=b-1;
    double dt=knots[b].time-knots[a].time,u=(t-knots[a].time)/dt;
    if(a>0 && b+1<knots.size() && std::abs(knots[a].time-knots[a-1].time-dt)<1e-5 && std::abs(knots[b+1].time-knots[b].time-dt)<1e-5) {
      double u2=u*u,u3=u2*u;
      double b0=std::pow(1-u,3)/6,b1=(3*u3-6*u2+4)/6,b2=(-3*u3+3*u2+3*u+1)/6,b3=u3/6;
      const Pose& T0=knots[a-1].pose;
      const Pose& T1=knots[a].pose;
      const Pose& T2=knots[b].pose;
      const Pose& T3=knots[b+1].pose;
      pose=T0*expSE3((1-b0)*logSE3(T0.inverse()*T1))*expSE3((b2+b3)*logSE3(T1.inverse()*T2))*expSE3(b3*logSE3(T2.inverse()*T3));
      ++spline_queries;
    } else {
      pose.R=knots[a].pose.R*expSO3(u*logSO3(knots[a].pose.R.transpose()*knots[b].pose.R));
      pose.p=(2*u*u*u-3*u*u+1)*knots[a].pose.p+(u*u*u-2*u*u+u)*dt*knots[a].velocity+
      (-2*u*u*u+3*u*u)*knots[b].pose.p+(u*u*u-u*u)*dt*knots[b].velocity;
      ++fallback_queries;
    } return true;
  }
  void TrajectoryBuffer::correctFrom(double t,const Pose& before,const Pose& after) {
    Pose delta=after*before.inverse();
    for(auto& k:knots) if(k.time>=t-1e-8) {
      k.pose=delta*k.pose;
      k.velocity=delta.R*k.velocity;
    }
  }
  Mat3 TrajectoryBuffer::pointCovariance(const Vec3& p,double dt,const LidarConfig& c) const {
    double r=std::max(p.norm(),1e-9);
    Vec3 dir=p/r;
    Mat3 cov=c.range_noise*c.range_noise*dir*dir.transpose()+c.beam_noise*c.beam_noise*r*r*(Mat3::Identity()-dir*dir.transpose());
    // Conditional process-noise approximation: no absolute pose prior is counted again in R.
    double a=std::abs(dt);
    cov+=Mat3::Identity()*(0.04*a*a*a/3)+skew(p)*skew(p).transpose()*(0.0004*a);
    Eigen::Matrix<double,3,6> J;
    J  << -skew(p),Mat3::Identity();
    return cov+J*c.extrinsic_cov*J.transpose();
  }
  bool SensorHealthMonitor::observe(const std::string& key,double t,double arrival,bool valid) {
    auto& h=sensors[key];
    ++h.received;
    h.last_arrival=arrival;
    if(!valid) {
      ++h.rejected;
      h.consecutive_good=0;
      h.isolated=true;
      return false;
    }
    if(h.last_time>=0 && t<=h.last_time+1e-9) {
      if(std::abs(t-h.last_time)<1e-9)++h.duplicates;
      else++h.stale;
      return false;
    }
    h.last_time=t;
    ++h.consecutive_good;
    if(h.isolated && h.consecutive_good>=5) {
      h.isolated=false;
      ++h.recoveries;
    }
    return !h.isolated;
  }
} // namespace rig
