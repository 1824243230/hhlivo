#include "rig/core.h"
#include <iostream>
#include <stdexcept>
using namespace rig;
void require(bool v,const char* s){
  if(!v)throw std::runtime_error(s);
}
void covariance(const JointState& s){
  require(s.finite(),"finite state");
  require((s.covariance-s.covariance.transpose()).norm()<1e-9,"symmetric covariance");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> e(s.covariance);
  require(e.eigenvalues().minCoeff()>-1e-10,"positive semidefinite covariance");
}
int main(){
  try{
    SensorRegistry c;
    ImuConfig i;
    i.id=0;
    i.topic="/imu0";
    c.imus.push_back(i);
    i.id=1;
    i.topic="/imu1";
    i.extrinsic.R=expSO3(Vec3(0.2,-0.1,0.4));
    i.extrinsic.p=Vec3(0.2,0.1,0.05);
    c.imus.push_back(i);
    LidarConfig l;
    l.topic="/lidar";
    c.lidars.push_back(l);
    for(int k=0; k<8; ++k){
      CameraConfig a;
      a.id=k;
      a.topic="/camera"+std::to_string(k);
      c.cameras.push_back(a);
    }c.validate();
    JointFilter f(c);
    std::map<int,ImuSample> mean;
    for(const auto& x:c.imus)mean[x.id]={
      x.id,0,x.extrinsic.R.transpose()*Vec3(0,0,9.81),Vec3::Zero()
    };
    f.initialize(0,mean);
    require(f.state.layout.size()==41,"41-D state");
    {
      JointFilter rigid(c);
      rigid.initialize(0,mean);
      JointState original=rigid.state;
      auto residual=[&](const JointState& s){
        Eigen::Matrix<double,6,1> r;
        Pose e=c.imus[0].extrinsic.inverse()*c.imus[1].extrinsic;
        const auto& a=s.imus.at(0).pose;
        const auto& b=s.imus.at(1).pose;
        r  << logSO3((a.R*e.R).transpose()*b.R),b.p-a.p-a.R*e.p;
        return r;
      };
      Eigen::MatrixXd numeric(6,original.layout.size());
      for(int k=0; k<original.layout.size(); ++k){
        JointState a=original,b=original;
        Eigen::VectorXd d=Eigen::VectorXd::Zero(original.layout.size());
        d(k)=1e-6;
        a.inject(d);
        b.inject(-d);
        numeric.col(k)=(residual(a)-residual(b))/2e-6;
      }
      Eigen::Matrix<double,6,6> precision=Eigen::Matrix<double,6,6>::Zero();
      precision.diagonal().head<3>().setConstant(1/std::pow(c.rigid_rotation_noise,2));
      precision.diagonal().tail<3>().setConstant(1/std::pow(c.rigid_position_noise,2));
      auto analytic=rigid.rigidEquations();
      require((analytic.A-numeric.transpose()*precision*numeric).norm()/analytic.A.norm()<1e-6,"rigid Jacobian finite difference");
    }
    {
      SensorRegistry quiet=c;
      for(auto& i:quiet.imus){
        i.gyro_noise=1e-9;
        i.accel_noise=1e-9;
        i.gyro_walk=0;
        i.accel_walk=0;
      }
      JointFilter initial(quiet);
      initial.initialize(0,mean);
      initial.state.covariance.setIdentity();
      JointFilter propagated=initial;
      propagated.propagate(0.001,mean);
      Eigen::MatrixXd numeric(initial.state.layout.size(),initial.state.layout.size());
      for(int k=0; k<initial.state.layout.size(); ++k){
        JointFilter a=initial,b=initial;
        Eigen::VectorXd d=Eigen::VectorXd::Zero(initial.state.layout.size());
        d(k)=1e-6;
        a.state.inject(d);
        b.state.inject(-d);
        a.propagate(0.001,mean);
        b.propagate(0.001,mean);
        numeric.col(k)=(a.state.difference(propagated.state)-b.state.difference(propagated.state))/2e-6;
      }
      // The first-order propagation Jacobian has O(dt^2) discretization error.
      require((propagated.state.covariance-numeric*numeric.transpose()).norm()<1e-4,"propagation covariance finite difference");
    }
    for(int k=1; k<=100; ++k){
      f.propagate(k*0.01,mean);
      f.constrainRig();
    }require(f.bodyPose().p.norm()<1e-7,"stationary propagation");
    require(f.propagated[0]==100&&f.propagated[1]==100,"both IMUs propagated");
    covariance(f.state);
    Pose body{
      expSO3(Vec3(0.2,-0.3,0.1)),Vec3(1,2,3)
    };
    Vec3 p(2,1,4),n=Vec3(0.2,0.3,0.7).normalized();
    auto H=JointFilter::planeJacobian(body,p,n);
    for(int k=0; k<6; ++k){
      Pose a=body,b=body;
      double e=1e-6;
      if(k<3){
        Vec3 d=Vec3::Zero();
        d(k)=e;
        a.R*=expSO3(d);
        b.R*=expSO3(-d);
      }else{
        a.p(k-3)+=e;
        b.p(k-3)-=e;
      }require(std::abs((n.dot(a.apply(p))-n.dot(b.apply(p)))/(2*e)-H(k))<1e-6,"plane Jacobian finite difference");
    }
    f.base_id=1;
    Eigen::MatrixXd J=f.bodyJacobian();
    JointState before=f.state;
    Pose original=f.bodyPose();
    for(int k=0; k<f.state.layout.size(); ++k){
      Eigen::VectorXd d=Eigen::VectorXd::Zero(f.state.layout.size());
      d(k)=1e-6;
      f.state=before;
      f.state.inject(d);
      Pose a=f.bodyPose();
      Eigen::Matrix<double,6,1> diff;
      diff  << logSO3(original.R.transpose()*a.R),(a.p-original.p);
      require((diff/1e-6-J.col(k)).norm()<1e-5,"body Jacobian finite difference");
    }f.state=before;
    // A body-position observation must affect the coupled IMU states and biases.
    NormalEquations eq(f.state.layout.size());
    Eigen::RowVectorXd row=J.row(3);
    eq.add(row,0.05,0.001);
    JointState prior=f.state;
    Eigen::MatrixXd post;
    Eigen::VectorXd d;
    require(f.iteration(prior,eq,post,d),"joint measurement accepted");
    f.commit(post,d);
    require(d.segment<3>(f.state.layout.imu(0)+9).norm()+d.segment<3>(f.state.layout.imu(0)+12).norm()>1e-10,"measurement updates correlated bias");
    covariance(f.state);
    f.removeImu(0);
    require(f.state.layout.size()==26&&f.base_id==1,"IMU marginalization");
    covariance(f.state);
    f.restoreImu(0);
    require(f.state.layout.size()==41,"IMU augmentation");
    covariance(f.state);
    TrajectoryBuffer t;
    for(int k=0; k<=100; ++k)t.append({
      k*0.01,{
        Mat3::Identity(),Vec3(k*0.01,0,0)
      },Vec3(1,0,0),Mat6::Identity()*0.01
    });
    Pose q;
    require(t.query(0.555,q),"spline query");
    require(std::abs(q.p.x()-0.555)<1e-9&&t.spline_queries>0,"spline constant velocity");
    require(!t.query(2,q),"no trajectory extrapolation");
    auto near=t.pointCovariance(Vec3(1,0,0),0.01,l),far=t.pointCovariance(Vec3(10,0,0),0.1,l);
    require(far.trace()>near.trace(),"point uncertainty increases with range/time");
    TrajectoryBuffer curve;
    for(int k=0; k<=100; ++k){
      double x=k*0.01;
      curve.append({
        x,{
          Mat3::Identity(),Vec3(x*x,0,0)
        },Vec3(2*x,0,0),Mat6::Zero()
      });
    }
    Pose left,center,right;
    curve.query(0.5-1e-8,left);
    curve.query(0.5,center);
    curve.query(0.5+1e-8,right);
    require((center.p-(left.p+right.p)/2).norm()<1e-8,"continuous spline at exact control times");
    TrajectoryBuffer screw;
    for(int k=0; k<=100; ++k){
      double angle=k*0.001;
      screw.append({
        k*0.01,{
          expSO3(Vec3(0,0,angle)),Vec3(std::sin(angle)/0.1,(1-std::cos(angle))/0.1,0)
        },Vec3(std::cos(angle),std::sin(angle),0),Mat6::Zero()
      });
    }
    require(screw.query(0.555,q),"SE3 screw query");
    require((q.p-Vec3(std::sin(0.0555)/0.1,(1-std::cos(0.0555))/0.1,0)).norm()<1e-8,"SE3 constant body twist");
    SensorHealthMonitor h;
    require(h.observe("imu/0",0,0,true),"healthy sample");
    require(!h.observe("imu/0",0,0,true),"duplicate rejected");
    require(!h.observe("imu/0",1,1,false),"invalid sample isolated");
    for(int k=1; k<=5; ++k)h.observe("imu/0",k, k,true);
    require(!h.sensors["imu/0"].isolated&&h.sensors["imu/0"].recoveries==1,"recovery hysteresis");
    std::cout << "rig core: state, covariance, Jacobians, spline, uncertainty, faults passed\n";
  }catch(const std::exception& e){
    std::cerr << e.what()
    << "\n";
    return 1;
  }return 0;
}
