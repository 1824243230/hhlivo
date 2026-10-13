#include "rig/frontend.h"
#include <iostream>
#include <stdexcept>
using namespace rig;
int main(){
  try{
    vk::PinholeCamera camera(320,240,1,180,170,160,120);
    Pose body{
      expSO3(Vec3(0.2,-0.1,0.15)),Vec3(1,2,0.5)
    },extrinsic{
      expSO3(Vec3(0.03,0.04,-0.1)),Vec3(0.1,0.2,0.05)
    };
    Vec3 world=(body*extrinsic).apply(Vec3(0.5,0.3,5));
    auto H=VisualFrontend::projectionJacobian(body,extrinsic,world,camera);
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
      }
      Eigen::Vector2d numeric=(camera.world2cam((a*extrinsic).inverse().apply(world))-camera.world2cam((b*extrinsic).inverse().apply(world)))/(2*e);
      if((numeric-H.col(k)).norm()>1e-4)throw std::runtime_error("photometric projection Jacobian mismatch");
    }
    cv::Mat image(10,15,CV_8UC1);
    for(int y=0; y<image.rows; ++y)for(int x=0; x<image.cols; ++x)image.at<uint8_t>(y,x)=x+2*y;
    double value;
    if(!VisualFrontend::pixel(image,4.5,3.5,value)||std::abs(value-11.5)>1e-9)throw std::runtime_error("bilinear pixel interpolation");
    if(VisualFrontend::pixel(image,100,100,value))throw std::runtime_error("out of bounds pixel accepted");
    std::cout << "rig frontend: photometric Jacobian and interpolation passed\n";
  }catch(const std::exception& e){
    std::cerr << e.what()
    << "\n";
    return 1;
  }return 0;
}
