#include "rig/mapper.h"
#include <rosbag/view.h>
#include <omp.h>
#include <iostream>
#include <map>
int main(int argc,char** argv) {
  try {
    std::string config,output="rig_results";
    std::vector<std::string> bags;
    for(int i=1; i<argc; ++i){
      std::string a=argv[i];
      if(a=="--config"&&i+1<argc)config=argv[++i];
      else if(a=="--output"&&i+1<argc)output=argv[++i];
      else if(a=="--bag"&&i+1<argc){
        bags.push_back(argv[++i]);
        while(i+1<argc&&std::string(argv[i+1]).rfind("--",0)!=0&&std::string(argv[i+1]).find(":=")==std::string::npos)bags.push_back(argv[++i]);
      }else if(a=="--help"){
        std::cout << "rig_mapping --config CONFIG --output DIR [--bag BAG ...]\n";
        return 0;
      }else if(a.find(":=")==std::string::npos)throw std::runtime_error("unknown argument: "+a);
    }
    if(config.empty())throw std::runtime_error("--config is required");
    auto registry=rig::SensorRegistry::load(config);
    omp_set_num_threads(registry.threads);
    cv::setNumThreads(registry.threads);
    ros::Time::init();
    bool online=bags.empty();
    rig::RigMapper mapper(registry,output,online);
    std::map<std::string,std::pair<std::string,int>> topics;
    for(const auto& c:registry.imus)if(c.enabled)topics[c.topic]={
      "imu",c.id
    };
    for(const auto& c:registry.lidars)if(c.enabled)topics[c.topic]={
      "lidar",c.id
    };
    for(const auto& c:registry.cameras)if(c.enabled)topics[c.topic]={
      "camera",c.id
    };
    if(!online){
      uint64_t last=0;
      for(const auto& path:bags){
        rosbag::Bag bag(path,rosbag::bagmode::Read);
        rosbag::View view(bag);
        for(const auto& message:view){
          auto it=topics.find(message.getTopic());
          if(it==topics.end())continue;
          uint64_t arrival=message.getTime().toNSec();
          last=arrival;
          if(it->second.first=="imu"){
            auto m=message.instantiate<sensor_msgs::Imu>();
            if(!m)throw std::runtime_error("incorrect IMU message type");
            mapper.imu(*m,it->second.second,arrival);
          }
          else if(it->second.first=="lidar"){
            auto m=message.instantiate<sensor_msgs::PointCloud2>();
            if(!m)throw std::runtime_error("incorrect LiDAR message type");
            mapper.lidar(*m,it->second.second,arrival);
          }
          else{
            auto m=message.instantiate<sensor_msgs::Image>();
            if(!m)throw std::runtime_error("incorrect camera message type");
            mapper.camera(*m,it->second.second,arrival);
          }
          mapper.advance(arrival);
        }bag.close();
      }mapper.advance(last,true);
      mapper.finish();
      return 0;
    }
    ros::init(argc,argv,"rig_mapping");
    ros::NodeHandle nh;
    mapper.publishWith(nh);
    std::vector<ros::Subscriber> subscribers;
    for(const auto& c:registry.imus)if(c.enabled)subscribers.push_back(nh.subscribe<sensor_msgs::Imu>(c.topic,1000,[&mapper,id=c.id](const sensor_msgs::Imu::ConstPtr& m){
      uint64_t now=ros::Time::now().toNSec(); mapper.imu(*m,id,now); mapper.advance(now);
    }));
    for(const auto& c:registry.lidars)if(c.enabled)subscribers.push_back(nh.subscribe<sensor_msgs::PointCloud2>(c.topic,20,[&mapper,id=c.id](const sensor_msgs::PointCloud2::ConstPtr& m){
      uint64_t now=ros::Time::now().toNSec(); mapper.lidar(*m,id,now); mapper.advance(now);
    }));
    for(const auto& c:registry.cameras)if(c.enabled)subscribers.push_back(nh.subscribe<sensor_msgs::Image>(c.topic,10,[&mapper,id=c.id](const sensor_msgs::Image::ConstPtr& m){
      uint64_t now=ros::Time::now().toNSec(); mapper.camera(*m,id,now); mapper.advance(now);
    }));
    auto timer=nh.createTimer(ros::Duration(0.05),[&mapper](const ros::TimerEvent&){
      mapper.advance(ros::Time::now().toNSec());
    });
    ros::spin();
    mapper.finish();
  }catch(const std::exception& e){
    std::cerr << "rig_mapping: "
    << e.what()
    << std::endl;
    return 1;
  }return 0;
}
