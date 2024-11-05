#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_CLOUD_INFO_PUBLISHER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_CLOUD_INFO_PUBLISHER_H_

#include <assert.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <iostream>
#include <vector>
#include <unordered_map>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "ros/ros.h"
#include "vehicle_msgs/CloudInfo.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

namespace planning {

class EudmPlannerPublisher {
 public:
  EudmPlannerPublisher(ros::NodeHandle nh, const int &max_num) // TODO for MLAD: 需要将其改为，发送时再指定特定的agv id
      : nh_(nh), max_num_(max_num) { }

  EudmPlannerPublisher(ros::NodeHandle nh)
      : nh_(nh) { }

  void Init() {
    // std::string cloud_info_topic = std::string("/local/agent_") +
    //                                 std::to_string(0) +
    //                                 std::string("/cloud_info_topic");
    // std::cout << cloud_info_topic << std::endl;
    // publisher_ =
    //     nh_.advertise<vehicle_msgs::CloudInfo>(cloud_info_topic, 1);
    for (int i=0; i<max_num_; i++) {
      std::string cloud_info_topic = std::string("/local/agent_") +
                                     std::to_string(i) +
                                     std::string("/cloud_info_topic");

      cloud_infos_pubs_.insert(std::make_pair(i, nh_.advertise<vehicle_msgs::CloudInfo>(cloud_info_topic, 1)));
    }
    // std::string cloud_info_topic = std::string("/local/agent_") +
    //                                 std::to_string(0) +
    //                                 std::string("/cloud_info_topic");

    // cloud_infos_pubs_.insert(std::make_pair(0, nh_.advertise<vehicle_msgs::CloudInfo>(cloud_info_topic, 1)));
  }

  void PublishDataWithStamp(const ros::Time& stamp, const int &vehicle_id, vehicle_msgs::CloudInfo info) {
    if (cloud_infos_pubs_.count(vehicle_id)==0) {
      //新建此id对应的publisher
      std::string cloud_info_topic = std::string("/local/agent_") +
                                     std::to_string(vehicle_id) +
                                     std::string("/cloud_info_topic");

      cloud_infos_pubs_.insert(std::make_pair(vehicle_id, nh_.advertise<vehicle_msgs::CloudInfo>(cloud_info_topic, 1)));
        
      // key_ids_.push_back(vehicle_id);
    }
    //LOG(ERROR) << "[EudmPlannerPublisher::PublishDataWithStamp] publish msg: " << info.behavior.actual_desired_velocity 
           //    << " for vehicle: " << vehicle_id;
    info.header.stamp = stamp;

    
    // publisher_.publish(info);
    cloud_infos_pubs_.at(vehicle_id).publish(info);
  }

 private:
  ros::NodeHandle nh_;
  int max_num_;
  ros::Publisher publisher_;
  std::unordered_map<int, ros::Publisher> cloud_infos_pubs_;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_CLOUD_INFO_PUBLISHER_H_