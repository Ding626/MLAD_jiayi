#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_EGO_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_EGO_ROS_ADAPTER_H_

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "ros/ros.h"
#include "semantic_map_manager/ego_data_renderer.h"
#include "vehicle_msgs/decoder.h"
#include "semantic_map_manager/ego_semantic_map_manager.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

namespace semantic_map_manager {

class EgoRosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  EgoRosAdapter() {}
  EgoRosAdapter(ros::NodeHandle nh, EgoSemanticMapManager* ptr_smm, bool receive_cloud) : nh_(nh), receive_cloud_(receive_cloud) {
    p_smm_ = ptr_smm;
    p_data_renderer_ = new EgoDataRenderer(ptr_smm);
  }
  EgoRosAdapter(ros::NodeHandle nh, EgoSemanticMapManager* ptr_smm, const bool &receive_cloud, const int &ego_vehicle_id) : 
                    nh_(nh), receive_cloud_(receive_cloud), ego_vehicle_id_(ego_vehicle_id) {
    p_smm_ = ptr_smm;
    p_data_renderer_ = new EgoDataRenderer(ptr_smm);
  }
  EgoRosAdapter(ros::NodeHandle nh, EgoSemanticMapManager* ptr_smm) : nh_(nh){
    p_smm_ = ptr_smm;
    p_data_renderer_ = new EgoDataRenderer(ptr_smm);
  }
  ~EgoRosAdapter() {}

  void BindMapUpdateCallback(std::function<int(const EgoSemanticMapManager&)> fn);

  void Init();

 private:
  // ! DEPRECATED (@lu.zhang)
  void ArenaInfoCallback(const vehicle_msgs::ArenaInfo::ConstPtr& msg);

  void ArenaInfoStaticCallback(
      const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg);
  void ArenaInfoDynamicCallback(
      const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg);
  void CloudInfoCallback(const vehicle_msgs::CloudInfo::ConstPtr& msg);
  ros::NodeHandle nh_;

  // communicate with phy simulator
  ros::Subscriber arena_info_sub_;
  ros::Subscriber arena_info_static_sub_;
  ros::Subscriber arena_info_dynamic_sub_;
  ros::Subscriber cloud_info_sub_;

  common::Vehicle ego_vehicle_;
  common::VehicleSet surrounding_vehicles_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;
  common::SemanticBehavior behavior_;

  EgoDataRenderer* p_data_renderer_;
  EgoSemanticMapManager* p_smm_;

  bool get_arena_info_static_ = false;

  bool receive_cloud_ = false;

  int ego_vehicle_id_{0};

  bool has_callback_binded_ = false;
  std::function<int(const EgoSemanticMapManager&)> private_callback_fn_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_