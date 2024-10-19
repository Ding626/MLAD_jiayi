#include "semantic_map_manager/ego_ros_adapter.h"

namespace semantic_map_manager {

void EgoRosAdapter::Init() {
  // communicate with phy simulator
  {

    arena_info_static_sub_ = nh_.subscribe(
        "arena_info_static", 2, &EgoRosAdapter::ArenaInfoStaticCallback, this,
        ros::TransportHints().tcpNoDelay());
    arena_info_dynamic_sub_ = nh_.subscribe(
        "arena_info_dynamic", 2, &EgoRosAdapter::ArenaInfoDynamicCallback, this,
        ros::TransportHints().tcpNoDelay());

    if (receive_cloud_) {
      cloud_info_sub_ = nh_.subscribe(
        "/local/agent_" + std::to_string(ego_vehicle_id_) + "/cloud_info_topic", 1, 
        &EgoRosAdapter::CloudInfoCallback, this, ros::TransportHints().tcpNoDelay());
      
    }
  }
}

// ! DEPRECATED (@lu.zhang)
void EgoRosAdapter::ArenaInfoCallback(
    const vehicle_msgs::ArenaInfo::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                           obstacle_set_);
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

void EgoRosAdapter::ArenaInfoStaticCallback(
    const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;
}

void EgoRosAdapter::ArenaInfoDynamicCallback(
    const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_);
  
  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                             obstacle_set_);
    if (has_callback_binded_) {
      private_callback_fn_(*p_smm_);
    }
  }
}

void EgoRosAdapter::CloudInfoCallback(
    const vehicle_msgs::CloudInfo::ConstPtr& msg) {
  ros::Time time_stamp;
  std::string uid;
  vehicle_msgs::Decoder::GetCloudDataFromCloudInfo(
      *msg, &time_stamp, &behavior_, &uid);
  p_data_renderer_->Render(time_stamp.toSec(), behavior_, uid); // TODO for MLAD: 这里会有多线程安全问题：ego_smm在更新behavior和uid时，被前面加入到queue中了。应该如何解决？？
  
  // if (has_callback_binded_) {
  //   private_callback_fn_(*p_smm_);
  // }
}

void EgoRosAdapter::BindMapUpdateCallback(
    std::function<int(const EgoSemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager