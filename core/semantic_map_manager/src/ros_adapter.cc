#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // communicate with phy simulator
  {
    // arena_info_sub_ =
    //     nh_.subscribe("arena_info", 2, &RosAdapter::ArenaInfoCallback, this,
    //                   ros::TransportHints().tcpNoDelay());
    arena_info_static_sub_ = nh_.subscribe(
        "arena_info_static", 2, &RosAdapter::ArenaInfoStaticCallback, this,
        ros::TransportHints().tcpNoDelay()); // TODO for MLAD: 通过订阅此消息，获知所有车道等信息，是否需要改为，服务端的配置文件直接读取？
    arena_info_dynamic_sub_ = nh_.subscribe(
        "arena_info_dynamic", 2, &RosAdapter::ArenaInfoDynamicCallback, this,
        ros::TransportHints().tcpNoDelay()); // TODO for MLAD: 通过订阅此消息，来得到所有agv的位置、速度等等情况。对semantic_map_manager，需要对所有的vehicle设置一个读写互斥量，在这里收到更新后，需要占用并更新map_manager中储存的，释放后，才能被优先级判定模块和行为决策规划模块获得
        // 此回调函数是被动的，所以需要对semantic_map_manager中的信息设置锁，每时每刻更新所有AGV的信息
        // 发送位置的节点，每隔一段时间，就发送所有车的信息，用锁来读写每一辆车，让每一辆车根据实际上报的信息，更新位置
    task_info_sub_ = nh_.subscribe(
        "task_info", 2, &RosAdapter::VehicleTaskCallback, this,
        ros::TransportHints().tcpNoDelay());
    for (int i=0; i<p_smm_->max_ego_vehicle_num(); i++) {
      std::string traj_update_info_topic = std::string("/local/agent_") +
                                           std::to_string(i) +
                                           std::string("/traj_update_info_topic");
      _vehicle_traj_sub_set[i] = nh_.subscribe(
          traj_update_info_topic, 2, &RosAdapter::VehicleTrajectoryCallback, this,
          ros::TransportHints().tcpNoDelay());
    }
    
  }
}

// ! DEPRECATED (@lu.zhang)
// void RosAdapter::ArenaInfoCallback(
//     const vehicle_msgs::ArenaInfo::ConstPtr& msg) {
//   ros::Time time_stamp;
//   vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
//       *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
//   p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
//                            obstacle_set_);
//   if (has_callback_binded_) {
//     private_callback_fn_(*p_smm_);
//   }
// }

void RosAdapter::ArenaInfoStaticCallback(
    const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(
    const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg) { // msg中的vehicle需要指出自己是不是合作agv，
  ros::Time time_stamp;
  
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_); // TODO for MLAD: 这里msg要传指针？

  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                             obstacle_set_);
    // if (has_callback_binded_) {
    //   private_callback_fn_(*p_smm_);
    // }
  }
}

void RosAdapter::VehicleTaskCallback(
    const vehicle_msgs::TaskSet::ConstPtr& msg) {
  ros::Time time_stamp;
  if(fleet_task_id != msg->fleet_task_id){
    vehicle_msgs::Decoder::GetTaskFromTaskSet(*msg, &time_stamp, &task_set_);
    p_data_renderer_->UpdateCurrentTask(task_set_);
    fleet_task_id = msg->fleet_task_id;
    //LOG(ERROR) << "[VehicleTaskCallback] task updated \n";
  }

  //LOG(ERROR) << "[VehicleTaskCallback] function in \n";
}

void RosAdapter::VehicleTrajectoryCallback(
      const vehicle_msgs::TrajUpdateInfo::ConstPtr& msg) {
  ros::Time time_stamp;

  int ego_id;
  std::string behavior_uid;
  vec_E<common::State> traj;
  vehicle_msgs::Decoder::GetUpdatedTrajFromTrajUpdateInfo(*msg, &time_stamp, &traj, &behavior_uid, &ego_id);

  p_data_renderer_->UpdateExecutingTrajectory(time_stamp.toSec(), ego_id, behavior_uid, traj);
  //add update task
}

// void RosAdapter::VehicleTaskCallback(
//       const vehicle_msgs::TaskSet::ConstPtr& msg) {
//   ros::Time time_stamp;

//   int ego_id;
//   std::string behavior_uid;
//   vec_E<common::State> traj;
//   //vehicle_msgs::Decoder::GetUpdatedTrajFromTrajUpdateInfo(*msg, &time_stamp, &traj, &behavior_uid, &ego_id);
//   vehicle_msgs::Decoder::GetTaskFromTaskSet(*msg, &time_stamp, &task);
//   p_data_renderer_->UpdateCurrentTask(time_stamp.toSec(), ego_id, behavior_uid, traj);
//   //add update task
// }

void RosAdapter::BindMapUpdateCallback(
    std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager