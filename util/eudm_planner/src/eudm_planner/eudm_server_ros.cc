/**
 * @file eudm_server_ros.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, SemanticMapManager *smm)
    : nh_(nh), smm_(smm), work_rate_(20.0) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_); 
  
  //p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      //config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, SemanticMapManager *smm, double work_rate)
    : nh_(nh), smm_(smm), work_rate_(work_rate) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, smm_->max_ego_vehicle_num());
  p_cloud_info_pubs_ = new EudmPlannerPublisher(nh, smm_->max_ego_vehicle_num());
  //p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      //config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
  id_queue = common::PriorityIdQueue::getInstance(); // 单例模式，获取其指针
}

void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm); // TODO for MLAD: 使用生产者消费者模式，现在不能这样做，因为中间涉及到很多其它的车辆的位置更新，不能使用这样的方式，需要改为指针
}

void EudmPlannerServer::PublishData() { // TODO for MLAD: 这里应该是每次规划结束，将当前的结果发送出去
  p_visualizer_->PublishDataWithStamp(ros::Time::now(), cur_ego_id_);
}

void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_, smm_);
  //joy_sub_ = nh_.subscribe("/joy", 10, &EudmPlannerServer::JoyCallback, this); //TODO for MLAD: behavior planning在此处接收向左向右变道的请求，将/joy替换为变道决策模块。需要设置一个哈希表<agvId, 指针>储存所有AGV的behavior_planner
  nh_.param("use_sim_state", use_sim_state_, true);
  p_visualizer_->Init();
  p_cloud_info_pubs_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

void EudmPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // ~ buttons[2] --> 1 -->  lcl
  // ~ buttons[1] --> 1 -->  lcr
  // ~ buttons[3] --> 1 -->  +1m/s
  // ~ buttons[0] --> 1 -->  -1m/s
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  if (msg->buttons[2] == 1) {
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

void EudmPlannerServer::Start() {
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void EudmPlannerServer::PlanCycleCallback() {
  if (smm_ == nullptr || id_queue == nullptr) return;

  if (id_queue->Empty()) {
    // PublishData();
    if(smm_->check_task()){
      std::vector<int> rank;
      for(int i = 1; i <= 8; i++){
        rank.push_back(i); 
      }
      for(int i = 1; i <= 8; i++){
        id_queue->UpdateQueue(rank);
      }
    }
    return;
  }

  int cur_ego_id = id_queue->Front();

  
  if (cur_ego_id==-1) {
    // PublishData();
    return;
  }

  cur_ego_id_ = cur_ego_id;
  
  // TODO for MLAD: 首先调用smm，判断此agv是否需要变道，若不需要，则跳过，否则生成task

  task.is_under_ctrl = true;
  //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] cur_ego_id:" << cur_ego_id;
  // if (!smm_->GetSpecVehicleCurrentTask(cur_ego_id, &task)) {
  //   // 不需要变道
  //   //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] no task";
  //   PublishData();
  //   return;
  // }
  if (!smm_->GetSpecVehicleCurrentTaskfromMsg(cur_ego_id, &task)) {
    // 不需要变道
    //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] no task";
    PublishData();
    return;
  }
  //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] task.user_perferred_behavior:"  << task.user_perferred_behavior;
  decimal_t replan_duration = 1.0 / work_rate_;
  double stamp = std::floor(smm_->time_stamp() / replan_duration) * replan_duration;
  //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] stamp: " << stamp;
  
  if (bp_manager_.GetCurEgoVehicleLastConfigs(cur_ego_id) != kSuccess) {
    //LOG(ERROR) << "[PlanCycleCallback] GetCurEgoVehicleLastConfigs fail.";
    return;
  }
  // TODO for MLAD: 在下面使用行为规划，对指定agv进行规划，并生成对应的msg。在这里进行加锁？需要验证动作接收后，agv是否完成变道且在缓冲区外，若进入缓冲区，则取消本次任务
  if (bp_manager_.Run(stamp, cur_ego_id, task) == kSuccess) {
    //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] bp_manager_.Run(stamp, cur_ego_id, task):"  << task.target_lane_id;
    //LOG(ERROR) << "[EudmPlannerServer::PlanCycleCallback] bp_manager_.Run(stamp, cur_ego_id, task):"  << task.user_perferred_behavior;
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);

    // TODO for MLAD: 在这里调用函数，发送msg，函数需要包含判断确认agv是否收到，同时id_queue是否更新
    vehicle_msgs::CloudInfo msg;
    if (bp_manager_.GetCloudInfoMsg(cur_ego_id, behavior, &msg) == kSuccess) {
      p_cloud_info_pubs_->PublishDataWithStamp(ros::Time::now(), cur_ego_id, msg);
      //LOG(ERROR) << "[PlanCycleCallback] sending data.";
      
      bool wait_reply = true, behavior_confirm = false;
      while (true) {
        if (smm_->GetSpecEgoVehicleReplyComplete(cur_ego_id, &wait_reply, &behavior_confirm) != kSuccess) {
          //LOG(ERROR) << "[PlanCycleCallback] get behavior reply failed.";
          return;
        }
        if (!wait_reply) break;
      }
      //LOG(ERROR) << "[PlanCycleCallback] end waiting for reply.";
      if (behavior_confirm) {
        //LOG(ERROR) << "[PlanCycleCallback] behavior reply confirm. Write back to smm.";
         // 确认agv接收到后，需要将当前的behavior以及agv反馈的轨迹，写入到smm中
        bp_manager_.UpdateCurEgoVehicleConfigsAfterPlanning(cur_ego_id);
        smm_->UpdateSpecEgoVehicleNextTask(cur_ego_id, task);
        smm_->UpdateSpecEgoVehicleNextSemanticBehavior(cur_ego_id, behavior);
        // TODO for MLAD: 當前暫時使用snapshot中的軌跡，在UpdateCurEgoVehicleConfigsAfterPlanning中一起更新
        // smm_->UpdateSpecEgoVehicleNextTraj(cur_ego_id);
        PublishData();
      }
     
    }
  }

  


  
}

void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
