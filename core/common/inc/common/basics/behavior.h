/**
 * @file behavior.h
 * @author Yijun Mao
 * @brief
 * @version 0.1
 * @date 2022-3-11
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_BASICS_BEHAVIOR_H_
#define _CORE_SEMANTIC_MAP_INC_BASICS_BEHAVIOR_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>

#include "common/basics/basics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/free_state.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/basics/semantics.h"
#include "common/planning/dcp_tree.h"
#include "common/planning/eudm_itf.h"

namespace common {
/**
 * @brief Vehicle with behavior info
 * 主要是储存其上一轮给出的语义动作、与变道决策目标车道等等（TODO: 需要留意还有哪些需求，可以通过BehavioralVehicle实现
 */
class BehavioralVehicle {
 public:
  // using DcpAction = planning::DcpTree::DcpAction;
  // using CostStructure = planning::EudmPlanner::CostStructure;
  // using Vehicle = common::Vehicle;
  // using State = common::State;
  // using SemanticBehavior = common::SemanticBehavior;
  // using LaneRaw = common::LaneRaw;
  // using Lane = common::Lane;

  BehavioralVehicle();
  BehavioralVehicle(const Vehicle &vehicle);
  ~BehavioralVehicle() {} //TODO for MLAD: 若有指针，需要添加析构函数

  void InitializeVehicle(const Vehicle &vehicle);
  ErrorType UpdateVehicle(const Vehicle &vehicle);
  ErrorType UpdateNextTask(const Task &task);
  ErrorType UpdateNextSemanticBehavior(const SemanticBehavior &behavior);
  ErrorType UpdateNextTraj(const vec_E<State> &state_traj, const std::string &updated_behavior_uid);
  ErrorType UpdateNextLaneChangeContext(const LaneChangeContext &lc_context);
  ErrorType UpdateNextReplanningContext(const ReplanningContext &context);
  ErrorType UpdateNextSnapshotForPlanning(const Snapshot &snapshot);
  ErrorType UpdateNextLaneChangeProposal(const LaneChangeProposal &lc_proposal);
  ErrorType UpdateNextPreliminaryActiveRequests(const std::vector<ActivateLaneChangeRequest> &preliminary_active_requests);

  ErrorType GetLastReplanningContext(ReplanningContext *context);
  ErrorType GetLastTask(Task *task);
  ErrorType GetLastSnapshotForPlanning(Snapshot *snapshot);
  ErrorType GetLastLaneChangeProposal(LaneChangeProposal *lc_proposal);
  ErrorType GetLastPreliminaryActiveRequests(std::vector<ActivateLaneChangeRequest> *preliminary_active_requests);
  ErrorType GetLastLaneChangeContext(LaneChangeContext *lc_context);
  bool IsRunningPlannedTrajAtSpecTime(const decimal_t &time);
  ErrorType GetExecutingStateAtSpecTime(const decimal_t &time, State *state_output);
  // 获取此刻生成的behavior的uid和生成的时间
  ErrorType GetLastBehaviorUIDWithTime(std::string *behavior_uid, ros::Time *time_stamp);

  inline int nearest_lane_id() const { return nearest_lane_id_; }
  inline Vehicle vehicle() const { return vehicle_; }
  inline int target_lane_id() const { return target_lane_id_; }
  inline decimal_t dist_to_lane() const { return dist_to_lane_; }
  inline decimal_t arc_len_onlane() const { return arc_len_onlane_; }
  inline bool is_task_init() const { return last_task_.is_under_ctrl; }
  inline std::string updated_behavior_uid() {
    // std::lock_guard<std::mutex> lock_guard(updated_behavior_uid_mtx_); 
    return updated_behavior_uid_;
  }
  inline std::string behavior_uid() const { 
    return behavior_uid_; 
  }
  inline int conflict_leading_uncertain_vehicle_id() const { return conflict_leading_uncertain_vehicle_id_; }
  inline decimal_t opp_uncertain_vehicle_dist() const { return opp_uncertain_vehicle_dist_; }
  inline int lane_drop_conflict_lane_id() const { return lane_drop_conflict_lane_id_; }

  inline void set_nearest_lane_id(const int &nearest_lane_id) { nearest_lane_id_ = nearest_lane_id; }
  inline void set_target_lane_id(const int &target_lane_id) { target_lane_id_ = target_lane_id; }
  inline void set_dist_to_lane(const decimal_t &dist_to_lane) { dist_to_lane_ = dist_to_lane; }
  inline void set_arc_len_onlane(const decimal_t &arc_len_onlane) { arc_len_onlane_ = arc_len_onlane; }
  inline void set_behavior_uid(const std::string &uid) { 
    behavior_uid_ = uid; 
    last_behavior_time_stamp_ = ros::Time::now();
  }
  inline void set_conflict_leading_uncertain_vehicle_id(const int &uncertain_vehicle_id) { conflict_leading_uncertain_vehicle_id_ = uncertain_vehicle_id; }
  inline void set_opp_uncertain_vehicle_dist(const decimal_t &distance) { opp_uncertain_vehicle_dist_ = distance; }
  inline void set_lane_drop_conflict_lane_id(const int &lane_drop_conflict_lane_id) { lane_drop_conflict_lane_id_ = lane_drop_conflict_lane_id; }

 private:
	Vehicle vehicle_;

  int target_lane_id_{kInvalidLaneId};

  vec_E<State> motion_planned_forward_state_traj_; // vehicle上传上来的实际的轨迹
	
  // * nearest lane info
  int nearest_lane_id_{kInvalidLaneId};
  decimal_t dist_to_lane_{-1.0};
  decimal_t arc_len_onlane_{-1.0};

  // // * prediction
  // ProbDistOfLatBehaviors probs_lat_behaviors_;// TODO for MLAD: 这里不应该有

  // * argmax behavior
  LateralBehavior lat_behavior_{LateralBehavior::kUndefined};
  LongitudinalBehavior lon_behavior_;

  // * SemanticBehavior
  vec_E<vec_E<Vehicle>> behavior_forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  decimal_t actual_desired_velocity_{0.0};

  Task last_task_;

  bool is_last_action_valid_; // TODO for MLAD: 默认为false，当有上一次规划动作时，改为true
  decimal_t last_action_seq_start_time_;
  std::vector<DcpAction> action_seq_; // TODO for MLAD: 这个和forward_behaviors基本一样，需要合并

  // Snapshot
  bool last_snapshot_valid_ = false; // 默认为false，当有上一次规划动作时，改为true
  int original_winner_id_;
  int processed_winner_id_;
  State plan_state_;
  std::vector<std::vector<DcpAction>> action_script_;
  std::vector<bool> sim_res_;
  std::vector<bool> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<decimal_t> final_cost_;
  std::vector<std::vector<CostStructure>> progress_cost_;
  std::vector<CostStructure> tail_cost_;
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs_;
  Lane ref_lane_;
  int ref_lane_id_;

  double plan_stamp_ = 0.0;
  double time_cost_ = 0.0;

  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
  
  // uid of last behavior in cloud info
  std::string behavior_uid_ = ""; // NOTE for MLAD: 由于涉及到behavior uid的部分，都在ros_server中，是顺序执行且都在一个线程内部，按理说互斥锁不需要
  std::string updated_behavior_uid_ = ""; // 上报update的轨迹的对应的behavior的uid
  // std::mutex updated_behavior_uid_mtx_; 
  ros::Time last_behavior_time_stamp_;

  int conflict_leading_uncertain_vehicle_id_ = kInvalidAgentId; // 和当前vehicle有相遇冲突的uncertain vehicle的id。默认为kInvalidAgentId，表示没有相遇冲突
  decimal_t opp_uncertain_vehicle_dist_ = kInf; // 如果有相遇冲突，与前方反向的uncertain vehicle的距离。默认为kInf
  int lane_drop_conflict_lane_id_ = kInvalidLaneId; // 如果本agent检测到发生lane drop，则此时发生lane drop的车道的id。默认表示没有
  // std::mutex traj_update_mutex_; // 轨迹的读写锁
};

struct BehavioralVehicleSet {
  std::unordered_map<int, BehavioralVehicle> behavioral_vehicles;
};
struct TaskSet {
  std::unordered_map<int, Task> lc_tasks;
};
/**
 * @brief lane with behavior info
 * 主要是储存当前车道上，agv的数量、当前通行方向等等（TODO: 需要留意还有哪些需求，可以通过BehavioralVehicle实现
 */
class BehavioralLane {
 public:
  BehavioralLane();
  BehavioralLane(const LaneRaw &raw_lane);
  //~BehavioralLane();

  // inline LaneRaw raw_lane() const { return raw_lane_; }
  inline int id() const { return id_; }
  inline decimal_t dir() const { return dir_; }
  inline std::vector<int> child_id() const { return child_id_; }
  inline std::vector<int> father_id() const { return father_id_; }
  inline int l_lane_id() const { return l_lane_id_; }
  inline int r_lane_id() const { return r_lane_id_; }
  inline int group() const { return group_; }
  inline bool l_change_avbl() const { return l_change_avbl_; }
  inline bool r_change_avbl() const { return r_change_avbl_; }
  inline decimal_t length() const { return length_; }
  inline decimal_t buffer() const { return buffer_; }
  inline bool cur_pass_dir_straight() const { return cur_pass_dir_straight_; }
  inline bool dir_changeable() const { return dir_changeable_; }
  inline int real_ego_vehicle_num() const { return real_ego_vehicle_num_; }
  inline int real_surrounding_vehicle_num() const { return real_surrounding_vehicle_num_; }
  inline int exp_ego_vehicle_num() const { return exp_ego_vehicle_num_; }
  inline int exp_surrounding_vehicle_num() const { return exp_surrounding_vehicle_num_; }
  inline int uncertain_vehicle_num() const { return uncertain_vehicle_num_; }

  inline int real_key_vehicle_num() const { return real_ego_vehicle_num_ + real_surrounding_vehicle_num_; }
  inline int exp_key_vehicle_num() const { return exp_ego_vehicle_num_ + exp_surrounding_vehicle_num_; }
  inline bool opp_uncertain_vehicle_encounter_conflict() const { return opp_uncertain_vehicle_encounter_conflict_; }
  inline bool lane_drop_conflict() const { return lane_drop_conflict_; }

  inline void reverse_cur_pass_dir_straight() { cur_pass_dir_straight_ = !cur_pass_dir_straight_; }
  
  inline void set_cur_pass_dir_straight(const bool &is_straight) { cur_pass_dir_straight_ = is_straight; }
  inline void set_real_ego_vehicle_num(const int &real_ego_vehicle_num) { real_ego_vehicle_num_ = real_ego_vehicle_num; }
  inline void set_real_surrounding_vehicle_num(const int &real_surrounding_vehicle_num) { real_surrounding_vehicle_num_ = real_surrounding_vehicle_num; }
  inline void set_exp_ego_vehicle_num(const int &exp_ego_vehicle_num) { exp_ego_vehicle_num_ = exp_ego_vehicle_num; }
  inline void set_exp_surrounding_vehicle_num(const int &exp_surrounding_vehicle_num) { exp_surrounding_vehicle_num_ = exp_surrounding_vehicle_num; }
  inline void set_uncertain_vehicle_num(const int &uncertain_vehicle_num) { uncertain_vehicle_num_ = uncertain_vehicle_num; }
  inline void set_opp_uncertain_vehicle_encounter_conflict(const bool &opp_uncertain_vehicle_encounter_conflict) { opp_uncertain_vehicle_encounter_conflict_ = opp_uncertain_vehicle_encounter_conflict; }
  inline void set_lane_drop_conflict(const bool &lane_drop_conflict) { lane_drop_conflict_ = lane_drop_conflict; }
  
  void Initialize(const LaneRaw &raw_lane);

 private:

	// Lane lane_;
  int id_;
  decimal_t dir_; // 车道的主方向，初始化后不会改变

  std::vector<int> child_id_;
  std::vector<int> father_id_;

  int l_lane_id_;
  bool l_change_avbl_;
  int r_lane_id_;
  bool r_change_avbl_;
  bool dir_changeable_; // 此车道的通行方向是否是可变的

  std::string behavior_; // 路径的属性

  decimal_t length_;
  decimal_t buffer_;

  int group_; // 多径通道的组号

  // NOTE for MLAD: 需要确定一个主方向，load路径时标号顺序的左右，是以主方向为准的
  // NOTE for MLAD: 在主方向下，0代表最左边的车道，越大代表越靠右的车道，json文件中车道为-1表示不存在车道
  bool cur_pass_dir_straight_; // 当前车道的通行方向是否和主方向一致

  int real_ego_vehicle_num_; //当前实际的可参与变道agv的数量
  int real_surrounding_vehicle_num_; // 当前实际的不可参与变道agv的数量
  int exp_ego_vehicle_num_; // 预期的可参与变道agv的数量
  int exp_surrounding_vehicle_num_; // 预期的不可参与变道agv的数量
  int uncertain_vehicle_num_; // 非合作agv的数量

  bool opp_uncertain_vehicle_encounter_conflict_; // 当前车道上，是否有相遇冲突的情况发生（相遇冲突指非合作agv在合作agv同车道前方指定距离内相向行驶）
  bool lane_drop_conflict_; // 当前车道是否被检测到会消失，即前方出现lane-drop bottleneck
};

struct BehavioralLaneSet {
  BehavioralLaneSet();
  BehavioralLaneSet(const LaneNet &whole_lane_net);

  /**
   * @brief Return the size of container
   *
   * @return int size
   */
  inline int size() const { return behavioral_lanes.size(); }

  /**
   * @brief Return the size of container
   *
   * @return int size
   */
  void Initialize(const LaneNet &whole_lane_net);

  /**
   * @brief Clear the container
   */
  void clear() { behavioral_lanes.clear(); }

  /**
   * @brief Print info
   */
  void print() const;

  /**
   * @brief Get all lanes id
   */
  inline std::vector<int> GetAllBehavioralLaneIds() const { return all_lane_ids; }
 //private:
  std::unordered_map<int, BehavioralLane> behavioral_lanes;
  std::vector<int> all_lane_ids;
};

}

#endif //_CORE_SEMANTIC_MAP_INC_BASICS_BEHAVIOR_H_