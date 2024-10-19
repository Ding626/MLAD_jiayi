#ifndef _CORE_COMMON_INC_PLANNING_EUDM_ITF_H_
#define _CORE_COMMON_INC_PLANNING_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/planning/dcp_tree.h"
#include "common/lane/lane.h"

namespace common {
using DcpAction = DcpTree::DcpAction;

struct LaneChangeInfo {
  bool forbid_lane_change_left = false;
  bool forbid_lane_change_right = false;
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
  int cur_lane_id = kInvalidLaneId;
  int target_lane_id = kInvalidLaneId;

  void Reset() {
    forbid_lane_change_left = false;
    forbid_lane_change_right = false;
    lane_change_left_unsafe_by_occu = false;
    lane_change_right_unsafe_by_occu = false;
    left_solid_lane = false;
    right_solid_lane = false;
    recommend_lc_left = false;
    recommend_lc_right = false;
    cur_lane_id = kInvalidLaneId;
    target_lane_id = kInvalidLaneId;
  }
};

struct Task {
  bool is_under_ctrl = false;
  double user_desired_vel = 0;
  int user_perferred_behavior = 0; // 1为向right右变道，-1为向left变道
  LaneChangeInfo lc_info;

  int target_lane_id{kInvalidLaneId};

  void Reset() {
    is_under_ctrl = false;
    user_desired_vel = 0;
    user_perferred_behavior = 0;
    lc_info.Reset();
    target_lane_id = kInvalidLaneId;
  }
};

struct ReplanningContext {
  bool is_valid = false;
  decimal_t seq_start_time;
  std::vector<DcpAction> action_seq;

  void Reset() {
    is_valid = false;
    seq_start_time = 0;
    action_seq.clear();
  }
};

struct ActivateLaneChangeRequest {
  decimal_t trigger_time;
  decimal_t desired_operation_time;
  int ego_lane_id;
  LateralBehavior lat = LateralBehavior::kLaneKeeping;
};

struct LaneChangeProposal {
  bool valid = false;
  decimal_t trigger_time = 0.0;
  decimal_t operation_at_seconds = 0.0;
  int ego_lane_id;
  LateralBehavior lat = LateralBehavior::kLaneKeeping;

  void Reset() {
    valid = false;
    trigger_time = 0.0;
    operation_at_seconds = 0.0;
    lat = LateralBehavior::kLaneKeeping;
  }
};

enum class LaneChangeTriggerType { kStick = 0, kActive };

struct LaneChangeContext {
  bool completed = true;
  bool trigger_when_appropriate = false;
  decimal_t trigger_time = 0.0;
  decimal_t desired_operation_time = 0.0;
  int ego_lane_id = 0;
  LateralBehavior lat = LateralBehavior::kLaneKeeping;
  LaneChangeTriggerType type;

  void Reset() {
    completed = true;
    trigger_when_appropriate = false;
    trigger_time = 0.0;
    desired_operation_time = 0.0;
    ego_lane_id = 0;
    lat = LateralBehavior::kLaneKeeping;
  }
};

struct EfficiencyCost {
  decimal_t ego_to_desired_vel = 0.0;
  decimal_t leading_to_desired_vel = 0.0;
  decimal_t ave() const {
    return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
  }
};

struct SafetyCost {
  decimal_t rss = 0.0;
  decimal_t occu_lane = 0.0;
  decimal_t ave() const { return (rss + occu_lane) / 2.0; }
};

struct NavigationCost {
  decimal_t lane_change_preference = 0.0;
  decimal_t ave() const { return lane_change_preference; }
};

struct CostStructure {
  // * associate cost with micro action using this index
  int valid_sample_index_ub;
  // * efficiency
  EfficiencyCost efficiency;
  // * safety
  SafetyCost safety;
  // * navigation
  NavigationCost navigation;
  decimal_t weight = 1.0;
  decimal_t ave() const {
    return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const CostStructure& cost) {
    os << std::fixed;
    os << std::fixed;
    os << std::setprecision(3);
    os << "(efficiency: "
        << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
        << cost.efficiency.leading_to_desired_vel << "), safety: ("
        << cost.safety.rss << "," << cost.safety.occu_lane
        << "), navigation: " << cost.navigation.lane_change_preference << ")";
    return os;
  }
};

struct Snapshot {
  bool valid = false;
  int original_winner_id;
  int processed_winner_id;
  State plan_state;
  std::vector<std::vector<DcpAction>> action_script;
  std::vector<bool> sim_res;
  std::vector<bool> risky_res;
  std::vector<std::string> sim_info;
  std::vector<decimal_t> final_cost;
  std::vector<std::vector<CostStructure>> progress_cost;
  std::vector<CostStructure> tail_cost;
  vec_E<vec_E<Vehicle>> forward_trajs;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;
  Lane ref_lane;
  int ref_lane_id;

  double plan_stamp = 0.0;
  double time_cost = 0.0;

  void Reset() {
    valid = false;
    action_script.clear();
    sim_res.clear();
    risky_res.clear();
    sim_info.clear();
    final_cost.clear();
    progress_cost.clear();
    tail_cost.clear();
    forward_trajs.clear();
    forward_lat_behaviors.clear();
    forward_lon_behaviors.clear();
    surround_trajs.clear();
    plan_stamp = 0.0;
    time_cost = 0.0;
  }
};

} 

#endif
