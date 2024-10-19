#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_FORWARD_PLANNER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_FORWARD_PLANNER_H_

#include <ros/ros.h>

#include <memory>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "common/planning/dcp_tree.h"

#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
#include "semantic_map_manager/traffic_signal_manager.h"
#include "common/visualization/common_visualization_util.h"

namespace semantic_map_manager {

class ForwardPlanner {
 public:
 	using ObstacleMapType = uint8_t;
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
	using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
  using DcpTree = common::DcpTree;

	ForwardPlanner();
	~ForwardPlanner() {}

  std::string Name();

  ErrorType RunOnce(const common::Vehicle &ego_vehicle, const int &ego_lane_id_by_pos,
										const common::SemanticVehicleSet &semantic_vehicle_set, const int &aggressive_level, 
										const bool &enable_openloop_prediction, common::SemanticLaneSet semantic_lane_set,
										const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs,
										common::SemanticBehavior *behavior);

  inline void set_sim_resolution(const decimal_t sim_resolution) {
    sim_resolution_ = sim_resolution;
  }

  inline void set_sim_horizon(const decimal_t sim_horizon) {
    sim_horizon_ = sim_horizon;
  }

  inline void set_user_desired_velocity(const decimal_t user_desired_velocity) {
    user_desired_velocity_ = user_desired_velocity;
  }

  inline void set_reference_desired_velocity(const decimal_t reference_desired_velocity) {
    reference_desired_velocity_ = reference_desired_velocity;
  }

  ErrorType RunMpdm();

 protected:
  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LongitudinalBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      const planning::OnLaneForwardSimulation::Param &ego_sim_param,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      const planning::OnLaneForwardSimulation::Param &ego_sim_param,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LongitudinalBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LongitudinalBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>& valid_surround_trajs,
      int *winner_id, decimal_t* winner_score,
      decimal_t* desired_vel);

  ErrorType EvaluateSinglePolicyTraj(
      const LongitudinalBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);

	ErrorType GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle,
    decimal_t *distance_residual_ratio);

	ErrorType CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res);

	ErrorType CheckCollisionUsingStateAndVehicleParam(
    const common::VehicleParam &vehicle_param, const common::State &state,
    bool *res);

	ErrorType CheckCollisionUsingGlobalPosition(
    const Vec2f &p_w, bool *res) const;

  void VisualizeForwardTrajectories();

  Behavior behavior_;

  decimal_t user_desired_velocity_{20.0};
  decimal_t reference_desired_velocity_{20.0};
  decimal_t acc_cmd_vel_gap_{10.0};
  decimal_t dec_cmd_vel_gap_{10.0};
  decimal_t lon_aggressive_ratio_{0.25};
  int autonomous_level_{2};

  decimal_t sim_resolution_{0.3};
  decimal_t sim_horizon_{3.0};
  int aggressive_level_{3};
  planning::OnLaneForwardSimulation::Param sim_param_;

	decimal_t pred_step_ = 0.2;

  bool lock_to_hmi_ = false;
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

	TrafficSignalManager traffic_singal_manager_;

	common::SemanticLaneSet semantic_lane_set_;
  common::SemanticVehicleSet semantic_vehicle_set_;
	common::Vehicle ego_vehicle_;
	common::Lane ego_reflane_;

	GridMap2D obstacle_map_;
	std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs_;
	bool enable_openloop_prediction_;

  // track the ego lane id
  int ego_lane_id_{kInvalidLaneId};
  int ego_id_;
  std::vector<int> potential_lk_lane_ids_;
  // debug
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  int winner_id_;

  // visualize
  // std::string forward_traj_topic_ = std::string("/vis/agent_") +
  //                                    std::to_string(ego_id_) +
  //                                    std::string("/forward_trajs");
  // int last_forward_trajs_marker_cnt_ = 0;
  // ros::Publisher publisher_;
};

}  // namespace planning

#endif