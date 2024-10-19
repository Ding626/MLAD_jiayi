#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_

#include <iostream>
#include <set>
#include <unordered_set>

#include "common/basics/semantics.h"
#include "common/planning/dcp_tree.h"
#include "common/planning/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/ego_semantic_map_manager.h"
#include "vehicle_msgs/encoder.h"
#include "vehicle_msgs/CloudInfo.h"
#include "ros/ros.h"

namespace planning {

class EudmPlannerMapAdapter : public EudmPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  bool IsValid() override;
  ErrorType GetEgoState(State *state) override;
  ErrorType GetEgoId(int *id) override;
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;
  bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) override;
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane);
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) override;
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) override;
  ErrorType GetCloudInfoMsgFromCurrentSemanticBehavior(
      const int ego_vehicle_id, common::SemanticBehavior &behavior, vehicle_msgs::CloudInfo *msg);

	void set_smm(IntegratedMap *smm);

  void set_map(const int ego_vehicle_id);
  // 在这里，从BehavioralVehicle中获取上一次规划时，得到的结果，并转换为ReplanningContext的格式
  ErrorType GetLastReplanningContextForCurEgoVehicle(const int ego_vehicle_id, common::ReplanningContext *context);

	// 在这里，从BehavioralVehicle中获取上一次规划时，进行的任务
  ErrorType GetLastTaskForCurEgoVehicle(const int ego_vehicle_id, common::Task *task);

	// 在这里，从BehavioralVehicle中获取上一次规划时，得到的Snapshot
  ErrorType GetLastSnapshotForCurEgoVehicle(const int ego_vehicle_id, common::Snapshot *snapshot);

	// 在这里，从BehavioralVehicle中获取上一次规划时，得到的LaneChangeProposal
  ErrorType GetLastLaneChangeProposalForCurEgoVehicle(const int ego_vehicle_id, common::LaneChangeProposal *lc_proposal);

	// 在这里，从BehavioralVehicle中获取上一次规划时，得到的preliminary_active_requests
  ErrorType GetLastPreliminaryActiveRequestsForCurEgoVehicle(const int ego_vehicle_id, std::vector<common::ActivateLaneChangeRequest> *preliminary_active_requests);

	// 在这里，从BehavioralVehicle中获取上一次规划时，获取之前的LaneChangeContext
  ErrorType GetLastLaneChangeContextForCurEgoVehicle(const int ego_vehicle_id, common::LaneChangeContext *lc_context);

  semantic_map_manager::EgoSemanticMapManager map() { return map_; }

	// NOTE for MLAD: 获取所有BehavioralKeyVehicle的id
	ErrorType GetBehavioralKeyVehiclesIds(std::unordered_set<int> *behavioral_key_vehicle_ids);

	// NOTE for MLAD: 获取指定时刻时，agv是否正在执行规划好的轨迹
	ErrorType IsBehavioralVehicleRunningPlannedTraj(const int &behavioral_key_vehicle_id, const decimal_t &time, bool *is_running);

	// NOTE for MLAD: 获取agv上传轨迹还未执行结束的agv，在指定时刻的状态
	ErrorType GetRunningBehavioralVehicleStateAtSpecTime(const int &behavioral_key_vehicle_id, const decimal_t &time, common::State *state_output);

  ErrorType UpdateCurEgoVehiclePlanningConfigs(const int &ego_vehicle_id, const common::ReplanningContext &context, const common::Snapshot &snapshot, 
      const common::LaneChangeProposal &lc_proposal, const common::LaneChangeContext &lc_context, 
      const std::vector<common::ActivateLaneChangeRequest> &preliminary_active_requests);

  ErrorType IsEgoPriorityHigher(const int &ego_vehicle_id, const int &compare_id, bool *is_ego_prior);
  
 private:
	IntegratedMap *smm_; // TODO for MLAD: 需要加一个函数，即给出当前的id，以及传入一个EgoSemanticMapManager指针，修改此指针的值
  semantic_map_manager::EgoSemanticMapManager map_; // TODO for MLAD: 原始是一个共享指针，这里需要改为一个类，每次对不同的车辆，去修改其中的值
	bool is_smm_init_ = false;
  bool is_valid_ = false;
};

}  // namespace planning

#endif