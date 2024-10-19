#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

#include "common/planning/dcp_tree.h"
#include "common/planning/eudm_itf.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "vehicle_msgs/CloudInfo.h"
/**
 * @brief since eudm itself is completely stateless, we use a manager to
 *        track the input and output of the eudm, to satisfy task level
 *        constraints.
 */
namespace planning {
class EudmManager {
 public:
  using DcpLatAction = common::DcpTree::DcpLatAction;
  using DcpLonAction = common::DcpTree::DcpLonAction;
  using DcpAction = common::DcpTree::DcpAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = common::CostStructure;
  using Task = common::Task;
  using Snapshot = common::Snapshot;
  using ReplanningContext = common::ReplanningContext;
  using LaneChangeContext = common::LaneChangeContext;
  using LaneChangeProposal = common::LaneChangeProposal;
  using ActivateLaneChangeRequest = common::ActivateLaneChangeRequest;
  using LaneChangeTriggerType = common::LaneChangeTriggerType;
  using DcpTree = common::DcpTree;

  EudmManager() {}

  void Init(const std::string& config_path, const decimal_t work_rate, semantic_map_manager::SemanticMapManager *smm);

  ErrorType Run(
      const decimal_t stamp,
      const int cur_ego_id,
      const Task& task);

  void Reset();

  void ConstructBehavior(common::SemanticBehavior* behavior);

  ErrorType GetCloudInfoMsg(const int &cur_ego_id, common::SemanticBehavior &behavior, vehicle_msgs::CloudInfo *msg);

  EudmPlanner& planner();

  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }
  semantic_map_manager::EgoSemanticMapManager map() {
    return map_adapter_.map();
  }

  // TODO for MLAD: 对context_，last_snapshot_, lc_context_, last_lc_proposal_, preliminary_active_requests_进行更新。
  // TODO for MLAD: 规划结束时暂不更新，需要等到收到agv反馈后，再和轨迹一起更新
  ErrorType UpdateCurEgoVehicleConfigsAfterPlanning(const int &cur_ego_id);

  ErrorType GetCurEgoVehicleLastConfigs(const int &cur_ego_id);

  inline bool replane_task() const { return replane_task_; }

 private:
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                          const decimal_t& delta);

  bool IsTriggerAppropriate(const LateralBehavior& lat);

  ErrorType Prepare(
      const decimal_t stamp,
      const Task& task);

  ErrorType EvaluateReferenceVelocity(const Task& task,
                                      decimal_t* ref_vel);

  bool GetReplanDesiredAction(const decimal_t current_time,
                              DcpAction* desired_action);

  void SaveSnapshot(Snapshot* snapshot);

  ErrorType ReselectByContext(const decimal_t stamp, const Snapshot& snapshot,
                              int* new_seq_id);

  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                     const Task& task);

  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                       const Task& task);

  semantic_map_manager::SemanticMapManager *smm_;
  EudmPlanner bp_;
  EudmPlannerMapAdapter map_adapter_; // TODO for MLAD: 在这里，将语义地图中相关的信息，全部拿出来，使其在整个行为规划期间，不再改变？
  decimal_t work_rate_{20.0};
  bool replane_task_;

  int ego_lane_id_;
  ReplanningContext context_; // TODO for MLAD: 每次规划时，需要从BehavioralVehicle中拿
  Snapshot last_snapshot_;
  Task last_task_;
  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace planning

#endif