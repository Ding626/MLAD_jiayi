#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <math.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "ros/ros.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/basics/behavior.h"
#include "common/basics/id_queue.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/math/calculations.h"
#include "common/mobil/mobil_behavior_prediction.h"
#include "common/rss/rss_checker.h"
#include "common/planning/dcp_tree.h"
#include "common/planning/eudm_itf.h"
#include "motion_predictor/onlane_fs_predictor.h"
#include "semantic_map_manager/config_loader.h"
#include "semantic_map_manager/traffic_signal_manager.h"
#include "semantic_map_manager/ego_semantic_map_manager.h"
#include "semantic_map_manager/behavioral_lane_visualizer.h"
#include "vehicle_model/idm_model.h"
#include "nanoflann/include/nanoflann.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "roguelike_ray_casting/roguelike_ray_casting.h"

namespace semantic_map_manager {

class SemanticMapManager {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
  using State = common::State;
  using Lane = common::Lane;
  using LateralBehavior = common::LateralBehavior;
  using SemanticLane = common::SemanticLane;

  SemanticMapManager() {}
  SemanticMapManager(ros::NodeHandle nh, 
                     const std::string &agent_config_path, 
                     const std::string &lane_config_path);
  SemanticMapManager(ros::NodeHandle nh, 
                     const std::string &agent_config_path, 
                     const std::string &lane_config_path,
                     const decimal_t &uncertain_avoid_replan_interval,
                     const decimal_t &change_lane_threshold,
                     const decimal_t &max_wait_update_time);
  // SemanticMapManager(const int &id, const std::string &agent_config_path);
  // SemanticMapManager(const int &id, const decimal_t surrounding_search_radius,
  //                    bool enable_openloop_prediction, bool use_right_hand_axis);
  // SemanticMapManager(const vector<int> &ids, const std::string &agent_config_path);
  // SemanticMapManager(const vector<int> &ids, const decimal_t surrounding_search_radius,
  //                    bool enable_openloop_prediction, bool use_right_hand_axis);
  
  // SemanticMapManager(const std::string &agent_config_path);
  // SemanticMapManager(const decimal_t surrounding_search_radius,
  //                    bool enable_openloop_prediction, bool use_right_hand_axis); // Tips: 初始化时没有任何agv，根据render的上报，来获取agv

  ~SemanticMapManager() {}

  // ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,
  //                                             bool *res) const;

  // ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,
  //                                                  ObstacleMapType *res);

  // ErrorType CheckCollisionUsingStateAndVehicleParam(
  //     const common::VehicleParam &vehicle_param, const common::State &state,
  //     bool *res);

  // ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
  //                                    const common::State &state_a,
  //                                    const common::VehicleParam &param_b,
  //                                    const common::State &state_b, bool *res);

  ErrorType CheckCollisionUsingStateVec(
      const vec_E<common::State> state_vec) const;

  ErrorType GetDistanceToLanesUsing3DofState(
      const Vec3f &state,
      std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const;

  // ErrorType UpdateSemanticMap(
  //     const double &time_stamp, const common::Vehicle &ego_vehicle,
  //     const common::LaneNet &whole_lane_net,
  //     const common::LaneNet &surrounding_lane_net,
  //     const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
  //     const std::set<std::array<decimal_t, 2>> &obstacle_grids,
  //     const common::VehicleSet &surrounding_vehicles);

  ErrorType UpdateSemanticMap(
      const double &time_stamp, const common::VehicleSet &key_vehicles,
      const common::LaneNet &whole_lane_net,
      const common::ObstacleSet &obstacle_set,
      const common::VehicleSet &uncertain_vehicles,
      const bool &key_vehicle_change);

  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) const;

  ErrorType NaiveRuleBasedLateralBehaviorPrediction(
      const common::Vehicle &vehicle, const int nearest_lane_id,
      common::ProbDistOfLatBehaviors *lat_probs);

  ErrorType MobilRuleBasedBehaviorPrediction(
      const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
      common::ProbDistOfLatBehaviors *res);

  // ErrorType TrajectoryPredictionForVehicle(const common::Vehicle &vehicle,
  //                                          const common::Lane &lane,
  //                                          const decimal_t &t_pred,
  //                                          const decimal_t &t_step,
  //                                          vec_E<common::State> *traj);

  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) const;

  ErrorType GetRefLaneForStateByBehavior(const common::State &state,
                                         const std::vector<int> &navi_path,
                                         const LateralBehavior &behavior,
                                         const decimal_t &max_forward_len,
                                         const decimal_t &max_back_len,
                                         const bool is_high_quality,
                                         common::Lane *lane) const;
  ErrorType GetRefLaneForStateByBehavior(const common::State &state,
                                         const std::string &vehicle_type, 
                                         const std::vector<int> &navi_path,
                                         const LateralBehavior &behavior,
                                         const decimal_t &max_forward_len,
                                         const decimal_t &max_back_len,
                                         const bool is_high_quality,
                                         common::Lane *lane) const;
  ErrorType GetTargetLaneId(const int lane_id, const LateralBehavior &behavior,
                            int *target_lane_id) const;

  ErrorType GetLocalLaneSamplesByState(const common::State &state,
                                       const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) const;
  ErrorType GetLeadingVehicleOnLane(const common::Lane &ref_lane,
                                    const common::State &ref_state,
                                    const common::VehicleSet &vehicle_set,
                                    const decimal_t &lat_range,
                                    common::Vehicle *leading_vehicle,
                                    decimal_t *distance_residual_ratio) const;
  ErrorType GetFollowingVehicleOnLane(const common::Lane &ref_lane,
                                      const common::State &ref_state,
                                      const common::VehicleSet &vehicle_set,
                                      const decimal_t &lat_range,
                                      common::Vehicle *leading_vehicle) const;

  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) const;

  ErrorType SaveMapToLog();

  bool IsLocalLaneContainsLane(const int &local_lane_id,
                               const int &seg_lane_id) const;

  // ErrorType GetSpeedLimit(const State &state, const Lane &lane,
  //                         decimal_t *speed_limit) const;

  // ErrorType GetTrafficStoppingState(const State &state, const Lane &lane,
  //                                   State *stopping_state) const;

  // ErrorType GetEgoNearestLaneId(int *ego_lane_id) const;

  ErrorType GetVehicleNearestLaneId(const common::State &state, int *ego_lane_id) const;

  // TODO for MLAD: 需要找出，对当前进行规划的agv，需要考虑哪些其它的agv，并找出所有需要考察的agv的id
  ErrorType GetInvestigatedVehicleIdsForTargetVehicle(const int &ego_id, 
                                      std::vector<int> &investigated_ids);

  // TODO for MLAD: 需要找出，对当前进行规划的agv，需要考虑哪些车道，并找出所有需要考察的车道的id
  ErrorType GetSurroundingLaneNetForTargetVehicle(const int &ego_lane_id, 
                                  std::vector<int> &surrounding_lane_ids);

  // TODO for MLAD: 需要找出，对当前进行规划的agv，需要考虑哪些障碍物（obstacle_map需要在外部定义好，初始化后再传入指针）
  ErrorType GetObstacleMapForTargetVehicle(const int &ego_lane_id, 
                                           common::GridMapND<ObstacleMapType, 2> *obstacle_map,
                                           std::set<std::array<decimal_t, 2>> &obstacle_grids);

  inline double time_stamp() const { return time_stamp_; }

  // inline int ego_id() const { return ego_id_; }

  // inline common::Vehicle ego_vehicle() const { return ego_vehicle_; }

  // inline common::GridMapND<ObstacleMapType, 2> obstacle_map() const {
  //   return obstacle_map_;
  // }
  // inline common::GridMapND<ObstacleMapType, 2> *obstacle_map_ptr() {
  //   return &obstacle_map_;
  // }
  // inline std::set<std::array<decimal_t, 2>> obstacle_grids() const {
  //   return obstacle_grids_;
  // }
  // inline common::VehicleSet surrounding_vehicles() const {
  //   return surrounding_vehicles_;
  // }
  // inline common::VehicleSet key_vehicles() const { return key_vehicles_; }
  inline common::LaneNet whole_lane_net() const { return whole_lane_net_; }
  // inline common::LaneNet surrounding_lane_net() const {
  //   return surrounding_lane_net_;
  // }
  inline common::SemanticLaneSet semantic_lane_set() const {
    return semantic_lane_set_;
  }
  inline const common::SemanticLaneSet *semantic_lane_set_cptr() const {
    const common::SemanticLaneSet *ptr = &semantic_lane_set_;
    return ptr;
  }
  // inline common::SemanticBehavior ego_behavior() const { return ego_behavior_; }
  // inline common::SemanticVehicleSet semantic_surrounding_vehicles() const {
  //   return semantic_surrounding_vehicles_;
  // }
  // inline common::SemanticVehicleSet semantic_key_vehicles() const {
  //   return semantic_key_vehicles_;
  // }
  // inline AgentConfigInfo agent_config_info() const {
  //   return agent_config_info_;
  // }
  inline common::VehicleSet uncertain_vehicles() const { return uncertain_vehicles_; }
  inline std::unordered_map<std::string, AgentConfigInfo> agent_config_infos() const {
    return agent_config_infos_;
  }
  inline std::vector<int> key_vehicle_ids() const { return key_vehicle_ids_; }

  inline int max_ego_vehicle_num() const { return agent_config_infos_.at(p_config_loader_->ego_type()).max_num; }

  inline std::vector<int> uncertain_vehicle_ids() const {
    return uncertain_vehicle_ids_;
  }

  inline common::ObstacleSet obstacle_set() const { return obstacle_set_; }

  inline std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs()
      const {
    return openloop_pred_trajs_;
  }

  inline std::unordered_map<int, common::Lane> local_lanes() const {
    return local_lanes_;
  }

  inline void set_obstacle_set(const common::ObstacleSet &obstacle_set) { obstacle_set_ = obstacle_set; }

  inline void set_whole_lane_net(const common::LaneNet &in) {
    whole_lane_net_ = in;
  }
  inline void set_semantic_lane_set(const common::SemanticLaneSet &in) {
    semantic_lane_set_ = in;
  }
  // inline void set_ego_behavior(const common::SemanticBehavior &in) {
  //   ego_behavior_ = in;
  // }
  inline void set_uncertain_vehicle_ids(
      const std::vector<int> &uncertain_vehicle_ids) {
    uncertain_vehicle_ids_ = uncertain_vehicle_ids;
  }
  inline void set_uncertain_vehicles(const common::VehicleSet &in) {
    uncertain_vehicles_ = in;
  }

  ErrorType GetSpecEgoSemanticMapManager(const int &ego_vehicle_id, EgoSemanticMapManager *ego_smm);

  // 获取指定ego vehicle的当前的任务，决定是否变道，若不变，返回false；否则返回true，并生成task
  bool GetSpecVehicleCurrentTask(const int &ego_vehicle_id, common::Task *task);
  bool GetSpecVehicleCurrentTaskfromMsg(const int &ego_vehicle_id, common::Task *task);
  //获取指定ego vehicle上一次规划时的planning::EudmManager::ReplanningContext
  ErrorType GetSpecEgoLastReplanningContext(const int &ego_vehicle_id, common::ReplanningContext *context);

  //获取指定ego vehicle上一次规划时的Task
  ErrorType GetSpecEgoLastTask(const int &ego_vehicle_id, common::Task *task);

  //获取指定ego vehicle上一次规划时的planning::EudmManager::Snapshot
  ErrorType GetSpecEgoLastSnapshot(const int &ego_vehicle_id, common::Snapshot *snapshot);

  //获取指定ego vehicle上一次规划时的LaneChangeProposal
  ErrorType GetSpecEgoLastLaneChangeProposal(const int &ego_vehicle_id, common::LaneChangeProposal *lc_proposal);

  //获取指定ego vehicle上一次规划时的PreliminaryActiveRequests
  ErrorType GetSpecEgoLastPreliminaryActiveRequests(const int &ego_vehicle_id, 
                                                    std::vector<common::ActivateLaneChangeRequest> *preliminary_active_requests);

  //获取指定ego vehicle上一次规划时的planning::EudmManager::LaneChangeContext
  ErrorType GetSpecEgoLastLaneChangeContext(const int &ego_vehicle_id, common::LaneChangeContext *lc_context);

  //获取所有behavioral_key_vehicles_的id，保存在一个unordered_set里
  ErrorType GetAllBehavioralKeyVehiclesIds(std::unordered_set<int> *behavioral_key_vehicle_ids);

  // 给出指定的vehicle在指定的时刻，是否正处于上报的规划好的轨迹中running
  ErrorType IsSpecVehicleRunningPlannedTraj(const int &behavioral_key_vehicle_id, const decimal_t &time, bool *is_running);

  // 获取对正在执行上报的轨迹的指定的vehicle，其在指定的时刻的状态，指定时刻需要在上报的轨迹的运行时间段内
  ErrorType GetSpecRunningVehicleStateAtSpecTime(const int &behavioral_key_vehicle_id, const decimal_t &time, common::State *state_output);

  //
  ErrorType UpdateSpecEgoVehicleNextTask(const int &ego_vehicle_id, const common::Task &task);

  ErrorType UpdateSpecEgoVehicleNextSemanticBehavior(const int &ego_vehicle_id, const common::SemanticBehavior &behavior);

  ErrorType UpdateSpecEgoVehicleNextTraj(const int &ego_vehicle_id, const vec_E<common::State> &state_traj, const std::string &behavior_uid);

  ErrorType UpdateTaskSetbyMsg(const common::TaskSet taskset);

  ErrorType UpdateSpecEgoVehiclePlanningConfigs(const int &ego_vehicle_id, const common::ReplanningContext &context, 
      const common::Snapshot &snapshot, const common::LaneChangeProposal &lc_proposal, 
      const common::LaneChangeContext &lc_context, 
      const std::vector<common::ActivateLaneChangeRequest> &preliminary_active_requests);

  
  static bool IsSameOrientation(const decimal_t &rad1, const decimal_t &rad2) {
    // 两个弧度值的差，是否小于pi
    decimal_t diff = rad1 - rad2;
    while (diff >= M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;
    return std::fabs(diff) < (M_PI/2);
  }

  ErrorType GetVehicleOriginDesireVelocity(const std::string &type, decimal_t *desire_velocity);

  static std::string GenerateUID() {
    auto uuid = boost::uuids::random_generator()();
	  return boost::uuids::to_string(uuid);
  }

  // 对指定的agv，生成新的下发消息的uid，并赋值
  ErrorType GetSpecEgoVehicleNewUID(const int &ego_vehicle_id, std::string *uid);

  // 对指定的agv，获取是否继续等待agv上传的轨迹回复，以及是否确认轨迹
  ErrorType GetSpecEgoVehicleReplyComplete(const int &ego_vehicle_id, bool *wait_reply, bool *behavior_confirm);

  bool check_task(){
    return task_updated_;
  }

 private:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<decimal_t, common::PointVecForKdTree>,
      common::PointVecForKdTree, 2>
      KdTreeFor2dPointVec;

  // ErrorType UpdateSemanticLaneSet();

  // ErrorType UpdateLocalLanesAndFastLut();

  // ErrorType UpdateSemanticVehicles();

  ErrorType UpdateKeyVehicles();

  // ErrorType OpenloopTrajectoryPrediction();

  ErrorType GetDistanceOnLaneNet(const int &lane_id_0,
                                 const decimal_t &arc_len_0,
                                 const int &lane_id_1,
                                 const decimal_t &arc_len_1,
                                 decimal_t *dist) const;

  ErrorType SampleLane(const common::Lane &lane, const decimal_t &s0,
                       const decimal_t &s1, const decimal_t &step,
                       vec_E<Vecf<2>> *samples, decimal_t *accum_dist) const;

  // void GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
  //     const decimal_t &node_id, const decimal_t &node_length,
  //     const decimal_t &aggre_length, const std::vector<int> &path_to_node,
  //     std::vector<std::vector<int>> *all_paths);

  // void GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
  //     const decimal_t &node_id, const decimal_t &node_length,
  //     const decimal_t &aggre_length, const std::vector<int> &path_to_node,
  //     std::vector<std::vector<int>> *all_paths);

  ErrorType GetLocalLaneUsingLaneIds(const common::State &state,
                                     const std::vector<int> &lane_ids,
                                     const decimal_t max_reflane_dist,
                                     const decimal_t max_backward_dist,
                                     const bool &is_high_quality,
                                     common::Lane *lane);

  ErrorType GetLaneBySampledPoints(const vec_Vecf<2> &samples,
                                   const bool &is_high_quality,
                                   common::Lane *lane) const;

  //  在初始化时，读取到whole_lane_net_后，需要调用此函数，生成behavioral_whole_lane_net_和all_behavioral_lane_ids_
  void InitBehavioralLaneSet(); 

  // 在这里，根据是否可以变道，将key vehicles分为ego和surrounding                   
  ErrorType UpdateKeyVehicles(const common::VehicleSet &key_vehicles);

  // 在这里，将uncertain_vehicle更新为semantic_uncertain_vehicle，主要是为了变道决策等使用
  ErrorType UpdateUncertainVehicles(const common::VehicleSet &uncertain_vehicles);

  ErrorType UpdateSemanticLaneSet();

  // 在这里，根据最新的车道方向的信息，，必须首先更新whole_lane_net!
  ErrorType UpdateBehaviralLanesDirection();

  // TODO for MLAD: 在这里，对ego_vehicles计算优先级，并排序
  ErrorType GetPriorityRankingForEgoVehicles(std::vector<int> *rank);
  
  // TODO for MLAD: 计算vehicle的优先级分数 
  decimal_t ComputePriorityScore(const int &vehicle_id);

  // 检查是否在缓冲区内部
  bool IsVehicleInBehavioralLaneBufferRegion(const common::Vehicle &vehicle, 
                                             const int &nearest_lane_id);

  // 获取当前点到沿着当前行驶方向的路径终点的距离
  decimal_t GetDistanceToFinalPoint(const common::Vehicle &vehicle, 
                                    const int &nearest_lane_id);

  // TODO for MLAD: 获取正向行驶、反向行驶的vehicle数量，以及指定车道上，vehicle的数量。不考慮當前agv
  ErrorType GetVehicleNumInBehavioralLanes(const int &ego_vehicle_id, int *pos_dir_vehicle_num, int *neg_dir_vehicle_num,
                                           std::unordered_map<int, int> *cur_num_in_spec_lane,
                                           std::unordered_map<int, int> *target_num_in_spec_lane);

  ErrorType GetSurroundingLaneNet(const common::LaneNet &lane_net, 
                                  const common::Vehicle &ego_vehicle, 
                                  const decimal_t &surrounding_search_radius, 
                                  common::LaneNet *surrounding_lane_net);

  ErrorType GetObstacleMap(const common::GridMapMetaInfo &obstacle_map_info, 
                           const common::Vehicle &ego_vehicle, 
                           GridMap2D *obstacle_map);

  ErrorType FakeMapper(const common::GridMapMetaInfo &obstacle_map_info, 
                       const common::Vehicle &ego_vehicle, 
                       GridMap2D *obstacle_map, 
                       std::set<std::array<decimal_t, 2>> *obstacle_grids);

  ErrorType RayCastingOnObstacleMap(const common::Vehicle &ego_vehicle, 
                                    GridMap2D *obstacle_map,
                                    std::set<std::array<decimal_t, 2>> *obstacle_grids);
  // 展示behavioral lanes
  void PublishLanes();

  ErrorType GetLeadingUncertainVehicleOnLane(const int &lane_id, const common::State &ref_state, const common::VehicleSet &vehicle_set,
                                             const decimal_t &lat_range, const decimal_t &max_forward_search_dist,
                                             int *leading_uncertain_vehicle_id,
                                             decimal_t *distance_residual_ratio);

  // 需要进行动态障碍物避障检查，判断是否有相遇冲突需要变道决策
  ErrorType UncertainVehiclesAvoidanceCheck();

  // 如果前方一定距离内是断头路，且当前车道是multilane-route，则需要变道
  ErrorType LaneDropAvoidanceCheck();

  // 检查是否有速度为0的CAV。速度为0，短时期规划未启动车，说明其还在上一次的长时期规划中
  ErrorType StopVehicleCheck();

  double time_stamp_{0.0};

  decimal_t pred_time_ = 5.0;
  decimal_t pred_step_ = 0.2;

  decimal_t nearest_lane_range_ = 1.5;
  decimal_t lane_range_ = 10.0;

  decimal_t max_distance_to_lane_ = 2.0;
  // TODO for MLAD: 後續設置爲讀取配置文件
  decimal_t lane_direction_change_threshold_ = 0.5; // 车道方向更新阈值
  decimal_t least_change_lane_distance_ = 0.0; // 当距离缓冲区还有6米时，就不能变道了
  decimal_t impulse_to_center_lane_ = 0.0; // 向中心车道靠拢的冲动
  decimal_t vehicle_change_lane_threshold_ = 0.1; // vehicle变道阈值

  decimal_t search_lat_radius_ = 2.5;

  bool has_fast_lut_ = false;
  std::unordered_map<int, common::Lane> local_lanes_;
  std::unordered_map<int, std::vector<int>> local_to_segment_lut_;
  std::unordered_map<int, std::set<int>> segment_to_local_lut_;

  decimal_t local_lane_length_forward_ = 250.0;
  decimal_t local_lane_length_backward_ = 150.0;

  std::string agent_config_path_; 
  std::unordered_map<std::string, AgentConfigInfo> agent_config_infos_;
  bool use_right_hand_axis_ = true;

  // used to get surrounding LaneNet
  common::PointVecForKdTree lane_net_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_lane_net_;

  // TODO for MLAD: 将BehavioralVehicle设置为主要需要的Vehicle对象，使用一个BehavioralVehicleSet存储全部的合作agv，并分别用一些vector储存ego、key、surrounding的id
  common::BehavioralVehicleSet behavioral_key_vehicles_; // NOTE: 全部的合作agv
  common::TaskSet lane_change_tasks_; // NOTE: 全部的合作agv的任务
  bool task_updated_ = false;                                              
  std::unordered_set<int> ego_vehicle_ids_; // NOTE: 参与变道的agv
  std::unordered_set<int> surrounding_vehicle_ids_; // 不参与变道的agv
  std::vector<int> key_vehicle_ids_; // 所有的agv
  std::vector<int> uncertain_vehicle_ids_; // 非合作agv的id
  common::VehicleSet uncertain_vehicles_; // 在得到ego_semantic_map_manager时，需要将uncertain_vehicles_和behavioral_key_vehicles_合并，作为semantic_key_vehicles_等
  common::SemanticVehicleSet semantic_uncertain_vehicles_; 
  // TODO for MLAD: 需要为vehicle的更新和读取使用加读写锁，留两个函数给外部，用来加锁和解锁

  std::string lane_config_path_;
  common::LaneNet whole_lane_net_;
  common::SemanticLaneSet semantic_lane_set_;
  common::BehavioralLaneSet behavioral_whole_lane_net_; // TODO for MLAD: 将代码中所有whole_lane_net_和semantic_lane_set相关的，全部改为使用behavioral_whole_lane_net_，为了方便后期加读写锁
  // std::unordered_map<int, std::vector<int>> groups_behavioral_lane_ids_; // 各组多径通道的车道id （！不需要分组了，因为优先级判定部分，决定了只能有一组）
  std::vector<int> all_behavioral_lane_ids_;

  bool need_plan_now_ = false;
  // * open loop prediction only for collision checking for onlane mp
  std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs_; // TODO for MLAD: 可以删除

  TicToc global_timer_;
  ConfigLoader *p_config_loader_;

  common::ObstacleSet obstacle_set_; // NOTE: 所有的障碍物信息

  // queue for communication between SematicMapManager and BehaviorPlanner
  common::PriorityIdQueue *id_queue;

  // 记录当前每辆CAV的优先级名次以及分数，分数越小，优先级越高
  std::unordered_map<int, std::pair<int, decimal_t>> priority_scores_;

  // * For highway-like lane structure only
  bool is_simple_lane_structure_ = false;

  static std::mutex vehicle_update_mtx; // 用于相关信息更新、重置ego semantic map manager的互斥锁

  roguelike_ray_casting::roguelike_ray_casting rogue;

  decimal_t max_wait_update_time_ = 3.0; // second
  
  decimal_t uncertain_avoidance_replan_interval_ = 1.0; // second，非合作車輛避障下，對同一個障礙物的重新規劃的時間間隔

  decimal_t drop_lane_avoidance_distance_ = 500.0; // drop lane变道，需要距离终点这么远就需要开始了
  // TODO for MLAD：后续需要在变道规划处，加上这个忽略type，避免去考虑动态障碍物的未来行为
  std::string ignore_vehicle_type_ = "dyn_obstacle";

  BehavioralLaneVisualizer *p_behavioral_lane_visualizer;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_