#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_

#include <random>
#include <assert.h>
#include <iostream>
#include <set>
#include <vector>
#include <unordered_set>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "nanoflann/include/nanoflann.hpp"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {
class DataRenderer {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  DataRenderer(SemanticMapManager *smm_ptr);
  ~DataRenderer() {}

  // inline void set_ego_id(const int id) { ego_id_ = id; } // ! Deprecate
  inline void set_key_vehicle_ids(const std::vector<int> &ids) { 
    key_vehicle_ids_.clear();
    for (const int &id:ids) {
      key_vehicle_ids_.insert(id);
    }
   }
  // inline void set_obstacle_map_info(const common::GridMapMetaInfo &info) {
  //   obstacle_map_info_ = info;
  // }

  // ErrorType Render(const double &time_stamp, const common::LaneNet &lane_net,
  //                  const common::VehicleSet &vehicle_set,
  //                  const common::ObstacleSet &obstacle_set); // ! Deprecate

  ErrorType Render(const double &time_stamp, const common::LaneNet &lane_net,
                   const common::VehicleSet &vehicle_set,
                   const common::ObstacleSet &obstacle_set);

  // 对agv上传的实际正在执行的轨迹，进行更新
  ErrorType UpdateExecutingTrajectory(const double &time_stamp, const int &id, 
                                      const std::string &behavior_uid, const vec_E<common::State> &traj);
  ErrorType UpdateCurrentTask(const common::TaskSet taskset); 

 private:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<decimal_t, common::PointVecForKdTree>,
      common::PointVecForKdTree, 2>
      KdTreeFor2dPointVec;

  // ErrorType GetEgoVehicle(const common::VehicleSet &vehicle_set); // ! Deprecate
  ErrorType UpdateVehicles(const common::VehicleSet &vehicle_set);
  // ErrorType GetObstacleMap(const common::ObstacleSet &obstacle_set);
  ErrorType GetWholeLaneNet(const common::LaneNet &lane_net);
  // ErrorType GetSurroundingLaneNet(const common::LaneNet &lane_net);
  // ErrorType GetSurroundingVehicles(const common::VehicleSet &vehicle_set);
  ErrorType GetUncertainVehicles(const common::VehicleSet &vehicle_set);
  // ErrorType GetSurroundingObjects();

  // ErrorType RayCastingOnObstacleMap();
  // ErrorType FakeMapper();
  // ErrorType InjectObservationNoise();

  // bool if_kdtree_lane_net_updated_ = false;
  // common::PointVecForKdTree lane_net_pts_;
  // std::shared_ptr<KdTreeFor2dPointVec> kdtree_lane_net_;

  // bool if_kdtree_obstacle_set_updated_ = false;
  // common::PointVecForKdTree obstacle_set_pts_;
  // std::shared_ptr<KdTreeFor2dPointVec> kdtree_obstacle_set_;

  // common::PointVecForKdTree vehicle_set_pts_;
  // std::shared_ptr<KdTreeFor2dPointVec> kdtree_vehicle_;

  // int ego_id_ = 0; // ! Deprecate

  int ray_casting_num_ = 1440;

  std::set<std::array<decimal_t, 2>> obs_grids_;
  std::set<std::array<decimal_t, 2>> free_grids_;

  double time_stamp_;

  // common::Vehicle ego_vehicle_; // ! Deprecate
  common::VehicleSet key_vehicles_;
  // common::VehicleParam ego_param_;  // ! Deprecate
  // common::State ego_state_;  // ! Deprecate
  // NOTE: render部分只区分key_vehicles和障碍物，key_vehicles中可以变道的ego和surrounding，在semantic map里区分
  // common::VehicleSet surrounding_vehicles_; // ! Deprecate 

  // common::GridMapMetaInfo obstacle_map_info_; // ! Deprecate
  // GridMap2D *p_obstacle_grid_; // ! Deprecate

  // decimal_t surrounding_search_radius_; // ! Deprecate

  // common::LaneNet surrounding_lane_net_; // ! Deprecate
  common::LaneNet whole_lane_net_;

  SemanticMapManager *p_semantic_map_manager_;

  std::vector<int> new_vehicle_ids_enter_now_; // NOTE: 只包含key vehicles的id，若在set中新出现，但未在这里出现，则认为是uncertain的
  std::vector<int> old_vehicle_ids_out_now_; 

  std::unordered_set<int> key_vehicle_ids_; // 合作agv的id
  std::unordered_set<int> uncertain_vehicle_ids_; // 非合作agv的id
  common::VehicleSet uncertain_vehicles_;

  std::unordered_map<int, vec_E<common::State>> trajs_; // TODO for MLAD: 需要在这里面对轨迹进行过滤，即只将新出现的state更新给smm
  std::unordered_map<int, double> trajs_last_update_time_; // NOTE for MLAD: 此id的traj上一次更新的时间，如果当前的time stamp小于等于上一次更新时间，则跳过不更新

  std::mt19937 random_engine_;
  int cnt_random_ = 0;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
