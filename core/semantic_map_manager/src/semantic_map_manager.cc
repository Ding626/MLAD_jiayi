#include "semantic_map_manager/semantic_map_manager.h"
// #include "roguelike_ray_casting/roguelike_ray_casting.h"
#include <glog/logging.h>

namespace semantic_map_manager {

std::mutex SemanticMapManager::vehicle_update_mtx;

SemanticMapManager::SemanticMapManager(ros::NodeHandle nh, const std::string &agent_config_path, const std::string &lane_config_path)
    : agent_config_path_(agent_config_path), lane_config_path_(lane_config_path) {
  // NOTE: In MLAD, this func is the only way to initialize the object
  p_config_loader_ = new ConfigLoader();
  p_config_loader_->set_agent_config_path(agent_config_path_);
  p_config_loader_->set_lane_config_path(lane_config_path_);
  p_config_loader_->ParseAgentConfig(&agent_config_infos_);
  // TODO for MLAD: Load Lane net
  p_config_loader_->ParseLaneConfig(&whole_lane_net_);
  UpdateSemanticLaneSet();
  InitBehavioralLaneSet();

  id_queue = common::PriorityIdQueue::getInstance();

  p_behavioral_lane_visualizer = new BehavioralLaneVisualizer(nh);

  global_timer_.tic();
}

SemanticMapManager::SemanticMapManager(ros::NodeHandle nh, const std::string &agent_config_path, 
                                       const std::string &lane_config_path,
                                       const decimal_t &uncertain_avoid_replan_interval,
                                       const decimal_t &change_lane_threshold,
                                       const decimal_t &max_wait_update_time)
    : agent_config_path_(agent_config_path), lane_config_path_(lane_config_path), 
      uncertain_avoidance_replan_interval_(uncertain_avoid_replan_interval),
      vehicle_change_lane_threshold_(change_lane_threshold),
      max_wait_update_time_(max_wait_update_time) {
  // NOTE: In MLAD, this func is the only way to initialize the object
  p_config_loader_ = new ConfigLoader();
  p_config_loader_->set_agent_config_path(agent_config_path_);
  p_config_loader_->set_lane_config_path(lane_config_path_);
  p_config_loader_->ParseAgentConfig(&agent_config_infos_);
  // TODO for MLAD: Load Lane net
  p_config_loader_->ParseLaneConfig(&whole_lane_net_);
  UpdateSemanticLaneSet();
  InitBehavioralLaneSet();

  id_queue = common::PriorityIdQueue::getInstance();

  p_behavioral_lane_visualizer = new BehavioralLaneVisualizer(nh);

  global_timer_.tic();
}

ErrorType SemanticMapManager::UpdateSemanticMap(
    const double &time_stamp, const common::VehicleSet &key_vehicles,
    const common::LaneNet &whole_lane_net,
    const common::ObstacleSet &obstacle_set,
    const common::VehicleSet &uncertain_vehicles,
    const bool &key_vehicle_change) {
  
  TicToc timer;
  time_stamp_ = time_stamp;
  
  {
    std::lock_guard<std::mutex> lock_guard(vehicle_update_mtx); 
    need_plan_now_ = false;
    set_whole_lane_net(whole_lane_net); // NOTE: 由于更新BehavioralVehicles可能需要最新的道路信息，故先更新它
    UpdateKeyVehicles(key_vehicles); //NOTE for MLAD: 这里，需要设置，处理新加入的agv的函数
    UpdateUncertainVehicles(uncertain_vehicles);
    set_obstacle_set(obstacle_set);

    // UncertainVehiclesAvoidanceCheck();
    // LaneDropAvoidanceCheck();
    // StopVehicleCheck();
  
    // TODO for MALD: 需要每次判斷，當前形式下，需不需要變道決策，即每個agv與周圍距離遠近
    // if (need_plan_now_) {
    //   // 当前需要开始规划，首先计算各个车道的方向，其次计算各个agv优先级，再将优先级加入到id_queue
    //   // UpdateBehaviralLanesDirection();
    //   std::vector<int> rank;
    //   //LOG(ERROR) << "[UpdateSemanticMap] Start to get priority.";
    //   GetPriorityRankingForEgoVehicles(&rank);
    //   //LOG(ERROR) << "[UpdateSemanticMap] rank size is "<< rank.size();
    //   id_queue->UpdateQueue(rank);
    //   //LOG(ERROR) << "[UpdateSemanticMap] finish updating queue.\n";
    // }

    PublishLanes();
  }
  return kSuccess;
}

ErrorType SemanticMapManager::SaveMapToLog() {
  if (agent_config_infos_.empty()) {
    return kWrongStatus;
  }
  std::ostringstream line_info;

  // common::VehicleSet vehicle_set = surrounding_vehicles_;
  // vehicle_set.vehicles.insert(std::make_pair(ego_vehicle_.id(), ego_vehicle_));
  line_info << "key vehicles <id,time,x,y,v,acc,theta,curvature,steer>:\n";
  for (auto &v : behavioral_key_vehicles_.behavioral_vehicles) {
    line_info << v.first << "," << v.second.vehicle().state().time_stamp << ","
              << v.second.vehicle().state().vec_position[0] << ","
              << v.second.vehicle().state().vec_position[1] << ","
              << v.second.vehicle().state().velocity << ","
              << v.second.vehicle().state().acceleration << ","
              << v.second.vehicle().state().angle << ","
              << v.second.vehicle().state().curvature << ","
              << v.second.vehicle().state().steer << "\n";
  }
  line_info << "uncertain vehicles <id,time,x,y,v,acc,theta,curvature,steer>:\n";
  for (auto &v : uncertain_vehicles_.vehicles) {
    line_info << v.first << "," << v.second.state().time_stamp << ","
              << v.second.state().vec_position[0] << ","
              << v.second.state().vec_position[1] << ","
              << v.second.state().velocity << ","
              << v.second.state().acceleration << ","
              << v.second.state().angle << ","
              << v.second.state().curvature << ","
              << v.second.state().steer << "\n";
  }
  LOG(INFO) << line_info.str();
  return kSuccess;
}

ErrorType SemanticMapManager::NaiveRuleBasedLateralBehaviorPrediction( // Tips: 在这里对其它agv进行意图判断，没有用lstm
    const common::Vehicle &vehicle, const int nearest_lane_id,
    common::ProbDistOfLatBehaviors *lat_probs) {
  if (nearest_lane_id == kInvalidLaneId) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  SemanticLane nearest_lane =
      semantic_lane_set_.semantic_lanes.at(nearest_lane_id);
  common::StateTransformer stf(nearest_lane.lane);

  common::FrenetState fs;
  if (stf.GetFrenetStateFromState(vehicle.state(), &fs) != kSuccess) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  decimal_t prob_lcl = 0.0;
  decimal_t prob_lcr = 0.0;
  decimal_t prob_lk = 0.0;

  const decimal_t lat_distance_threshold = 0.4;
  const decimal_t lat_vel_threshold = 0.35;
  if (use_right_hand_axis_) {
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.l_lane_id != kInvalidLaneId &&
        nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.r_lane_id != kInvalidLaneId &&
               nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;
    }
  } else {
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.r_lane_id != kInvalidLaneId &&
        nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.l_lane_id != kInvalidLaneId &&
               nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;
    }
  }

  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeLeft, prob_lcl);
  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeRight, prob_lcr);
  lat_probs->SetEntry(common::LateralBehavior::kLaneKeeping, prob_lk);
  lat_probs->is_valid = true;

  return kSuccess;
}

ErrorType SemanticMapManager::MobilRuleBasedBehaviorPrediction(
    const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
    common::ProbDistOfLatBehaviors *res) {
  decimal_t lane_radius;
  if (agent_config_infos_.find(vehicle.type()) == agent_config_infos_.end()) {
    lane_radius = 150.0;
  }
  else {
    lane_radius = agent_config_infos_.at(vehicle.type()).surrounding_search_radius;
  }

  vec_E<common::Lane> lanes;
  vec_E<common::Vehicle> leading_vehicles;
  vec_E<common::Vehicle> following_vehicles;
  vec_E<common::FrenetState> leading_frenet_states;
  vec_E<common::FrenetState> follow_frenet_states;

  std::vector<LateralBehavior> behaviors{LateralBehavior::kLaneKeeping,
                                         LateralBehavior::kLaneChangeLeft,
                                         LateralBehavior::kLaneChangeRight};

  for (const auto &behavior : behaviors) {
    // ~ Prepare lane
    common::Lane ref_lane;
    GetRefLaneForStateByBehavior(vehicle.state(), vehicle.type(), std::vector<int>(), behavior,
                                 lane_radius, lane_radius, false, &ref_lane);

    // ~ Prepare leading and following vehicle
    bool has_leading_vehicle = false, has_following_vehicle = false;
    common::Vehicle leading_vehicle, following_vehicle;
    common::FrenetState leading_frenet_state, following_frenet_state;
    GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ref_lane, vehicle.state(), nearby_vehicles, &has_leading_vehicle,
        &leading_vehicle, &leading_frenet_state, &has_following_vehicle,
        &following_vehicle, &following_frenet_state);

    // ~ essemble
    lanes.push_back(ref_lane);
    leading_vehicles.push_back(leading_vehicle);
    following_vehicles.push_back(following_vehicle);
    leading_frenet_states.push_back(leading_frenet_state);
    follow_frenet_states.push_back(following_frenet_state);
  }

  if (common::MobilBehaviorPrediction::LateralBehaviorPrediction(
          vehicle, lanes, leading_vehicles, leading_frenet_states,
          following_vehicles, follow_frenet_states, nearby_vehicles,
          res) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetLeadingAndFollowingVehiclesFrenetStateOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
    common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
    bool *has_following_vehicle, common::Vehicle *following_vehicle,
    common::FrenetState *following_fs) const {
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = search_lat_radius_;
  GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                          leading_vehicle, &distance_residual_ratio);
  GetFollowingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                            following_vehicle);

  common::StateTransformer stf(ref_lane);
  *has_leading_vehicle = false;
  *has_following_vehicle = false;

  if (leading_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(leading_vehicle->state(), leading_fs) ==
        kSuccess) {
      *has_leading_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  if (following_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(following_vehicle->state(), following_fs) ==
        kSuccess) {
      *has_following_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetVehicleNearestLaneId(const common::State &state, int *ego_lane_id) const {
  int nearest_lane_id;
  decimal_t distance, arclen;
  if (GetNearestLaneIdUsingState(state.ToXYTheta(),
                                 std::vector<int>(), &nearest_lane_id,
                                 &distance, &arclen) != kSuccess) {
    return kWrongStatus;
  }
  *ego_lane_id = nearest_lane_id;
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateSemanticLaneSet() {
  semantic_lane_set_.clear();
  // Update semantic lane set
  {
    for (const auto &pe : whole_lane_net_.lane_set) {
      common::SemanticLane semantic_lane;
      semantic_lane.id = pe.second.id;
      semantic_lane.dir = pe.second.dir;
      semantic_lane.child_id = pe.second.child_id;
      semantic_lane.father_id = pe.second.father_id;
      semantic_lane.l_lane_id = pe.second.l_lane_id;
      semantic_lane.l_change_avbl = pe.second.l_change_avbl;
      semantic_lane.r_lane_id = pe.second.r_lane_id;
      semantic_lane.r_change_avbl = pe.second.r_change_avbl;
      semantic_lane.behavior = pe.second.behavior;
      semantic_lane.length = pe.second.length;

      vec_Vecf<2> samples;
      for (const auto &pt : pe.second.lane_points) {
        samples.push_back(pt);
      }
      if (common::LaneGenerator::GetLaneBySamplePoints(
              samples, &semantic_lane.lane) != kSuccess) {
        continue;
      }

      semantic_lane_set_.semantic_lanes.insert(
          std::pair<int, common::SemanticLane>(semantic_lane.id,
                                               semantic_lane));
    }
  }
  // Check the consistency of semantic map
  {
    for (auto &semantic_lane : semantic_lane_set_.semantic_lanes) {
      if (semantic_lane.second.l_change_avbl) {
        auto l_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.l_lane_id);
        if (l_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.l_change_avbl = false;
          semantic_lane.second.l_lane_id = kInvalidLaneId;
        }
      }
      if (semantic_lane.second.r_change_avbl) {
        auto r_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.r_lane_id);
        if (r_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.r_change_avbl = false;
          semantic_lane.second.r_lane_id = kInvalidLaneId;
        }
      }
      for (auto it = semantic_lane.second.father_id.begin();
           it < semantic_lane.second.father_id.end();) {
        auto father_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (father_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.father_id.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = semantic_lane.second.child_id.begin();
           it < semantic_lane.second.child_id.end();) {
        auto child_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (child_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.child_id.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetDistanceToLanesUsing3DofState(
    const Vec3f &state,
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const {
  for (const auto &p : semantic_lane_set_.semantic_lanes) {
    decimal_t arc_len;
    p.second.lane.GetArcLengthByVecPosition(Vec2f(state(0), state(1)),
                                            &arc_len);

    // decimal_t arc_len2;
    // p.second.lane.GetArcLengthByVecPositionUsingBinarySearch(
    //     Vec2f(state(0), state(1)), &arc_len2);

    // if (std::fabs(arc_len - arc_len2) > 1.0) {
    //   printf("[XXX]lane_id: %d, arc_len1: %lf, arc_len2: %lf\n", p.second.id,
    //          arc_len, arc_len2);
    // }

    Vec2f pt;
    p.second.lane.GetPositionByArcLength(arc_len, &pt);
    double dist = std::hypot((state(0) - pt(0)), (state(1) - pt(1)));

    if (dist > lane_range_) continue;

    decimal_t lane_angle;
    p.second.lane.GetOrientationByArcLength(arc_len, &lane_angle);
    decimal_t angle_diff = normalize_angle(lane_angle - state(2));

    res->insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        dist, arc_len, angle_diff, p.second.id));
  }
#if 0
  for (auto iter = res->begin(); iter != res->end(); ++iter) {
    std::cout << "[Dist] distance = " << std::get<0>(*iter)
              << " , angle_diff = " << std::get<1>(*iter)
              << " , id = " << std::get<2>(*iter) << std::endl;
  }
  std::cout << "\n";
#endif
  return kSuccess;
}

ErrorType SemanticMapManager::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[IsTopologicallyReachable]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }
  // ~ check whether any node of the path is reachable from lane id
  const int max_expansion_nodes = 20;

  // ~ BFS starting from lane_id;
  std::unordered_map<int, int> num_lc_map;
  std::set<int> visited_set;
  std::list<int> queue;
  visited_set.insert(lane_id);
  queue.push_back(lane_id);
  num_lc_map.insert(std::make_pair(lane_id, 0));

  int expanded_nodes = 1;
  int cur_id = lane_id;
  bool is_reachable = false;
  while (!queue.empty() && expanded_nodes < max_expansion_nodes) {
    cur_id = queue.front();
    queue.pop_front();
    expanded_nodes++;

    if (std::find(path.begin(), path.end(), cur_id) != path.end()) {
      *num_lane_changes = num_lc_map.at(cur_id);
      is_reachable = true;
      break;
    }
    std::vector<int> child_ids;
    auto it = semantic_lane_set_.semantic_lanes.find(cur_id);
    if (it == semantic_lane_set_.semantic_lanes.end()) {
      continue;
    } else {
      child_ids = it->second.child_id;
      if (it->second.l_change_avbl) child_ids.push_back(it->second.l_lane_id);
      if (it->second.r_change_avbl) child_ids.push_back(it->second.r_lane_id);
    }
    if (child_ids.empty()) continue;
    for (auto &id : child_ids) {
      if (visited_set.count(id) == 0) {
        visited_set.insert(id);
        queue.push_back(id);
        if (it->second.l_change_avbl && id == it->second.l_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else if (it->second.r_change_avbl && id == it->second.r_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id)));
        }
      }
    }
  }

  if (!is_reachable) {
    *res = false;
  } else {
    *res = true;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) const {
  // tuple: dist, arc_len, angle_diff, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> lanes_in_dist;
  if (GetDistanceToLanesUsing3DofState(state, &lanes_in_dist) != kSuccess) {
    return kWrongStatus;
  }

  if (lanes_in_dist.empty()) {
    printf("[GetNearestLaneIdUsingState]No nearest lane found.\n");
    return kWrongStatus;
  }

  //   if (navi_path.empty()) {
  //     *id = std::get<3>(*lanes_in_dist.begin());
  //     *distance = std::get<0>(*lanes_in_dist.begin());
  //     *arc_len = std::get<1>(*lanes_in_dist.begin());
  //   } else {
  // #if 0
  //     // bool has_nearest_lane_found = false;
  //     std::map<decimal_t, std::pair<int, decimal_t>> surrounding_lanes;
  //     for (auto &t : lanes_in_dist) {
  //       int lane_id = std::get<3>(t);
  //       decimal_t cur_distance = std::get<0>(t);
  //       bool is_reachable = false;
  //       int num_lane_changes;
  //       if (IsTopologicallyReachable(lane_id, navi_path, &num_lane_changes,
  //                                    &is_reachable) != kSuccess) {
  //         continue;
  //       }
  //       if (is_reachable) {
  //         const decimal_t lane_change_dist_cost = 0.1;
  //         decimal_t cost =
  //             num_lane_changes * lane_change_dist_cost + cur_distance;
  //         surrounding_lanes.emplace(cost, std::make_pair(lane_id,
  //         cur_distance));
  //       }
  //     }
  //     if (surrounding_lanes.empty()) {
  //       return kWrongStatus;
  //     }
  //     *id = surrounding_lanes.begin()->second.first;
  //     *distance = surrounding_lanes.begin()->second.second;
  // #else
  //     *id = std::get<3>(*lanes_in_dist.begin());
  //     *distance = std::get<0>(*lanes_in_dist.begin());
  //     *arc_len = std::get<1>(*lanes_in_dist.begin());
  // #endif
  //   }

  // * Get candidate lanes within a small range, then sort by angle_diff
  // tuple: angle_diff, dist, arc_len, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>>
      lanes_in_angle_diff;
  for (const auto &ele : lanes_in_dist) {
    if (std::get<0>(ele) > nearest_lane_range_) {
      break;
    }
    lanes_in_angle_diff.insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        fabs(std::get<2>(ele)), std::get<0>(ele), std::get<1>(ele),
        std::get<3>(ele)));
  }
  if (lanes_in_angle_diff.empty() ||
      std::get<0>(*lanes_in_angle_diff.begin()) > kPi / 2) {
    // Use the nearest lane with suitable angle diff
    for (const auto &ele : lanes_in_dist) {
      if (std::get<2>(ele) < kPi / 2) {
        *id = std::get<3>(ele);
        *distance = std::get<0>(ele);
        *arc_len = std::get<1>(ele);
        return kSuccess;
      }
    }
    // Otherwise, use the nearest lane
    *id = std::get<3>(*lanes_in_dist.begin());
    *distance = std::get<0>(*lanes_in_dist.begin());
    *arc_len = std::get<1>(*lanes_in_dist.begin());
    // printf(
    //     "[GetNearestLaneIdUsingState]No suitable lane in %f m, use the
    //     nearest " "one, dist: %lf, id: %d\n", nearest_lane_range_, *distance,
    //     *id);
    return kSuccess;
  }
  // * Use the lane with minimum angle diff
  *id = std::get<3>(*lanes_in_angle_diff.begin());
  *distance = std::get<1>(*lanes_in_angle_diff.begin());
  *arc_len = std::get<2>(*lanes_in_angle_diff.begin());

  if (std::get<3>(*lanes_in_angle_diff.begin()) !=
      std::get<3>(*lanes_in_dist.begin())) {
    // printf(
    //     "[GetNearestLaneIdUsingState]Use minimum angle diff lane, "
    //     "angle_diff: %lf, dist: %lf, id: %d\n",
    //     std::get<0>(*lanes_in_angle_diff.begin()), *distance, *id);
  }

  // *id = std::get<3>(*lanes_in_dist.begin());
  // *distance = std::get<0>(*lanes_in_dist.begin());
  // *arc_len = std::get<1>(*lanes_in_dist.begin());
  // printf("[GetNearestLaneIdUsingState]angle_diff: %lf, dist: %lf, id: %d\n",
  //        std::get<2>(*lanes_in_dist.begin()), *distance, *id);
  return kSuccess;
}

// ErrorType SemanticMapManager::TrajectoryPredictionForVehicle(
//     const common::Vehicle &vehicle, const common::Lane &lane,
//     const decimal_t &t_pred, const decimal_t &t_step,
//     vec_E<common::State> *traj) {
//   planning::OnLaneFsPredictor::GetPredictedTrajectory(lane, vehicle, t_pred,
//                                                       t_step, traj);
//   return kSuccess;
// }

ErrorType SemanticMapManager::UpdateKeyVehicles(const common::VehicleSet &key_vehicles) {
  // NOTE: 在这里调用behavioralVehicles的类，更新一下
  // TODO for MLAD: key_vehicle_ids_需要加互斥鎖！！！
  // 首先历遍，删除不在的agv
  // std::cout << "Length of key_vehicle_ids_: " << key_vehicle_ids_.size() << std::endl;
  for (const auto &id:key_vehicle_ids_) {
    if (key_vehicles.vehicles.find(id) == key_vehicles.vehicles.end()) {
      //新的key_vehicles不存在旧的vehicles，说明需要删除
      behavioral_key_vehicles_.behavioral_vehicles.erase(id);

      if (ego_vehicle_ids_.find(id) != ego_vehicle_ids_.end()) {
        ego_vehicle_ids_.erase(id);
        need_plan_now_ = true; // 有ego vehicle不存在了
      }

      if (surrounding_vehicle_ids_.find(id) != surrounding_vehicle_ids_.end()) {
        surrounding_vehicle_ids_.erase(id);
      }
    }
  }
  // 对已经存在的agv，更新；不存在的agv，将其加入到behavioral_key_vehicles_中。同时根据vehicles中的信息，分出ego和surround
  //ego_vehicle_ids_.clear();
  //ego_vehicle_ids_.shrink_to_fit();
  //surrounding_vehicle_ids_.clear();
  //surrounding_vehicle_ids_.shrink_to_fit();
  std::vector<int> tmp_key_vehicle_ids; // 所有的agv

  for (const auto &v:key_vehicles.vehicles) {
    tmp_key_vehicle_ids.emplace_back(v.first);
    common::Vehicle vehicle = v.second;

    // 对每一个vehicle，更新其实际车道id
    int nearest_lane_id;
    decimal_t dist_to_lane;
    decimal_t arc_len_onlane;
    if (GetNearestLaneIdUsingState(vehicle.state().ToXYTheta(),
                                  std::vector<int>(), &nearest_lane_id,
                                  &dist_to_lane, &arc_len_onlane) != kSuccess) {
      return kWrongStatus;
    }
    // 查看是否在缓冲区内部
    if (behavioral_whole_lane_net_.behavioral_lanes.find(nearest_lane_id) != behavioral_whole_lane_net_.behavioral_lanes.end()
        && !IsVehicleInBehavioralLaneBufferRegion(vehicle, nearest_lane_id)) {
      // ego_vehicle_ids_.insert(v.first); // 這樣做，多進程不安全
      if (ego_vehicle_ids_.find(v.first) == ego_vehicle_ids_.end()) {
        need_plan_now_ = true; // 有新的agv變爲ego vehicles
        ego_vehicle_ids_.insert(v.first); // 由於不會出現多個線程同時寫入insert的情況，只會有一個寫入一個讀取，所以是安全的
        //LOG(ERROR) << "[SemanticMapManager::UpdateKeyVehicles] agent " << v.first 
           //       << " first enter lane " << nearest_lane_id;
      }

      if (surrounding_vehicle_ids_.find(v.first) != surrounding_vehicle_ids_.end()) {
        surrounding_vehicle_ids_.erase(v.first);
      }

    }
    else {
      if (ego_vehicle_ids_.find(v.first) != ego_vehicle_ids_.end()) {
        // need_plan_now_ = true; // 有agv需要從ego vehicles中去除
        ego_vehicle_ids_.erase(v.first);
        behavioral_key_vehicles_.behavioral_vehicles.at(v.first).set_target_lane_id(nearest_lane_id);
      }

      if (surrounding_vehicle_ids_.find(v.first) == surrounding_vehicle_ids_.end()) {
        surrounding_vehicle_ids_.insert(v.first);
      }
    }

    auto iter = behavioral_key_vehicles_.behavioral_vehicles.find(v.first);
    if (behavioral_key_vehicles_.behavioral_vehicles.end() != iter) {
      behavioral_key_vehicles_.behavioral_vehicles.at(v.first).UpdateVehicle(vehicle);
    }
    else {
      // 新建一个behavioral_vehicle
      common::BehavioralVehicle behavior_vehicle;
      behavior_vehicle.InitializeVehicle(vehicle);
      behavioral_key_vehicles_.behavioral_vehicles.insert(std::make_pair(v.first, behavior_vehicle));
    }
    behavioral_key_vehicles_.behavioral_vehicles.at(v.first).set_nearest_lane_id(nearest_lane_id);
    behavioral_key_vehicles_.behavioral_vehicles.at(v.first).set_dist_to_lane(dist_to_lane);
    behavioral_key_vehicles_.behavioral_vehicles.at(v.first).set_arc_len_onlane(arc_len_onlane);

    if (behavioral_whole_lane_net_.behavioral_lanes.find(
        behavioral_key_vehicles_.behavioral_vehicles.at(v.first).target_lane_id()) == 
        behavioral_whole_lane_net_.behavioral_lanes.end()) {
      behavioral_key_vehicles_.behavioral_vehicles.at(v.first).set_target_lane_id(nearest_lane_id);
    }
  }

  std::swap(tmp_key_vehicle_ids, key_vehicle_ids_);
  
  return kSuccess;
}

// 检查是否在缓冲区内部
bool SemanticMapManager::IsVehicleInBehavioralLaneBufferRegion(const common::Vehicle &vehicle, const int &nearest_lane_id) {
  // 需要确保nearest_lane_id确实是BehavioralLane
  decimal_t distance = GetDistanceToFinalPoint(vehicle, nearest_lane_id);

  return distance < behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).buffer();
}

// TODO for MLAD: 获取2 * surrounding_search_radius范围内，正向行驶、反向行驶的vehicle数量，以及指定车道上，vehicle的数量
ErrorType SemanticMapManager::GetVehicleNumInBehavioralLanes(const int &ego_vehicle_id, int *pos_dir_vehicle_num, int *neg_dir_vehicle_num, 
                                                             std::unordered_map<int, int> *cur_num_in_spec_lane, 
                                                             std::unordered_map<int, int> *target_num_in_spec_lane) {
  *pos_dir_vehicle_num = 0;
  *neg_dir_vehicle_num = 0;
  common::State ego_state = behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).vehicle().state();
  decimal_t search_radius = agent_config_infos_.at(behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).vehicle().type()).surrounding_search_radius * 2;
  for (auto iter = behavioral_key_vehicles_.behavioral_vehicles.begin(); iter != behavioral_key_vehicles_.behavioral_vehicles.end(); iter++) {
    // 只考虑当前的最近车道
    if (ego_vehicle_id == iter->first) {
      continue; // NOTE for MLAD: 不將當前agv考慮進去？
    }

    // common::State state = iter->second.vehicle().state();
    // decimal_t dx = state.vec_position(0) - ego_state.vec_position(0);
    // decimal_t dy = state.vec_position(1) - ego_state.vec_position(1);
    // decimal_t dist = std::hypot(dx, dy);
    // if (dist > search_radius) {
    //   // 对于范围之外的车不考虑了
    //   continue;
    // }

    int lane_id = iter->second.nearest_lane_id();
    if (behavioral_whole_lane_net_.behavioral_lanes.find(lane_id) != behavioral_whole_lane_net_.behavioral_lanes.end()) {
      decimal_t vehicle_dir = iter->second.vehicle().state().angle;
      decimal_t lane_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(lane_id).dir();
      if (IsSameOrientation(vehicle_dir, lane_main_dir)) {
        *pos_dir_vehicle_num += 1;
      }
      else {
        *neg_dir_vehicle_num += 1;
      }

      if (cur_num_in_spec_lane->find(lane_id) != cur_num_in_spec_lane->end()) {
        cur_num_in_spec_lane->at(lane_id) += 1;
      }
    }
    int target_lane_id = iter->second.target_lane_id() == kInvalidLaneId ? lane_id : iter->second.target_lane_id();
    if (target_num_in_spec_lane->find(target_lane_id) != target_num_in_spec_lane->end()) {
      target_num_in_spec_lane->at(target_lane_id) += 1;
    }
    //LOG(ERROR) << "[SemanticMapManager::GetVehicleNumInBehavioralLanes] the target lane id of agent " 
           //   << iter->first << " is " << target_lane_id;
  }

  for (auto iter = semantic_uncertain_vehicles_.semantic_vehicles.begin(); iter != semantic_uncertain_vehicles_.semantic_vehicles.end(); iter++) {
    // 只考虑当前的最近车道
    if (iter->second.vehicle.type() == ignore_vehicle_type_) {
      continue;
    }

    // common::State state = iter->second.vehicle.state();
    // decimal_t dx = state.vec_position(0) - ego_state.vec_position(0);
    // decimal_t dy = state.vec_position(1) - ego_state.vec_position(1);
    // decimal_t dist = std::hypot(dx, dy);
    // if (dist > search_radius) {
    //   // 对于范围之外的车不考虑了
    //   continue;
    // }

    int lane_id = iter->second.nearest_lane_id;
    if (behavioral_whole_lane_net_.behavioral_lanes.find(lane_id) != behavioral_whole_lane_net_.behavioral_lanes.end()) {
      decimal_t vehicle_dir = iter->second.vehicle.state().angle;
      decimal_t lane_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(lane_id).dir();
      if (IsSameOrientation(vehicle_dir, lane_main_dir)) {
        *pos_dir_vehicle_num += 1;
      }
      else {
        *neg_dir_vehicle_num += 1;
      }

      if (cur_num_in_spec_lane->find(lane_id) != cur_num_in_spec_lane->end()) {
        cur_num_in_spec_lane->at(lane_id) += 1;
      }
    }

    lane_id = iter->second.target_lane_id == kInvalidLaneId ? lane_id : iter->second.target_lane_id;
    if (target_num_in_spec_lane->find(lane_id) != target_num_in_spec_lane->end()) {
      target_num_in_spec_lane->at(lane_id) += 1;
    }
  }

  return kSuccess;
}

ErrorType SemanticMapManager::UpdateBehaviralLanesDirection() {
  // 在这里，根据最新的车道行驶方向，必须首先更新whole_lane_net、合作agv与非合作agv!
  if (all_behavioral_lane_ids_.size() == 2) {
    // 若只有两个车道，则只能一个正向一个反向，不需要更新车道方向了
    return kSuccess;
  }

  // 先统计两个方向的agv各有多少个，考虑合作agv和非合作agv
  int pos_dir_vehicle_num=0, neg_dir_vehicle_num=0;
  std::unordered_map<int, int> cur_num_in_spec_lane;
  std::unordered_map<int, int> target_num_in_spec_lane;
  GetVehicleNumInBehavioralLanes(kInvalidAgentId, &pos_dir_vehicle_num, &neg_dir_vehicle_num, &cur_num_in_spec_lane, &target_num_in_spec_lane);

  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] pos_dir_vehicle_num is " << pos_dir_vehicle_num;
  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] neg_dir_vehicle_num is " << neg_dir_vehicle_num;

  if (pos_dir_vehicle_num == 0 && neg_dir_vehicle_num == 0) {
    // 沒有車
    return kWrongStatus; 
  }

  int pos_dir_lane_num=0, neg_dir_lane_num=0, cur_split_lane_id=-1;
  bool pre_dir_straight;

  for (int i=0; i<all_behavioral_lane_ids_.size(); i++) {
    if (behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[i]).cur_pass_dir_straight()) pos_dir_lane_num++;
    else neg_dir_lane_num++;

    if (i==0) {
      pre_dir_straight = behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[i]).cur_pass_dir_straight();
    }
    else if (pre_dir_straight != behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[i]).cur_pass_dir_straight()) {
      cur_split_lane_id = i;
      pre_dir_straight = !pre_dir_straight;
      // 由于与主方向相同的车道id都在右边，且其id更小，all_behavioral_lane_ids_中是按照id从小到大排序的，所以cur_split_lane_id为第一个与主方向反向的车道id
    }
  }

  if (pos_dir_lane_num==0 || neg_dir_lane_num==0) {
    // 理论上不可能出现一个方向上没有车道的情况，若有，则报错
    return kWrongStatus;
  }

  if (cur_split_lane_id==-1) {
    // 所有车道的方向都与最右边车道方向相同，出错
    return kWrongStatus;
  }

  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] pos_dir_lane_num is " << pos_dir_lane_num;
  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] neg_dir_lane_num is " << neg_dir_lane_num;

  decimal_t pos_vehicle_num_per_lane = static_cast<decimal_t>(pos_dir_vehicle_num)/static_cast<decimal_t>(pos_dir_lane_num);
  decimal_t neg_vehicle_num_per_lane = static_cast<decimal_t>(neg_dir_vehicle_num)/static_cast<decimal_t>(neg_dir_lane_num);
  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] pos_vehicle_num_per_lane is " << pos_vehicle_num_per_lane;
  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] neg_vehicle_num_per_lane is " << neg_vehicle_num_per_lane;

  if (pos_vehicle_num_per_lane < 1 && neg_vehicle_num_per_lane < 1) {
    //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] lanes are not fully used, no need to increase nor decrease lane number";
    return kSuccess;
  }

  decimal_t rate = std::fmin(pos_vehicle_num_per_lane, neg_vehicle_num_per_lane) / std::fmax(pos_vehicle_num_per_lane, neg_vehicle_num_per_lane);

  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] rate is " << rate;
  //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] cur_split_lane_id is " << cur_split_lane_id;
  if (rate < lane_direction_change_threshold_) {
    // 改变车道
    if (pos_vehicle_num_per_lane > neg_vehicle_num_per_lane 
        && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id]).dir_changeable()) {
      // 增加正向行驶的车道
      //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] add main dir lane";
      behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id]).reverse_cur_pass_dir_straight();
      pos_dir_lane_num++;
      neg_dir_lane_num--;
    }
    else if (pos_vehicle_num_per_lane < neg_vehicle_num_per_lane 
             && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id-1]).dir_changeable()) {
      // 增加与主方向相反行驶的车道数量
      //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] add opposite main dir lane";
      behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id-1]).reverse_cur_pass_dir_straight();
      pos_dir_lane_num--;
      neg_dir_lane_num++;
    }
  }

  // NOTE for MLAD: 需要在这里判断，是否有正方向/反方向车道只有一条，且这条车道是否因前方有动态障碍物、有相遇冲突而触发了变道决策。（如果动态障碍物在同一车道上距离远，是不会触发变道决策的）
  // REMINDER: 目前不支持所有车道上都出现相遇冲突的情况
  if (pos_dir_lane_num == 1 
      && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id-1]).opp_uncertain_vehicle_encounter_conflict()
      && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id]).dir_changeable()) {
    // 当主方向车道数只有一条，且这条车道上有相遇冲突，并且最邻近一条车道的方向是可以切换的，则将最邻近的反向车道变为主方向车道
    //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] add main dir lane because of encounter conflict.";
    behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id]).reverse_cur_pass_dir_straight();
    pos_dir_lane_num++;
    neg_dir_lane_num--;
  }
  else if (neg_dir_lane_num == 1
           && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id]).opp_uncertain_vehicle_encounter_conflict()
           && behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id-1]).dir_changeable()) {
    // 当反方向车道数只有一条，且这条车道上有相遇冲突，并且最邻近一条车道的方向是可以切换的，则将最邻近的主向车道变为反方向车道
    //LOG(ERROR) << "[SemanticMapManager::UpdateBehaviralLanesDirection] add main dir lane because of encounter conflict.";
    behavioral_whole_lane_net_.behavioral_lanes.at(all_behavioral_lane_ids_[cur_split_lane_id-1]).reverse_cur_pass_dir_straight();
    pos_dir_lane_num--;
    neg_dir_lane_num++;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetPriorityRankingForEgoVehicles(std::vector<int> *rank) {
  // TODO for MLAD: 在这里，对ego_vehicles计算优先级，并排序
  std::vector<std::pair<int, decimal_t>> id_with_priority;
  for (const int &vehicle_id:ego_vehicle_ids_) {
    // 查看是否在缓冲区内部，需要在UpdateKeyVehicles中进行，若在缓冲区内部，则不设置为ego_vehicle。
    id_with_priority.emplace_back(std::make_pair(vehicle_id, ComputePriorityScore(vehicle_id)));
  }
  //LOG(ERROR) << "[SemanticMapManager::GetPriorityRankingForEgoVehicles] get priority score.";
  //LOG(ERROR) << "[SemanticMapManager::GetPriorityRankingForEgoVehicles] size of id_with_priority: " << id_with_priority.size();
  // 从小到大排序
  sort(id_with_priority.begin(), id_with_priority.end(), [](const std::pair<int, decimal_t> &a, const std::pair<int, decimal_t> &b) {
        return a.second < b.second;
  });
  //LOG(ERROR) << "[SemanticMapManager::GetPriorityRankingForEgoVehicles] finish sorting.\n";

  std::unordered_map<int, std::pair<int, decimal_t>> tmp_scores;
  int order = 0;
  
  for (const auto &p:id_with_priority) {
    rank->emplace_back(p.first);
    //LOG(WARNING) << "[SemanticMapManager::GetPriorityRankingForEgoVehicles] id: " << p.first;
    if (!behavioral_key_vehicles_.behavioral_vehicles.at(p.first).is_task_init()) {
      // TODO for MLAD: 此 vehicle還沒有執行過任務，需要先下發直行的軌跡，後續可能要改
      rank->emplace_back(p.first);
    }

    tmp_scores[p.first] = std::pair<int, decimal_t>(order, p.second);
    order++;
  }

  std::swap(tmp_scores, priority_scores_);

  return kSuccess;
}
  
// 获取当前点到沿着当前行驶方向的路径终点的距离
decimal_t SemanticMapManager::GetDistanceToFinalPoint(const common::Vehicle &vehicle, const int &nearest_lane_id) {
  decimal_t vehicle_dir = vehicle.state().angle;
  decimal_t lane_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).dir();
  Vec2f final_point;
  if (IsSameOrientation(vehicle_dir, lane_main_dir)) {
    final_point = whole_lane_net_.lane_set.at(nearest_lane_id).final_point;
  }
  else {
    final_point = whole_lane_net_.lane_set.at(nearest_lane_id).start_point;
  }

  Vec2f cur_point = vehicle.state().vec_position;

  // decimal_t dis = std::sqrt((cur_point(0)-final_point(0))*(cur_point(0)-final_point(0)) + (cur_point(1)-final_point(1))*(cur_point(1)-final_point(1)));
  return fabs(cur_point(0)-final_point(0));
}
  
decimal_t SemanticMapManager::ComputePriorityScore(const int &vehicle_id) {
  // 计算vehicle的优先级分数。需要确保传入的vehicle_id一定是合法的
  // TODO for MLAD: 暂时只根据当前vehicle距离其路径终点的距离计算，后续需要更加精确的方法，同时考虑距离障碍物远近等信息

  int nearest_lane_id = behavioral_key_vehicles_.behavioral_vehicles.at(vehicle_id).nearest_lane_id();
  common::Vehicle cur_vehicle = behavioral_key_vehicles_.behavioral_vehicles.at(vehicle_id).vehicle();

  decimal_t distance = GetDistanceToFinalPoint(cur_vehicle, nearest_lane_id);

  // NOTE for MLAD: 若有相遇冲突，选择前方反向行驶的vehicle的距离与距离终点距离的最小值
  distance = std::min(distance, behavioral_key_vehicles_.behavioral_vehicles.at(vehicle_id).opp_uncertain_vehicle_dist());

  // TODO for MLAD: 检查是否在反向行驶，若在反向行驶，则检查其前方最近的东西的距离
  // 如果当前车辆行驶方向与当前车道通行方向相反，则需要先进行变道，才能把当前车道的位置空出来，以供其他的车辆进行变道
  decimal_t cur_lane_dir = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).dir();
  bool is_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).cur_pass_dir_straight(); // 当前车道是否是在主方向上
  if (!is_main_dir) {
    // 车道方向与主方向相反
    cur_lane_dir -= M_PI;
  }

  decimal_t score;

  if (IsSameOrientation(cur_vehicle.state().angle, cur_lane_dir)
      && behavioral_key_vehicles_.behavioral_vehicles.at(vehicle_id).conflict_leading_uncertain_vehicle_id() == kInvalidAgentId
      && behavioral_key_vehicles_.behavioral_vehicles.at(vehicle_id).lane_drop_conflict_lane_id() == kInvalidLaneId) {
    // 车道方向与行驶方向相同，且前方指定距离里没有障碍物，且不处在drop lane中，分数则为距离
    score = distance;
  }
  else {
    // 相反，则需要优先进行变道，分数需要为负数
    score = -1 / (distance + kSmallEPS); // distance 越小，分数越小，优先级越大
  }

  return score;
}

bool SemanticMapManager::GetSpecVehicleCurrentTask(const int &ego_vehicle_id, common::Task *task) {
  // 1. 获取指定ego vehicle的当前的任务，调用lane_evaluation决定是否变道，若不存在此agv、agv不可变道、当前车道不能变道、不需要云端规划，返回false；否则返回true，并生成task
  auto iter = behavioral_key_vehicles_.behavioral_vehicles.find(ego_vehicle_id);
  if (iter == behavioral_key_vehicles_.behavioral_vehicles.end()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] not in behavioral_key_vehicles_ ";
    return false;
  }

  int nearest_lane_id = iter->second.nearest_lane_id();

  // 初始化task，默认初始化为保持直行
  task->is_under_ctrl = true;
  task->user_perferred_behavior = 0; // 默认不需要变道
  task->target_lane_id = nearest_lane_id;
  task->lc_info.target_lane_id = nearest_lane_id;
  task->lc_info.cur_lane_id = nearest_lane_id;
  task->user_desired_vel = agent_config_infos_.at(iter->second.vehicle().type()).desire_velocity;

  // 若沒有初始化過，需要先下發直行的task
  common::Task last_task;
  iter->second.GetLastTask(&last_task);
  if (!last_task.is_under_ctrl) {
    return true;
  }

  // 2. 此agv是否可以变道
  if (!iter->second.vehicle().ready_change_lane()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] not ready_change_lane \n";
    return false;
  }
  
  //LOG(ERROR) << "[GetSpecVehicleCurrentTask] nearest_lane_id: " << nearest_lane_id;
  // 3. 当前车道所处的位置是否为可以变道的车道
  if (behavioral_whole_lane_net_.behavioral_lanes.find(nearest_lane_id) == behavioral_whole_lane_net_.behavioral_lanes.end()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] nearest_lane_id: " << nearest_lane_id << " not in behavioral_whole_lane_net_";
    return false;
  }

  // 4. 首先查看与缓冲区的距离，若其小于变道最小所需的距离，则保持直行。其次再看其所在的车道的方向，与其行驶方向是否一样。最后再看两边车道分数
  decimal_t distance = GetDistanceToFinalPoint(iter->second.vehicle(), nearest_lane_id);
  if (distance - behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).buffer() < least_change_lane_distance_) {
    // 与缓冲区的距离,小于变道最小所需的距离，则直行
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] buffer";
    // 爲了讓正在變道的agv，在車道方向更新後，能停止變道，保持直行
    return true;
  }

  decimal_t cur_lane_dir = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).dir();
  bool is_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).cur_pass_dir_straight(); // 当前车道是否是在主方向上
  if (!is_main_dir) {
    // 车道方向与主方向相反
    cur_lane_dir -= M_PI;
  }

  // TODO for MLAD: 暂时还没有考虑，当一条车道出现动态障碍物时，其它车不能变道进入此车道的情况
  if (IsSameOrientation(iter->second.vehicle().state().angle, cur_lane_dir)) {
    // 行驶方向与车道方向相同
    std::unordered_map<int, int> cur_num_in_spec_lane;
    std::unordered_map<int, int> target_num_in_spec_lane;
    
    bool left_lane_available = false; // 是否可以转入左边车道
    int left_lane_id = kInvalidLaneId;
    if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).l_change_avbl()) {
      left_lane_id = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).l_lane_id();
      // 与当前车道方向是否一致
      if (is_main_dir == behavioral_whole_lane_net_.behavioral_lanes.at(left_lane_id).cur_pass_dir_straight()
          && !behavioral_whole_lane_net_.behavioral_lanes.at(left_lane_id).lane_drop_conflict()) {
        left_lane_available = true;
        cur_num_in_spec_lane.insert(std::make_pair(left_lane_id, 0));
        target_num_in_spec_lane.insert(std::make_pair(left_lane_id, 0));
      }
    }

    bool right_lane_available = false; // 是否可以转入左边车道
    int right_lane_id = kInvalidLaneId;
    if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).r_change_avbl()) {
      right_lane_id = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).r_lane_id();
      // 与当前车道方向是否一致
      if (is_main_dir == behavioral_whole_lane_net_.behavioral_lanes.at(right_lane_id).cur_pass_dir_straight()
          && !behavioral_whole_lane_net_.behavioral_lanes.at(right_lane_id).lane_drop_conflict()) {
        right_lane_available = true;
        cur_num_in_spec_lane.insert(std::make_pair(right_lane_id, 0));
        target_num_in_spec_lane.insert(std::make_pair(right_lane_id, 0));
      }
    }

    task->lc_info.forbid_lane_change_left = !left_lane_available;
    task->lc_info.forbid_lane_change_right = !right_lane_available;

    if (!left_lane_available && !right_lane_available) {
      // 5. 两边车道都不能去
      //LOG(ERROR) << "[GetSpecVehicleCurrentTask] two sides are inavalible";
      // 爲了讓正在變道的agv，在車道方向更新後，能停止變道，保持直行
      return true;
    }

    int pos_dir_vehicle_num=0, neg_dir_vehicle_num=0;
    cur_num_in_spec_lane.insert(std::make_pair(nearest_lane_id, 0));
    target_num_in_spec_lane.insert(std::make_pair(nearest_lane_id, 0));
    // 求target車道agv數量時，需要把自身給排除。只考虑当前车surrounding_search_radius * 2范围内的车
    GetVehicleNumInBehavioralLanes(ego_vehicle_id, &pos_dir_vehicle_num, &neg_dir_vehicle_num, &cur_num_in_spec_lane, &target_num_in_spec_lane);

    decimal_t vehicle_num_cur_dir = is_main_dir ? pos_dir_vehicle_num : neg_dir_vehicle_num; // 当前方向行驶的vehicle的数量
    int lane_num_cur_dir = 0; // 当前方向的车道数量
    for (const auto bl:behavioral_whole_lane_net_.behavioral_lanes) {
      if (bl.second.cur_pass_dir_straight() == is_main_dir) {
        lane_num_cur_dir++;
      }
    }

    decimal_t vehicle_num_per_lane = vehicle_num_cur_dir / lane_num_cur_dir;
    //LOG(WARNING) << "[GetSpecVehicleCurrentTask]  vehicle_num_per_lane : " << vehicle_num_per_lane + kSmallEPS;
    // TODO for MLAD: 需要注意当前是只用预期在此车道上的vehicle数量计算，后面是否需要改？
    //LOG(WARNING) << "[GetSpecVehicleCurrentTask] target_num_in_spec_lane.at(nearest_lane_id): " << target_num_in_spec_lane.at(nearest_lane_id);
    
    decimal_t left_lane_score = -kInf; // 主方向上看，左侧车道的评分
    if (left_lane_available) {
      //LOG(WARNING) << "[GetSpecVehicleCurrentTask] target_num_in_spec_lane.at(left_lane_id): " << target_num_in_spec_lane.at(left_lane_id);
      left_lane_score = (target_num_in_spec_lane.at(nearest_lane_id) - target_num_in_spec_lane.at(left_lane_id)) / (vehicle_num_per_lane + kSmallEPS);
      // TODO for MLAD: 当前只实现了计算公式的第一项，之后需要实现第二项，根据车道距离边缘的距离
      // 距離邊緣的距離不好計算，這裏只加一個向中心走的分數
      if (is_main_dir) {
        left_lane_score += impulse_to_center_lane_;
      }
    }

    //LOG(WARNING) << "[GetSpecVehicleCurrentTask] left_lane_score: " << left_lane_score;

    decimal_t right_lane_score = -kInf; // 主方向上看，右侧车道的评分
    if (right_lane_available) {
      //LOG(WARNING) << "[GetSpecVehicleCurrentTask] target_num_in_spec_lane.at(right_lane_id): " << target_num_in_spec_lane.at(right_lane_id);
      right_lane_score = (target_num_in_spec_lane.at(nearest_lane_id) - target_num_in_spec_lane.at(right_lane_id)) / (vehicle_num_per_lane + kSmallEPS);

      if (!is_main_dir) {
        right_lane_score += impulse_to_center_lane_;
      }
    }

    //LOG(WARNING) << "[GetSpecVehicleCurrentTask] right_lane_score: " << right_lane_score;

    if (std::fmax(left_lane_score, right_lane_score) < vehicle_change_lane_threshold_ 
        && iter->second.conflict_leading_uncertain_vehicle_id() == kInvalidAgentId
        && iter->second.lane_drop_conflict_lane_id() == kInvalidLaneId) {
      // 6. 不变道，保持当前车道
      // 判断当前车道前方是否有相遇冲突，若conflict_leading_uncertain_vehicle_id()是kInvalidAgentId则表明没有，否则需要进入后续流程进行变道
      //LOG(WARNING) << "[GetSpecVehicleCurrentTask] scores are both lower than threshold. The max score is: " << std::fmax(left_lane_score, right_lane_score);
    }
    else if (left_lane_score > right_lane_score) {
      // 左边车道得分高，说明向左变道。若当前为主方向，则真实向左，-1；若当前不为主方向，则真实向右
      // task->user_perferred_behavior = is_main_dir ? -1 : 1;
      task->user_perferred_behavior = -1;
      task->target_lane_id = left_lane_id;
      task->lc_info.target_lane_id = left_lane_id;
    }
    else if (left_lane_score < right_lane_score) {
      // 右边车道得分高，说明向右变道。若当前为主方向，则真实向右，1；若当前不为主方向，则真实向左
      // task->user_perferred_behavior = is_main_dir ? 1 : -1;
      task->user_perferred_behavior = 1;
      task->target_lane_id = right_lane_id;
      task->lc_info.target_lane_id = right_lane_id;
    }
    else {
      // 得分一样，则向中心变道
      // task->user_perferred_behavior = -1;
      task->user_perferred_behavior = is_main_dir ? -1 : 1;
      task->target_lane_id = is_main_dir ? left_lane_id : right_lane_id;
      task->lc_info.target_lane_id = task->target_lane_id;
    }

    // TODO for MLAD: 还需要检查目标车道前方是否有反向行驶的vehicle
  }
  else {
    // 行驶方向与车道方向相反，寻找相同的车道
    if (is_main_dir) {
      // 车道方向与主方向相同，说明vehicle需要向左变道，去往与主方向相反的车道
      if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).l_change_avbl()) {
        // 去往与主方向相反的车道，从主方向看，是向左变道，由于车是反向行驶，因此车需要向左变道
        task->user_perferred_behavior = -1; //向左
        task->target_lane_id = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).l_lane_id();
        task->lc_info.target_lane_id = task->target_lane_id;
      }
    }
    else {
      // 当前车道方向与主方向不同，说明vehicle需要向右变道，去往与主方向相同的车道
      if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).r_change_avbl()) {
        // 去往与主方向相同的车道，从主方向看，是向右变道
        task->user_perferred_behavior = 1; //向右
        task->target_lane_id = behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).r_lane_id();
        task->lc_info.target_lane_id = task->target_lane_id;
      }
    }
  }

  if (task->user_perferred_behavior == 0) {
    // 不变道，不会进入这里
    // TODO for MLAD: 删除这段？
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] task->user_perferred_behavior == 0 ";
    task->is_under_ctrl = true;
    task->user_desired_vel = agent_config_infos_.at(iter->second.vehicle().type()).desire_velocity;
    task->target_lane_id = nearest_lane_id;
    task->lc_info.target_lane_id = nearest_lane_id;
    task->lc_info.cur_lane_id = nearest_lane_id;
    // 爲了讓正在變道的agv，在車道方向更新後，能停止變道，保持直行
  }

  //LOG(ERROR) << "[SemanticMapManager::GetSpecVehicleCurrentTask] agv Id: " << ego_vehicle_id;
  //LOG(ERROR) << "[SemanticMapManager::GetSpecVehicleCurrentTask] task->user_perferred_behavior: " << task->user_perferred_behavior;

  return true;
}

bool SemanticMapManager::GetSpecVehicleCurrentTaskfromMsg(const int &ego_vehicle_id, common::Task *task){
  // 1. 获取指定ego vehicle的当前的任务，调用lane_evaluation决定是否变道，若不存在此agv、agv不可变道、当前车道不能变道、不需要云端规划，返回false；否则返回true，并生成task
  //LOG(ERROR) << "GetSpecVehicleCurrentTaskfromMsg" << ego_vehicle_id;
  //LOG(ERROR) << "[GetSpecVehicleCurrentTask] function in \n";
  auto iter = behavioral_key_vehicles_.behavioral_vehicles.find(ego_vehicle_id);
  if (iter == behavioral_key_vehicles_.behavioral_vehicles.end()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] not in behavioral_key_vehicles_ ";
    return false;
  }
  int nearest_lane_id = iter->second.nearest_lane_id();

  // 初始化task，默认初始化为保持直行
  task->is_under_ctrl = true;
  task->user_perferred_behavior = 0; // 默认不需要变道
  task->target_lane_id = nearest_lane_id;
  // task->lc_info.target_lane_id = nearest_lane_id;
  task->lc_info.cur_lane_id = nearest_lane_id;
  task->user_desired_vel = agent_config_infos_.at(iter->second.vehicle().type()).desire_velocity;

  // 若沒有初始化過，需要先下發直行的task
  common::Task last_task;
  iter->second.GetLastTask(&last_task);
  if (!last_task.is_under_ctrl) {
    return true;
  }

  // 2. 此agv是否可以变道
  if (!iter->second.vehicle().ready_change_lane()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] not ready_change_lane \n";
    return false;
  }
  
  //LOG(ERROR) << "[GetSpecVehicleCurrentTask] nearest_lane_id: " << nearest_lane_id;
  // 3. 当前车道所处的位置是否为可以变道的车道
  if (behavioral_whole_lane_net_.behavioral_lanes.find(nearest_lane_id) == behavioral_whole_lane_net_.behavioral_lanes.end()) {
    //LOG(ERROR) << "[GetSpecVehicleCurrentTask] nearest_lane_id: " << nearest_lane_id << " not in behavioral_whole_lane_net_";
    return false;
  }
  if(task_updated_){
      //LOG(ERROR) << "[SemanticMapManager::GetSpecVehicleCurrentTaskfromMsg" << task_updated_;
    task->is_under_ctrl = true;
    task->user_perferred_behavior = lane_change_tasks_.lc_tasks.find(ego_vehicle_id)->second.user_perferred_behavior;
    task->user_desired_vel = lane_change_tasks_.lc_tasks.find(ego_vehicle_id)->second.user_desired_vel;
    task->target_lane_id = lane_change_tasks_.lc_tasks.find(ego_vehicle_id)->second.target_lane_id;
    task->lc_info.target_lane_id = lane_change_tasks_.lc_tasks.find(ego_vehicle_id)->second.target_lane_id;
    task->lc_info.cur_lane_id = nearest_lane_id;
  }
  

  //LOG(ERROR) << "[SemanticMapManager::GetSpecVehicleCurrentTask] agv Id: " << ego_vehicle_id;
  //LOG(ERROR) << "[SemanticMapManager::GetSpecVehicleCurrentTask] task->user_perferred_behavior: " << task->user_perferred_behavior;

  return true;
}

ErrorType SemanticMapManager::UpdateUncertainVehicles(const common::VehicleSet &uncertain_vehicles) {
  set_uncertain_vehicles(uncertain_vehicles);
  uncertain_vehicle_ids_.clear();
  common::SemanticVehicleSet semantic_vehicles_tmp;
  for (const auto &v : uncertain_vehicles.vehicles) {
    uncertain_vehicle_ids_.emplace_back(v.first);

    common::SemanticVehicle semantic_vehicle;
    semantic_vehicle.vehicle = v.second;
    GetNearestLaneIdUsingState(
        semantic_vehicle.vehicle.state().ToXYTheta(), std::vector<int>(),
        &semantic_vehicle.nearest_lane_id, &semantic_vehicle.dist_to_lane,
        &semantic_vehicle.arc_len_onlane);

    NaiveRuleBasedLateralBehaviorPrediction(
        semantic_vehicle.vehicle, semantic_vehicle.nearest_lane_id,
        &semantic_vehicle.probs_lat_behaviors);
    semantic_vehicle.probs_lat_behaviors.GetMaxProbBehavior(
        &semantic_vehicle.lat_behavior);

    if (GetTargetLaneId(semantic_vehicle.nearest_lane_id, semantic_vehicle.lat_behavior, &semantic_vehicle.target_lane_id) != kSuccess) {
      // printf(
      //     "[GetRefLaneForStateByBehavior]fail to get target lane from lane %d "
      //     "with behavior %d.\n",
      //     current_lane_id, static_cast<int>(behavior));
      return kWrongStatus;
    }
    // GetRefLaneForStateByBehavior(
    //     semantic_vehicle.vehicle.state(), std::vector<int>(),
    //     semantic_vehicle.lat_behavior, forward_lane_len, max_backward_len,
    //     false, &semantic_vehicle.lane);

    semantic_vehicles_tmp.semantic_vehicles.insert(
        std::pair<int, common::SemanticVehicle>(semantic_vehicle.vehicle.id(),
                                                semantic_vehicle));
  }
  {
    semantic_uncertain_vehicles_.semantic_vehicles.swap(
        semantic_vehicles_tmp.semantic_vehicles);
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetLocalLaneSamplesByState(
    const common::State &state, const int lane_id,
    const std::vector<int> &navi_path, const decimal_t max_reflane_dist,
    const decimal_t max_backward_dist, vec_Vecf<2> *samples) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[GetLocalLaneSamplesByState]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }

  decimal_t arclen = 0.0;
  common::Lane target_lane = semantic_lane_set_.semantic_lanes.at(lane_id).lane;
  target_lane.GetArcLengthByVecPosition(state.vec_position, &arclen);
  decimal_t accum_dist_backward = 0.0;
  std::vector<int> ids_back;
  {
    if (arclen < max_backward_dist) {
      accum_dist_backward += arclen;
      int id_tmp = lane_id;
      while (accum_dist_backward < max_backward_dist) {
        std::vector<int> father_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).father_id;
        if (!father_ids.empty()) {
          // TODO: double check the logic for front
          int father_id = father_ids.front();
          for (auto &id : father_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              father_id = id;
              break;
            }
          }
          accum_dist_backward +=
              semantic_lane_set_.semantic_lanes.at(father_id).length;
          ids_back.push_back(father_id);
          id_tmp = father_id;
        } else {
          break;
        }
      }
    } else {
      accum_dist_backward = arclen;
    }
  }

  std::reverse(ids_back.begin(), ids_back.end());

  std::vector<int> ids_front;
  decimal_t accum_dist_forward = 0.0;
  {
    decimal_t dist_remain_target_lane =
        semantic_lane_set_.semantic_lanes.at(lane_id).length - arclen;
    if (dist_remain_target_lane < max_reflane_dist) {
      int id_tmp = lane_id;
      ids_front.push_back(lane_id);
      accum_dist_forward += dist_remain_target_lane;
      while (accum_dist_forward < max_reflane_dist) {
        std::vector<int> child_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).child_id;
        // TODO: double check the logic for front
        if (!child_ids.empty()) {
          int child_id = child_ids.front();
          for (auto &id : child_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              child_id = id;
              break;
            }
          }
          accum_dist_forward +=
              semantic_lane_set_.semantic_lanes.at(child_id).length;
          ids_front.push_back(child_id);
          id_tmp = child_id;
        } else {
          break;
        }
      }
    } else {
      ids_front.push_back(lane_id);
      accum_dist_forward = dist_remain_target_lane;
    }
  }

  std::vector<int> lane_id_all;
  lane_id_all.insert(lane_id_all.end(), ids_back.begin(), ids_back.end());
  lane_id_all.insert(lane_id_all.end(), ids_front.begin(), ids_front.end());

  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_id_all) {
    if (raw_samples.empty() &&
        (int)whole_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(
          whole_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1;
         i < (int)whole_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(
          whole_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }

  decimal_t acc_dist_tmp;
  decimal_t sample_start =
      std::max(0.0, accum_dist_backward - max_backward_dist);
  decimal_t forward_sample_len = std::min(max_reflane_dist, accum_dist_forward);
  SampleLane(long_lane, sample_start,
             sample_start + forward_sample_len +
                 std::min(accum_dist_backward, max_backward_dist),
             1.0, samples, &acc_dist_tmp);

  return kSuccess;
}

ErrorType SemanticMapManager::GetLocalLaneUsingLaneIds(
    const common::State &state, const std::vector<int> &lane_ids,
    const decimal_t forward_length, const decimal_t backward_length,
    const bool &is_high_quality, common::Lane *lane) {
  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_ids) {
    if (raw_samples.empty() &&
        whole_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < whole_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }
  decimal_t arc_len;
  long_lane.GetArcLengthByVecPosition(state.vec_position, &arc_len);

  vec_Vecf<2> samples;
  decimal_t acc_dist_tmp;
  decimal_t sample_start = std::max(0.0, arc_len - backward_length);
  decimal_t sample_end = std::min(arc_len + forward_length, long_lane.end());
  SampleLane(long_lane, sample_start, sample_end, 1.0, &samples, &acc_dist_tmp);

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetRefLaneForStateByBehavior(
    const common::State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality,
    common::Lane *lane) const {
  Vec3f state_3dof(state.vec_position(0), state.vec_position(1), state.angle);
  int current_lane_id;
  decimal_t distance_to_lane;
  decimal_t arc_len;
  if (GetNearestLaneIdUsingState(state_3dof, navi_path, &current_lane_id,
                                 &distance_to_lane, &arc_len) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  if (distance_to_lane > max_distance_to_lane_) {
    return kWrongStatus;
  }

  int target_lane_id;
  if (GetTargetLaneId(current_lane_id, behavior, &target_lane_id) != kSuccess) {
    // printf(
    //     "[GetRefLaneForStateByBehavior]fail to get target lane from lane %d "
    //     "with behavior %d.\n",
    //     current_lane_id, static_cast<int>(behavior));
    return kWrongStatus;
  }

  if (true && has_fast_lut_) {
    if (segment_to_local_lut_.end() !=
        segment_to_local_lut_.find(target_lane_id)) {
      // * here we just select the first local lane from several candidates
      int id = *segment_to_local_lut_.at(target_lane_id).begin();
      *lane = local_lanes_.at(id);
      return kSuccess;
    }
  }

  // ~ the reflane length should be consist with maximum speed and maximum
  // ~ forward simulation time, the current setup is for 30m/s x 7.5s forward
  vec_Vecf<2> samples;
  if (GetLocalLaneSamplesByState(state, target_lane_id, navi_path,
                                 max_forward_len, max_back_len,
                                 &samples) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get local lane samples.\n");
    return kWrongStatus;
  }

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetRefLaneForStateByBehavior(
    const common::State &state, const std::string &vehicle_type, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality,
    common::Lane *lane) const {
  Vec3f state_3dof(state.vec_position(0), state.vec_position(1), state.angle);
  int current_lane_id;
  decimal_t distance_to_lane;
  decimal_t arc_len;
  if (GetNearestLaneIdUsingState(state_3dof, navi_path, &current_lane_id,
                                 &distance_to_lane, &arc_len) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  if (distance_to_lane > max_distance_to_lane_) {
    return kWrongStatus;
  }

  int target_lane_id;
  if (GetTargetLaneId(current_lane_id, behavior, &target_lane_id) != kSuccess) {
    // printf(
    //     "[GetRefLaneForStateByBehavior]fail to get target lane from lane %d "
    //     "with behavior %d.\n",
    //     current_lane_id, static_cast<int>(behavior));
    return kWrongStatus;
  }

  if (agent_config_infos_.at(vehicle_type).enable_fast_lane_lut && has_fast_lut_) {
    if (segment_to_local_lut_.end() !=
        segment_to_local_lut_.find(target_lane_id)) {
      // * here we just select the first local lane from several candidates
      int id = *segment_to_local_lut_.at(target_lane_id).begin();
      *lane = local_lanes_.at(id);
      return kSuccess;
    }
  }

  // ~ the reflane length should be consist with maximum speed and maximum
  // ~ forward simulation time, the current setup is for 30m/s x 7.5s forward
  vec_Vecf<2> samples;
  if (GetLocalLaneSamplesByState(state, target_lane_id, navi_path,
                                 max_forward_len, max_back_len,
                                 &samples) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get local lane samples.\n");
    return kWrongStatus;
  }

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetLaneBySampledPoints(
    const vec_Vecf<2> &samples, const bool &is_high_quality,
    common::Lane *lane) const {
  if (is_high_quality) {
    double d = 0.0;
    std::vector<decimal_t> para;
    para.push_back(d);

    int num_samples = static_cast<int>(samples.size());
    for (int i = 1; i < num_samples; i++) {
      double dx = samples[i](0) - samples[i - 1](0);
      double dy = samples[i](1) - samples[i - 1](1);
      d += std::hypot(dx, dy);
      para.push_back(d);
    }

    const int num_segments = 20;
    Eigen::ArrayXf breaks =
        Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

    const decimal_t regulator = (double)1e6;
    if (common::LaneGenerator::GetLaneBySampleFitting(
            samples, para, breaks, regulator, lane) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    if (common::LaneGenerator::GetLaneBySamplePoints(samples, lane) !=
        kSuccess) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::SampleLane(const common::Lane &lane,
                                         const decimal_t &s0,
                                         const decimal_t &s1,
                                         const decimal_t &step,
                                         vec_E<Vecf<2>> *samples,
                                         decimal_t *accum_dist) const {
  Vecf<2> pt;
  for (decimal_t s = s0; s < s1; s += step) {
    lane.GetPositionByArcLength(s, &pt);
    samples->push_back(pt);
    (*accum_dist) += step;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetTargetLaneId(const int lane_id,
                                              const LateralBehavior &behavior,
                                              int *target_lane_id) const {
  auto it = semantic_lane_set_.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set_.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (behavior == common::LateralBehavior::kLaneKeeping ||
        behavior == common::LateralBehavior::kUndefined) {
      *target_lane_id = lane_id;
    } else if (behavior == common::LateralBehavior::kLaneChangeLeft) {
      if (it->second.l_change_avbl) {
        *target_lane_id = it->second.l_lane_id;
      } else {
        return kWrongStatus;
      }
    } else if (behavior == common::LateralBehavior::kLaneChangeRight) {
      if (it->second.r_change_avbl) {
        *target_lane_id = it->second.r_lane_id;
      } else {
        return kWrongStatus;
      }
    } else {
      assert(false);
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle,
    decimal_t *distance_residual_ratio) const {
  /**
   *    [>>>>]    ------->    [>>>>]
   *      | offset               |
   *      |                      |
   *  ------------------------------------ref lane
   *      lane_pt
   */
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  Vecf<2> lane_pt;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    return kWrongStatus;
  }
  ref_lane.GetPositionByArcLength(ref_fs.vec_s[0], &lane_pt);
  // Vecf<2> offset = ref_state.vec_position - lane_pt;

  // const decimal_t lane_width = 3.5;
  const decimal_t search_lat_radius = lat_range;
  const decimal_t max_forward_search_dist = 120.0;
  decimal_t search_lon_offset = 0.0;
  decimal_t resolution = search_lat_radius / 1.4;

  int leading_vehicle_id = kInvalidAgentId;

  bool find_leading_vehicle_in_set = false;
  bool find_occupied = false;
  common::Vehicle virtual_vehicle;

  for (decimal_t s = ref_fs.vec_s[0] + resolution + search_lon_offset;
       s < ref_fs.vec_s[0] + max_forward_search_dist + search_lon_offset;
       s += resolution) {
    decimal_t delta_s = s - ref_fs.vec_s[0];
    ref_lane.GetPositionByArcLength(s, &lane_pt);
    // lane_pt = lane_pt + offset;

    for (const auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).squaredNorm() <
          search_lat_radius * search_lat_radius) {
        find_leading_vehicle_in_set = true;
        leading_vehicle_id = entry.first;
        *distance_residual_ratio =
            (max_forward_search_dist - delta_s) / max_forward_search_dist;
        break;
      }
    }

    if (find_leading_vehicle_in_set) break;
  }

  if (find_leading_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(leading_vehicle_id);
    *leading_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetFollowingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *following_vehicle) const {
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    // printf("[FollowingVehicleOnLane]Cannot get ref state frenet state.\n");
    return kWrongStatus;
  }

  const decimal_t lane_width_tol = lat_range;
  decimal_t max_backward_search_dist =
      std::min(ref_fs.vec_s[0] - ref_lane.begin(), 100.0);

  int following_vehicle_id = kInvalidAgentId;
  Vecf<2> lane_pt;

  bool find_following_vehicle_in_set = false;

  for (decimal_t delta_s = lane_width_tol / 1.4;
       delta_s < max_backward_search_dist - 2.0 * lane_width_tol;
       delta_s += lane_width_tol / 1.4) {
    ref_lane.GetPositionByArcLength(ref_fs.vec_s[0] - delta_s, &lane_pt);
    for (auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).norm() <
          lane_width_tol) {
        find_following_vehicle_in_set = true;
        following_vehicle_id = entry.first;
        break;
      }
    }

    if (find_following_vehicle_in_set) break;
  }

  if (find_following_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(following_vehicle_id);
    *following_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

// ErrorType SemanticMapManager::GetSpeedLimit(const State &state,
//                                             const Lane &lane,
//                                             decimal_t *speed_limit) const {
//   return traffic_singal_manager_.GetSpeedLimit(state, lane, speed_limit);
// }

// ErrorType SemanticMapManager::GetTrafficStoppingState(
//     const State &state, const Lane &lane, State *stopping_state) const {
//   return traffic_singal_manager_.GetTrafficStoppingState(state, lane,
//                                                          stopping_state);
// }

bool SemanticMapManager::IsLocalLaneContainsLane(const int &local_lane_id,
                                                 const int &seg_lane_id) const {
  if (!has_fast_lut_) return false;
  auto ids = local_to_segment_lut_.at(local_lane_id);
  if (ids.end() != std::find(ids.begin(), ids.end(), seg_lane_id)) {
    return true;
  }
  return false;
}

// TODO(lu.zhang): Use general graph search instead in the future
ErrorType SemanticMapManager::GetDistanceOnLaneNet(const int &lane_id_0,
                                                   const decimal_t &arc_len_0,
                                                   const int &lane_id_1,
                                                   const decimal_t &arc_len_1,
                                                   decimal_t *dist) const {
  std::unordered_set<int> visited_list;
  std::set<std::pair<decimal_t, int>> pq;
  pq.insert(std::pair<decimal_t, int>(0.0, lane_id_0));
  decimal_t cost_lane_change = 10.0;

  int tar_node = lane_id_1;
  // decimal_t cost_aggre = -arc_len_0;

  while (!pq.empty()) {
    int cur_node = pq.begin()->second;

    if (cur_node == tar_node) {
      // finish
      break;
    }
    visited_list.insert(cur_node);
    std::vector<std::pair<int, decimal_t>> succ_nodes;

    // get successors
    {
      // child lane
      if (!whole_lane_net_.lane_set.at(cur_node).child_id.empty()) {
        auto ids = whole_lane_net_.lane_set.at(cur_node).child_id;
        auto cost = whole_lane_net_.lane_set.at(cur_node).length;
        for (const auto &id : ids) {
          succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
        }
      }

      // left lane
      if (whole_lane_net_.lane_set.at(cur_node).l_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).l_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }

      // right lane
      if (whole_lane_net_.lane_set.at(cur_node).r_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).r_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }
    }

    // for (int i = 0; i < succ_nodes.size(); ++i) {
    //   auto id = succ_nodes[i].first;
    //   auto it = visited_list.find(id);
    // }
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetSurroundingLaneNet(const common::LaneNet &lane_net, const common::Vehicle &ego_vehicle, 
                                                    const decimal_t &surrounding_search_radius, common::LaneNet *surrounding_lane_net) { 
  //TODO for MLAD: 需要改为根据车道、车道方向（才换方向的车道还要确认前方是否有低优先级的反向行驶agv）、当前agv Id，查找附近可以变道的车道
  surrounding_lane_net->clear();
  // TODO(lu.zhang): Use on lane distance instead

  lane_net_pts_.pts.clear();
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    for (int i = 0; i < static_cast<int>(iter->second.lane_points.size());
         ++i) {
      common::PointWithValue<int> p;
      int id = iter->second.id;
      p.pt.x = iter->second.lane_points[i](0);
      p.pt.y = iter->second.lane_points[i](1);
      p.values.push_back(id);
      p.values.push_back(i);
      lane_net_pts_.pts.push_back(p);
    }
  }
  kdtree_lane_net_ = std::make_shared<KdTreeFor2dPointVec>(
      2, lane_net_pts_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_lane_net_->buildIndex();

  const decimal_t query_pt[2] = {ego_vehicle.state().vec_position(0),
                                 ego_vehicle.state().vec_position(1)};
  const decimal_t search_radius =
      surrounding_search_radius * surrounding_search_radius * 4;
  std::vector<std::pair<size_t, decimal_t>> ret_matches;
  nanoflann::SearchParams params;
  // const size_t nMatches =
  kdtree_lane_net_->radiusSearch(&query_pt[0], search_radius, ret_matches,
                                 params);

  std::set<size_t> matched_lane_id_set;
  for (const auto e : ret_matches) {
    matched_lane_id_set.insert(lane_net_pts_.pts[e.first].values[0]);
  }

  for (const auto id : matched_lane_id_set) {
    surrounding_lane_net->lane_set.insert(
        std::pair<int, common::LaneRaw>(id, lane_net.lane_set.at(id)));
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetObstacleMap(const common::GridMapMetaInfo &obstacle_map_info, 
                                             const common::Vehicle &ego_vehicle, 
                                             GridMap2D *obstacle_map) {
  // ~ NOTICE:
  // ~ Origin of OccupancyGrid is at left-bottom corner,
  // ~ when x -> right, y -> up

  decimal_t x = ego_vehicle.state().vec_position(0) - obstacle_map_info.h_metric / 2.0;
  decimal_t y = ego_vehicle.state().vec_position(1) - obstacle_map_info.w_metric / 2.0;
  // Aligning to global coordinate to improve consistency
  decimal_t x_r = std::round(x);
  decimal_t y_r = std::round(y);

  // Use gridmap nd
  obstacle_map->fill_data(GridMap2D::UNKNOWN);
  std::array<decimal_t, 2> origin = {{x_r, y_r}};
  obstacle_map->set_origin(origin);

  cv::Mat grid_mat =
      cv::Mat(obstacle_map_info.height, obstacle_map_info.width,
              CV_MAKETYPE(cv::DataType<ObstacleMapType>::type, 1),
              obstacle_map->get_data_ptr());

  // Fill circle using opencv
  for (const auto &obs : obstacle_set_.obs_circle) {
    std::array<decimal_t, 2> center_w = {
        {obs.second.circle.center.x, obs.second.circle.center.y}};
    auto center_coord = obstacle_map->GetCoordUsingGlobalPosition(center_w);
    cv::Point2i center(center_coord[0], center_coord[1]);
    int radius = obs.second.circle.radius / obstacle_map_info.resolution;
    cv::circle(grid_mat, center, radius, cv::Scalar(GridMap2D::OCCUPIED), -1);
  }

  // Fill polygon using opencv
  std::vector<std::vector<cv::Point>> polys;
  for (const auto &obs : obstacle_set_.obs_polygon) {
    std::vector<cv::Point> poly;
    for (const auto &pt : obs.second.polygon.points) {
      std::array<decimal_t, 2> pt_w = {{pt.x, pt.y}};
      auto coord = obstacle_map->GetCoordUsingGlobalPosition(pt_w);
      cv::Point2i coord_cv(coord[0], coord[1]);
      poly.push_back(coord_cv);
    }
    polys.push_back(poly);
  }
  cv::fillPoly(grid_mat, polys, cv::Scalar(GridMap2D::OCCUPIED));

  return kSuccess;
}

ErrorType SemanticMapManager::FakeMapper(const common::GridMapMetaInfo &obstacle_map_info, 
                                         const common::Vehicle &ego_vehicle, 
                                         GridMap2D *obstacle_map, 
                                         std::set<std::array<decimal_t, 2>> *obstacle_grids) {
  decimal_t dist_thres = obstacle_map_info.h_metric / 2.0 * 0.8;
  if (RayCastingOnObstacleMap(ego_vehicle, obstacle_map, obstacle_grids) != kSuccess) {
    return kWrongStatus;
  }

  decimal_t ego_pos_x = ego_vehicle.state().vec_position(0);
  decimal_t ego_pos_y = ego_vehicle.state().vec_position(1);
  // Fill past obstacles and remove far ones
  for (auto it = obstacle_grids->begin(); it != obstacle_grids->end();) {
    decimal_t dx = abs((*it)[0] - ego_pos_x);
    decimal_t dy = abs((*it)[1] - ego_pos_y);
    if (dx >= dist_thres || dy >= dist_thres) {
      it = obstacle_grids->erase(it);
      continue;
    } else {
      obstacle_map->SetValueUsingGlobalPosition(
          *it, GridMap2D::SCANNED_OCCUPIED);
      ++it;
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::RayCastingOnObstacleMap(const common::Vehicle &ego_vehicle, 
                                                      GridMap2D *obstacle_map,
                                                      std::set<std::array<decimal_t, 2>> *obstacle_grids) {
  Vec3f ray_casting_origin;
  ego_vehicle.Ret3DofStateAtGeometryCenter(&ray_casting_origin);

  std::array<decimal_t, 2> origin = {
      {ray_casting_origin(0), ray_casting_origin(1)}};
  auto coord = obstacle_map->GetCoordUsingGlobalPosition(origin);
  int r_idx_max = obstacle_map->dims_size(0) * 3.0 / 4.0;

  std::vector<uint8_t> render_mat(obstacle_map->data_size(), 0);  // TODO
  for (int i = 0; i < 8; ++i) {
    std::set<std::array<int, 2>> obs_coord_set;
    rogue.RasterizeFOVOctant(
        coord[0], coord[1], r_idx_max, obstacle_map->dims_size(0),
        obstacle_map->dims_size(1), i, obstacle_map->data_ptr(),
        render_mat.data(), &obs_coord_set);
    for (const auto coord : obs_coord_set) {
      std::array<decimal_t, 2> p_w;
      obstacle_map->GetGlobalPositionUsingCoordinate(coord, &p_w);
      obstacle_grids->insert(p_w);
    }
  }
  for (int i = 0; i < obstacle_map->data_size(); ++i) {
    render_mat[i] = std::max(render_mat[i], obstacle_map->data(i));
  }

  obstacle_map->set_data(render_mat);

  return kSuccess;
}

void SemanticMapManager::InitBehavioralLaneSet() {
  behavioral_whole_lane_net_.Initialize(whole_lane_net_);
  all_behavioral_lane_ids_ = behavioral_whole_lane_net_.GetAllBehavioralLaneIds();
}

ErrorType SemanticMapManager::GetSpecEgoSemanticMapManager(const int &ego_vehicle_id, 
                                                           EgoSemanticMapManager *ego_smm) {
  std::lock_guard<std::mutex> lock_guard(vehicle_update_mtx); 

  if (ego_vehicle_ids_.count(ego_vehicle_id)==0) return kWrongStatus; 
  // 将对应vehicle的信息复制到ego_smm中
  common::Vehicle ego_vehicle = behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].vehicle();
  
  ego_smm->set_ego_id(ego_vehicle_id);

  decimal_t vehicle_dir = behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).vehicle().state().angle;
  decimal_t lane_main_dir = behavioral_whole_lane_net_.behavioral_lanes.at(behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).nearest_lane_id()).dir();
  bool reverse_raw_lane=false;
  if (!IsSameOrientation(vehicle_dir, lane_main_dir)) {
    reverse_raw_lane = true; // 需要將所有raw lane的sample點的順序反向
  }

  if (reverse_raw_lane) {
    ego_smm->set_whole_lane_net(whole_lane_net_.reverse());
  }
  else {
    ego_smm->set_whole_lane_net(whole_lane_net_);
  }
  ego_smm->set_agent_config_info(agent_config_infos_[ego_vehicle.type()]);
  ego_smm->set_time_stamp(time_stamp_);

  ego_smm->set_ego_vehicle(ego_vehicle);

  // obstacle的更新，仿照data_render中的函数
  common::GridMapMetaInfo obstacle_map_info = agent_config_infos_[ego_vehicle.type()].obstacle_map_meta_info;
  std::array<int, 2> map_size = {{obstacle_map_info.height, obstacle_map_info.width}};
  std::array<decimal_t, 2> map_resl = {{obstacle_map_info.resolution, obstacle_map_info.resolution}};
  std::array<std::string, 2> map_name = {{"height", "width"}};
  GridMap2D obstacle_map(map_size, map_resl, map_name);
  std::set<std::array<decimal_t, 2>> obstacle_grids;

  GetObstacleMap(obstacle_map_info, ego_vehicle, &obstacle_map);
  FakeMapper(obstacle_map_info, ego_vehicle, &obstacle_map, &obstacle_grids);

  ego_smm->set_obstacle_map(obstacle_map);
  ego_smm->set_obstacle_grids(obstacle_grids);

  ego_smm->set_priority_scores(priority_scores_);

  decimal_t surrounding_search_radius = agent_config_infos_[ego_vehicle.type()].surrounding_search_radius;

  // Get surrounding lane_net
  common::LaneNet surrounding_lane_net;
  if (GetSurroundingLaneNet(whole_lane_net_, ego_vehicle, surrounding_search_radius, &surrounding_lane_net) != kSuccess) {
    return kWrongStatus;
  }

  if (reverse_raw_lane) {
    ego_smm->set_surrounding_lane_net(surrounding_lane_net.reverse());
  }
  else {
    ego_smm->set_surrounding_lane_net(surrounding_lane_net);
  }

  // Get surrounding vehicles
  common::VehicleSet surrounding_vehicles;

  for (const auto &uv : uncertain_vehicles_.vehicles) {
    double dx = uv.second.state().vec_position(0) - ego_vehicle.state().vec_position(0);
    double dy = uv.second.state().vec_position(1) - ego_vehicle.state().vec_position(1);
    double dist = std::hypot(dx, dy);
    if (dist < surrounding_search_radius) {
      surrounding_vehicles.vehicles.insert(uv);
    }
  }

  for (const auto &kbv : behavioral_key_vehicles_.behavioral_vehicles) {
    if (kbv.first == ego_vehicle_id) continue;

    double dx = kbv.second.vehicle().state().vec_position(0) - ego_vehicle.state().vec_position(0);
    double dy = kbv.second.vehicle().state().vec_position(1) - ego_vehicle.state().vec_position(1);
    double dist = std::hypot(dx, dy);
    if (dist < surrounding_search_radius) {
      surrounding_vehicles.vehicles.insert(std::make_pair(kbv.first, kbv.second.vehicle()));
    }
  }
  ego_smm->set_surrounding_vehicles(surrounding_vehicles);

  ego_smm->UpdateEgoSemanticMap();

  return kSuccess;
}

ErrorType SemanticMapManager::GetSpecEgoLastReplanningContext(const int &ego_vehicle_id, 
                                                              common::ReplanningContext *context) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastReplanningContext(context);
}

ErrorType SemanticMapManager::GetSpecEgoLastTask(const int &ego_vehicle_id, common::Task *task) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastTask(task);
}

ErrorType SemanticMapManager::GetSpecEgoLastSnapshot(const int &ego_vehicle_id, common::Snapshot *snapshot) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastSnapshotForPlanning(snapshot);
}

ErrorType SemanticMapManager::GetSpecEgoLastLaneChangeProposal(const int &ego_vehicle_id, common::LaneChangeProposal *lc_proposal) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastLaneChangeProposal(lc_proposal);
}

ErrorType SemanticMapManager::GetSpecEgoLastPreliminaryActiveRequests(const int &ego_vehicle_id, 
                                                                      std::vector<common::ActivateLaneChangeRequest> *preliminary_active_requests) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastPreliminaryActiveRequests(preliminary_active_requests);
}

ErrorType SemanticMapManager::GetSpecEgoLastLaneChangeContext(const int &ego_vehicle_id, common::LaneChangeContext *lc_context) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastLaneChangeContext(lc_context);
}

ErrorType SemanticMapManager::GetAllBehavioralKeyVehiclesIds(std::unordered_set<int> *behavioral_key_vehicle_ids) {
  if (behavioral_key_vehicle_ids == nullptr) return kWrongStatus;

  std::lock_guard<std::mutex> lock_guard(vehicle_update_mtx); 

  for (const auto &id : key_vehicle_ids_) {
    behavioral_key_vehicle_ids->insert(id);
  }

  return kSuccess;
}

ErrorType SemanticMapManager::IsSpecVehicleRunningPlannedTraj(const int &behavioral_key_vehicle_id, const decimal_t &time, bool *is_running) {
  // 认定一旦vehicle变为不由云端控制变道的状态，就假定其当前的剩下的轨迹不真实
  if (ego_vehicle_ids_.find(behavioral_key_vehicle_id) == ego_vehicle_ids_.end()) {
    *is_running = false;
    if (surrounding_vehicle_ids_.find(behavioral_key_vehicle_id) == surrounding_vehicle_ids_.end()) {
      return kWrongStatus;
    }
    else {
      return kSuccess;
    }
  }

  // vehicle为可受控变道的vehicle
  *is_running = behavioral_key_vehicles_.behavioral_vehicles[behavioral_key_vehicle_id].IsRunningPlannedTrajAtSpecTime(time);

  return kSuccess;
}

ErrorType SemanticMapManager::GetSpecRunningVehicleStateAtSpecTime(const int &behavioral_key_vehicle_id, const decimal_t &time, common::State *state_output) {
  if (ego_vehicle_ids_.find(behavioral_key_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus; 

  return behavioral_key_vehicles_.behavioral_vehicles[behavioral_key_vehicle_id].GetExecutingStateAtSpecTime(time, state_output);
}

ErrorType SemanticMapManager::UpdateSpecEgoVehicleNextTask(const int &ego_vehicle_id, const common::Task &task) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) {
    //LOG(ERROR) << "[SemanticMapManager::UpdateSpecEgoVehicleNextTask] ego vehicle id update task fail " << ego_vehicle_id;
    return kWrongStatus;
  }

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextTask(task) != kSuccess) {
    return kWrongStatus;
  }
  
  // 更新目标车道
  int target_lane_id = task.target_lane_id;
  // if (task.user_perferred_behavior == 1) {
	// 	// 1为向右变道，-1为向左变道
		
	// }
  // else if (task.user_perferred_behavior == -1) {
  //   target_lane_id = whole_lane_net_.lane_set.at(behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).nearest_lane_id()).l_lane_id;
  // }
  // else {
  //   target_lane_id = behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).nearest_lane_id();
  // }

  if (target_lane_id == kInvalidLaneId) return kWrongStatus;

  behavioral_key_vehicles_.behavioral_vehicles.at(ego_vehicle_id).set_target_lane_id(target_lane_id);
  //LOG(ERROR) << "[SemanticMapManager::UpdateSpecEgoVehicleNextTask] ego vehicle id update task  ego_vehicle_id" << ego_vehicle_id;
  //LOG(ERROR) << "[SemanticMapManager::UpdateSpecEgoVehicleNextTask] ego vehicle id update task  target_lane_id" << target_lane_id;
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateSpecEgoVehicleNextSemanticBehavior(const int &ego_vehicle_id, const common::SemanticBehavior &behavior) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextSemanticBehavior(behavior);
}

ErrorType SemanticMapManager::UpdateSpecEgoVehicleNextTraj(const int &ego_vehicle_id, const vec_E<common::State> &state_traj, const std::string &behavior_uid) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextTraj(state_traj, behavior_uid);
}

ErrorType SemanticMapManager::UpdateTaskSetbyMsg(const common::TaskSet taskset){
  // if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;
  task_updated_ = false;
  lane_change_tasks_.lc_tasks.clear();
  // for(int i =0; i<taskset.lc_tasks.size(); i++){
  //   lane_change_tasks_.lc_tasks.emplace(i,taskset.lc_tasks.find(i)->second);
  // }
  for(auto k:taskset.lc_tasks){
    lane_change_tasks_.lc_tasks.emplace(k.first, k.second);
  }
  task_updated_ = true;
  //LOG(ERROR) << "[UpdateSemanticMap] lane_change_tasks_length" << lane_change_tasks_.lc_tasks.size();
  std::vector<int> rank;
  //LOG(ERROR) << "[UpdateSemanticMap] Start to get priority.";
  //GetPriorityRankingForEgoVehicles(&rank);
  for(int i = 8; i > 0; i--){
    rank.push_back(i); 
  }
  //LOG(ERROR) << "[UpdateSemanticMap] rank size is "<< rank.size();
  id_queue->UpdateQueue(rank);
  //LOG(ERROR) << "[UpdateSemanticMap] finish updating queue.\n";


  return kSuccess;
  //return behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextTask(task);
}

ErrorType SemanticMapManager::UpdateSpecEgoVehiclePlanningConfigs(const int &ego_vehicle_id, const common::ReplanningContext &context, 
      const common::Snapshot &snapshot, const common::LaneChangeProposal &lc_proposal, 
      const common::LaneChangeContext &lc_context, 
      const std::vector<common::ActivateLaneChangeRequest> &preliminary_active_requests) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextReplanningContext(context) != kSuccess) {
    return kWrongStatus;
  }

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextSnapshotForPlanning(snapshot) != kSuccess) {
    return kWrongStatus;
  } 

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextLaneChangeProposal(lc_proposal) != kSuccess) {
    return kWrongStatus;
  }

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextLaneChangeContext(lc_context) != kSuccess) {
    return kWrongStatus;
  }

  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextPreliminaryActiveRequests(preliminary_active_requests) != kSuccess) {
    return kWrongStatus;
  }

  // // TODO for MLAD: 當前暫時使用規劃時的軌跡，替代實際軌跡
  // vec_E<common::State> traj;
  // for (const common::Vehicle &v:snapshot.forward_trajs[snapshot.processed_winner_id]) {
  //   traj.emplace_back(v.state());
  // }
  // if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].UpdateNextTraj(traj) != kSuccess) {
  //   return kWrongStatus;
  // }

  return kSuccess;
}

ErrorType SemanticMapManager::GetVehicleOriginDesireVelocity(const std::string &type, decimal_t *desire_velocity) {
  if (agent_config_infos_.find(type) == agent_config_infos_.end()) {
    return kWrongStatus;
  }

  *desire_velocity = agent_config_infos_.at(type).desire_velocity;

  return kSuccess;
}

// 对指定的agv，生成新的下发消息的uid，并赋值
ErrorType SemanticMapManager::GetSpecEgoVehicleNewUID(const int &ego_vehicle_id, std::string *uid) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;

  std::string new_uid = GenerateUID();
  behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].set_behavior_uid(new_uid);
  *uid = new_uid;
  return kSuccess;
}

ErrorType SemanticMapManager::GetSpecEgoVehicleReplyComplete(const int &ego_vehicle_id, 
                                                             bool *wait_reply, bool *behavior_confirm) {
  if (ego_vehicle_ids_.find(ego_vehicle_id) == ego_vehicle_ids_.end()) return kWrongStatus;
  bool reply, confirm;
  std::string real_behavior_uid;
  ros::Time real_uid_time_stamp, cur_time_stamp = ros::Time::now();
  if (behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].GetLastBehaviorUIDWithTime(&real_behavior_uid, &real_uid_time_stamp) != kSuccess) {
    return kWrongStatus;
  }
  std::string updated_behavior_uid = behavioral_key_vehicles_.behavioral_vehicles[ego_vehicle_id].updated_behavior_uid();
  // printf("[GetSpecEgoVehicleReplyComplete] real_behavior_uid: %s, updated_behavior_uid: %s.\n", real_behavior_uid.c_str(), updated_behavior_uid.c_str());
  // wait_reply: 只有当需要继续等待时，才为true，即behavior_confirm为false，且此时无新车进入，或者新车进入的指定时间内部，继续等待
  if (real_behavior_uid == updated_behavior_uid) {
    reply = false;
    confirm = true;
  }
  else {
    // 设置等待确认回复只等待指定t秒？t秒后，之前规划出来的行为规划，已经失效，没有意义了
    if (cur_time_stamp.toSec() - real_uid_time_stamp.toSec() > max_wait_update_time_) {
      //LOG(ERROR) << "[GetSpecEgoVehicleReplyComplete] overtime. \n";
      reply = false;
    }
    else {
      reply = true;
    }

    confirm = false;
  }

  *wait_reply = reply;
  *behavior_confirm = confirm;
  return kSuccess;
}

void SemanticMapManager::PublishLanes() {
  std::vector<common::Lane> ref_lanes;
  std::vector<bool> cur_pass_dir_straights;

  for (auto iter=behavioral_whole_lane_net_.behavioral_lanes.begin(); 
        iter != behavioral_whole_lane_net_.behavioral_lanes.end();
        iter++) {
    ref_lanes.push_back(semantic_lane_set_.semantic_lanes.at(iter->first).lane);
    cur_pass_dir_straights.push_back(iter->second.cur_pass_dir_straight());
  }

  if (ref_lanes.empty()) {
    return;
  }

  p_behavioral_lane_visualizer->VisualizeDataWithStamp(ros::Time::now(), ref_lanes, cur_pass_dir_straights);
}

ErrorType SemanticMapManager::GetLeadingUncertainVehicleOnLane(
    const int &lane_id, const common::State &ref_state, const common::VehicleSet &vehicle_set,
    const decimal_t &lat_range, const decimal_t &max_forward_search_dist,
    int *leading_uncertain_vehicle_id,
    decimal_t *distance_to_vehicle) {
  /**
   *    [>>>>]    ------->    [>>>>]
   *      | offset               |
   *      |                      |
   *  ------------------------------------ref lane
   *      lane_pt
   */
  common::Lane &ref_lane = semantic_lane_set_.semantic_lanes.at(lane_id).lane;
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  Vecf<2> lane_pt;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    return kWrongStatus;
  }
  ref_lane.GetPositionByArcLength(ref_fs.vec_s[0], &lane_pt);
  // Vecf<2> offset = ref_state.vec_position - lane_pt;

  // const decimal_t lane_width = 3.5;
  const decimal_t search_lat_radius = lat_range;
  decimal_t search_lon_offset = 0.0;
  decimal_t resolution = search_lat_radius / 1.4;

  bool find_leading_vehicle_in_set = false;
  bool find_occupied = false;

  for (decimal_t s = ref_fs.vec_s[0] + resolution + search_lon_offset;
       s < ref_fs.vec_s[0] + max_forward_search_dist + search_lon_offset;
       s += resolution) {
    decimal_t delta_s = s - ref_fs.vec_s[0];
    ref_lane.GetPositionByArcLength(s, &lane_pt);
    // lane_pt = lane_pt + offset;

    for (const auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).squaredNorm() <
          search_lat_radius * search_lat_radius) {
        find_leading_vehicle_in_set = true;
        *leading_uncertain_vehicle_id = entry.first;
        *distance_to_vehicle = s - ref_fs.vec_s[0] - resolution - search_lon_offset;
        break;
      }
    }

    if (find_leading_vehicle_in_set) break;
  }

  if (find_leading_vehicle_in_set) {
    return kSuccess;
  } 
  else {
    return kWrongStatus;
  }
}

ErrorType SemanticMapManager::UncertainVehiclesAvoidanceCheck() {
  // Step 1: 将所有车道的opp_uncertain_vehicle_encounter_conflict设置为false
  for (auto iter=behavioral_whole_lane_net_.behavioral_lanes.begin(); 
        iter != behavioral_whole_lane_net_.behavioral_lanes.end();
        iter++) {
    iter->second.set_opp_uncertain_vehicle_encounter_conflict(false);
  }

  // Step 2: 检查每辆合作agv，判断其是否有相遇冲突
  for (auto id:ego_vehicle_ids_) {
    // 对所有参与变道的合作agv依次判断
    int temp_conflict_leading_uncertain_vehicle_id = kInvalidAgentId; // 默认没有相遇冲突
    decimal_t temp_opp_uncertain_vehicle_dist = kInf; // 默认为无穷大

    int nearest_lane_id = behavioral_key_vehicles_.behavioral_vehicles.at(id).nearest_lane_id();
    common::State ego_state = behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle().state();
    decimal_t opp_ego_dir = ego_state.angle - M_PI;

    // 首先选出所有目标车道在nearest_lane_id的、且行驶方向与当前车道反向的uncertain_vehicle
    common::VehicleSet vehicle_set;
    for (const auto& [id, sv]:semantic_uncertain_vehicles_.semantic_vehicles) {
      if (sv.target_lane_id == nearest_lane_id
          && IsSameOrientation(sv.vehicle.state().angle, opp_ego_dir)) {
        vehicle_set.vehicles.insert(std::pair<int, common::Vehicle>(id, sv.vehicle));
      }
    }

    int leading_uncertain_vehicle_id;
    decimal_t distance_to_vehicle;

    if (!vehicle_set.vehicles.empty() &&
        GetLeadingUncertainVehicleOnLane(nearest_lane_id, ego_state, vehicle_set, 
                                         search_lat_radius_, 
                                         agent_config_infos_.at(behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle().type()).surrounding_search_radius,
                                         &leading_uncertain_vehicle_id,
                                         &distance_to_vehicle) == kSuccess) {    
      // check the direct distance with the uncertain vehicle
      double dx = vehicle_set.vehicles.at(leading_uncertain_vehicle_id).state().vec_position(0) - ego_state.vec_position(0);
      double dy = vehicle_set.vehicles.at(leading_uncertain_vehicle_id).state().vec_position(1) - ego_state.vec_position(1);
      double dist = std::hypot(dx, dy);

      if (dist <= agent_config_infos_.at(behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle().type()).surrounding_search_radius) {
        // 直線距離小於surrounding_search_radius時，就需要設置衝突的id了，準備變道了
        behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).set_opp_uncertain_vehicle_encounter_conflict(true);
        temp_conflict_leading_uncertain_vehicle_id = leading_uncertain_vehicle_id;
        temp_opp_uncertain_vehicle_dist = distance_to_vehicle;

        // TODO for MLAD: 这里改为，id_queue为空时，触发变道，会不会更好？有障碍物时POMDP规划更加频繁，更安全，也可以避免规划失败后，只能停留在原地的尴尬？
        if (behavioral_key_vehicles_.behavioral_vehicles.at(id).conflict_leading_uncertain_vehicle_id() != temp_conflict_leading_uncertain_vehicle_id) {
          // 當障礙物和上一次不同時，觸發變道
          //LOG(ERROR) 
             // << "[UncertainVehiclesAvoidanceCheck] Find new leading opposite uncertain vehicle confict. Current ego id is "
            //  << id << ", uncertain vehicle id is " << leading_uncertain_vehicle_id;
          need_plan_now_ = true;
        }
        else if (id_queue->Empty()) {
          // 若前方障礙物的id是同一個，則需要確保id_queue已經爲空後，且距離當前vehicle上一次變道規劃的時間大於閾值，才觸發變道
          std::string last_behavior_uid;
          ros::Time last_uid_time_stamp;
          if (behavioral_key_vehicles_.behavioral_vehicles[id].GetLastBehaviorUIDWithTime(&last_behavior_uid, &last_uid_time_stamp) == kSuccess) {
            if (ros::Time::now().toSec() - last_uid_time_stamp.toSec() > uncertain_avoidance_replan_interval_) {
              //LOG(ERROR) 
               // << "[UncertainVehiclesAvoidanceCheck] still last leading opposite uncertain vehicle confict and would replan. Current ego id is " 
               // << id << ", uncertain vehicle id is " << leading_uncertain_vehicle_id;
              need_plan_now_ = true;
            }
          }
        }
      }
    }

    behavioral_key_vehicles_.behavioral_vehicles.at(id).set_conflict_leading_uncertain_vehicle_id(temp_conflict_leading_uncertain_vehicle_id); 
    behavioral_key_vehicles_.behavioral_vehicles.at(id).set_opp_uncertain_vehicle_dist(temp_opp_uncertain_vehicle_dist);
  }

  return kSuccess;
}

ErrorType SemanticMapManager::LaneDropAvoidanceCheck() {
  // TODO for MLAD: 这里需要考虑Lane Drop Bottleneck的问题

  // Step 1: 检查每辆合作agv，判断其是否有相遇冲突
  for (auto id:ego_vehicle_ids_) {
    // 对所有参与变道的合作agv依次判断
    int nearest_lane_id = behavioral_key_vehicles_.behavioral_vehicles.at(id).nearest_lane_id();

    if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).child_id().size() != 0) {
      // 当前车道的child id不是0个，则必不是需要drop 的lane
      behavioral_key_vehicles_.behavioral_vehicles.at(id).set_lane_drop_conflict_lane_id(kInvalidLaneId);
      behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).set_lane_drop_conflict(false);
      continue;
    }

    if (!behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).lane_drop_conflict()) {
      // 此时当前lane没有child，且不是drop lane，需要检查其距离终点是否还剩下指定距离
      decimal_t distance = GetDistanceToFinalPoint(behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle(), nearest_lane_id);
      if (distance < drop_lane_avoidance_distance_) {
        // 发现有车距离终点距离小于阈值，设置当前车道为true
        behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).set_lane_drop_conflict(true);
      }
    }

    if (behavioral_whole_lane_net_.behavioral_lanes.at(nearest_lane_id).lane_drop_conflict()) {
      // 当前车道是有lane drop的，需要进行变道
      if (behavioral_key_vehicles_.behavioral_vehicles.at(id).lane_drop_conflict_lane_id() != nearest_lane_id) {
        //LOG(ERROR) 
            //  << "[LaneDropAvoidanceCheck] nearest lane is drop-lane, which is not the same as previous one. Current ego id is "
            //  << id << ", nearest lane id is " << nearest_lane_id;
        need_plan_now_ = true;
      }
      else if (id_queue->Empty()) {
        // 若lane id是同一個，則需要確保id_queue已經爲空後，且距離當前vehicle上一次變道規劃的時間大於閾值，才觸發變道
        std::string last_behavior_uid;
        ros::Time last_uid_time_stamp;
        if (behavioral_key_vehicles_.behavioral_vehicles[id].GetLastBehaviorUIDWithTime(&last_behavior_uid, &last_uid_time_stamp) == kSuccess) {
          if (ros::Time::now().toSec() - last_uid_time_stamp.toSec() > uncertain_avoidance_replan_interval_) {
            //LOG(ERROR) 
            //  << "[LaneDropAvoidanceCheck] still drop-lane and would replan. Current ego id is " 
             // << id << ", nearest lane id is " << nearest_lane_id;
            need_plan_now_ = true;
          }
        }
      }
      behavioral_key_vehicles_.behavioral_vehicles.at(id).set_lane_drop_conflict_lane_id(nearest_lane_id);
      continue;
    }

    behavioral_key_vehicles_.behavioral_vehicles.at(id).set_lane_drop_conflict_lane_id(kInvalidLaneId);
    
  }
  return kSuccess;
}

ErrorType SemanticMapManager::StopVehicleCheck() {
  for (auto id:ego_vehicle_ids_) {
    // TODO for MLAD: 这里直接写了目标速度的十分之一，后期需要改为配置的吗
    if (behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle().state().velocity > 
        agent_config_infos_.at(behavioral_key_vehicles_.behavioral_vehicles.at(id).vehicle().type()).desire_velocity / 5) {
      continue;
    }

    if (id_queue->Empty()) {
        // 若lane id是同一個，則需要確保id_queue已經爲空後，且距離當前vehicle上一次變道規劃的時間大於閾值，才觸發變道
      std::string last_behavior_uid;
      ros::Time last_uid_time_stamp;
      if (behavioral_key_vehicles_.behavioral_vehicles[id].GetLastBehaviorUIDWithTime(&last_behavior_uid, &last_uid_time_stamp) == kSuccess) {
        if (ros::Time::now().toSec() - last_uid_time_stamp.toSec() > uncertain_avoidance_replan_interval_) {
          //LOG(ERROR) 
            //<< "[LaneDropAvoidanceCheck] vehicle is trapped and stop. Would replan. Current ego id is " << id;
          need_plan_now_ = true;
        }
      }
    }
  }
}

}  // namespace semantic_map_manager
