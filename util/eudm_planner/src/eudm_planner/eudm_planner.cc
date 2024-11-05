#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  //LOG(ERROR) << "\n[EudmPlanner] Loading eudm planner config";
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  if (!cfg_.IsInitialized()) {
    //LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();
  return kSuccess;
}

ErrorType EudmPlanner::Init(const std::string config) {
  ReadConfig(config);

  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer(),
                              cfg_.sim().duration().last_layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size();

  GetSimParam(cfg_.sim().ego(), &ego_sim_param_);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param_);

  rss_config_ = common::RssChecker::RssConfig(
      cfg_.safety().rss().response_time(),
      cfg_.safety().rss().longitudinal_acc_max(),
      cfg_.safety().rss().longitudinal_brake_min(),
      cfg_.safety().rss().longitudinal_brake_max(),
      cfg_.safety().rss().lateral_acc_max(),
      cfg_.safety().rss().lateral_brake_min(),
      cfg_.safety().rss().lateral_brake_max(),
      cfg_.safety().rss().lateral_miu());

  rss_config_strict_as_front_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_front().response_time(),
      cfg_.safety().rss_strict_as_front().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_acc_max(),
      cfg_.safety().rss_strict_as_front().lateral_brake_min(),
      cfg_.safety().rss_strict_as_front().lateral_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_miu());

  rss_config_strict_as_rear_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_rear().response_time(),
      cfg_.safety().rss_strict_as_rear().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_acc_max(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_min(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_miu());

  return kSuccess;
}

ErrorType EudmPlanner::TranslateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      //LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      //LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<DcpAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  // NOTE for MLAD: 若有变道动作，则lat_behavior为相应的第一个变道动作；若有变道动作且有直行动作，则is_cancel_operation为true
  // 若无变道动作，则lat_behavior为保持车道，is_cancel_operation为false，operation_at_seconds需要加一个layer()
  return kSuccess;
}

ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  // LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
  //           << " threads.";

  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  cooperative_info_.clear();
  cooperative_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  cooperative_cost_.clear();
  cooperative_cost_.resize(n_sequence);

  tail_cost_.clear();
  tail_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

ErrorType EudmPlanner::GetSurroundingForwardSimAgents(
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    ForwardSimAgentSet* fsagents) const { 
  // 在这里将所有的surrounding_semantic_vehicles中的common::SemanticVehicle，放在fsagents->forward_sim_agents
  for (const auto& psv : surrounding_semantic_vehicles.semantic_vehicles) {
    ForwardSimAgent fsagent;

    int id = psv.second.vehicle.id();
    fsagent.id = id;
    fsagent.vehicle = psv.second.vehicle;
    
    common::State state = psv.second.vehicle.state();

    // * lon
    if (behavioral_key_vehicle_ids_.find(id) == behavioral_key_vehicle_ids_.end()) {
      fsagent.sim_param = agent_sim_param_;
        if (state.acceleration >= 0) {
        fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
      } else {
        decimal_t est_vel =
            std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
        fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
      }
    }
    else {
      fsagent.sim_param = ego_sim_param_;
      fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    }

    // ~ If other vehicles' acc > 0, we assume constant velocity
    // NOTE for MLAD: 如果其它的合作agv的加速度大于0，则需要设置速度为匀速（以当前速度），若小于0，则设置为模拟时间结束时的速度为匀速
    // if (state.acceleration >= 0) {
    //   fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    // } else {
    //   decimal_t est_vel =
    //       std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
    //   fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
    // }

    // * lat
    fsagent.lat_probs = psv.second.probs_lat_behaviors;
    fsagent.lat_behavior = psv.second.lat_behavior;

    fsagent.lane = psv.second.lane;
    fsagent.stf = common::StateTransformer(fsagent.lane);

    // * other
    fsagent.lat_range = cfg_.sim().agent().cooperative_lat_range();

    fsagents->forward_sim_agents.insert(std::make_pair(id, fsagent));
  }

  return kSuccess;
}

ErrorType EudmPlanner::RunEudm() {
  // * get relevant information
  common::SemanticVehicleSet surrounding_semantic_vehicles;
  // TODO for MLAD: 這裏合作agv的橫向語義動作，默認都是直行，需要根據實際情況來改？這裏是否重要？
  if (map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles) != // TODO for MLAD: 从surrounding_semantic_vehicles这里拿出所有agv，包含了真实的合作AGV（可变道和不可变道agv），以及非合作aagv
      kSuccess) {
    //LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }

  behavioral_key_vehicle_ids_.clear();
  if (map_itf_->GetBehavioralKeyVehiclesIds(&behavioral_key_vehicle_ids_) != kSuccess) {
    //LOG(ERROR) << "[Eudm][Fatal]fail to get behavioral key vehicle ids. Exit";
    return kWrongStatus;
  }

  ForwardSimAgentSet surrounding_fsagents;
  GetSurroundingForwardSimAgents(surrounding_semantic_vehicles,
                                 &surrounding_fsagents); 

  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // * prepare for multi-threading
  // 首先筛去不合格的acion
  std::vector<int> valid_seq;
  for (int i = 0; i < n_sequence; ++i) {
    if (pre_deleted_seq_ids_.find(i) != pre_deleted_seq_ids_.end()) {
      sim_res_[i] = 0;
      sim_info_[i] = std::string("(Pre-deleted)");
    }
    else {
      valid_seq.push_back(i);
    }
  }

  std::vector<std::thread> thread_set(valid_seq.size());
  PrepareMultiThreadContainers(n_sequence); // 这里还是初始化n sequence，但不合格的动作对应的数组为空

  // * threading
  // TODO(@lu.zhang) Use thread pool?
  TicToc timer;
  for (int i = 0; i < valid_seq.size(); ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle_,
                    surrounding_fsagents, surrounding_semantic_vehicles, action_script[valid_seq[i]], valid_seq[i]);
  }
  for (int i = 0; i < valid_seq.size(); ++i) {
    thread_set[i].join();
  }

  // LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // * finish multi-threading, summary simulation results
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  for (int i : valid_seq) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    line_info << cooperative_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[Eudm][Progress cost][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]" << "\n";

      line_info << "[Eudm][Cooperative cost][e;s;w:";
      for (const auto& c : cooperative_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.weight;
        line_info << "|";
      }
      line_info << "]" << "\n";
    }
    //LOG(WARNING) << line_info.str();
  }
  //LOG(ERROR) << "[Eudm][Result]Sim status: " << sim_success << " with "
  //            << num_valid_behaviors << " behaviors.";
  if (!sim_success) {
    //LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // * evaluate
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    //LOG(ERROR)
    //    << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoBehaviorsUsingAction(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    //LOG(ERROR) << "[Eudm]Translate action error";
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * Get the type of lateral action sequence
  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // * get action sequence type
  if (is_cancel_behavior) { 
    //先有变道动作，然后变成保持当前车道动作
    ego_fsagent->lat_behavior_longterm = LateralBehavior::kLaneKeeping;
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
    }
    ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
  }

  // * lon
  decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity); //  對小車agv，速度只有0.4，向下取整直接變爲0了
  // decimal_t desired_vel = ego_fsagent->vehicle.state().velocity;
  simulator::IntelligentDriverModel::Param idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;
  // NOTE for MLAD: 前向的速度，根据第二个来判断desire速度
  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      idm_param_tmp.kDesiredVelocity = std::min(
          desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      break;
    }
    case DcpLonAction::kDecelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      //LOG(ERROR) << "[Eudm]Error - Lon action not valid";
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;
  ego_fsagent->lat_range = cfg_.sim().ego().cooperative_lat_range();

  // TODO for MLAD: 當前只考慮，各車道一直平行，變道過程中不會出現可變道車道合並或者結束的事情。因此，longterm lane是一開始就可以確定的
  // * longterm lanes
  auto state = ego_fsagent->vehicle.state();
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);
  // ego_fsagent->longterm_lane = task_target_lane_;
  // ego_fsagent->longterm_stf = task_target_stf_;

  return kSuccess;
}

ErrorType EudmPlanner::UpdateSimSetupForLayer(
    const DcpAction& action, const ForwardSimAgentSet& other_fsagent, const bool& lat_action_finished,
    ForwardSimEgoAgent* ego_fsagent) const {
  // TODO for MLAD：对于ego要变道，但目标车道有落后于ego车的合作other车，需要将other车的gap_front_id设为ego车，让other车减速避让；对于保持车道的ego车，若左右两边车道前方有打算变道到此车道的合作other车，需要在gap_front_id处添加对应车
  // TODO for MLAD: 因此需要对高优先级的车添加其变道意向
  // * action -> behavior
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  // * related lanes
  auto state = ego_fsagent->vehicle.state();
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  common::Lane lane_current;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), LateralBehavior::kLaneKeeping,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_current) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->current_stf = common::StateTransformer(lane_current);

  // TODO for MLAD: 设置为lat_behavior_longterm，让CAV在kl阶段就提前减速，能更快变道
  common::Lane lane_target;
  if (cfg_.sim().explicit_cooperation_enable()) {
    if (map_itf_->GetRefLaneForStateByBehavior(
            state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
            forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
            &lane_target) != kSuccess) {
      return kWrongStatus;
    }
  }
  else {
    if (map_itf_->GetRefLaneForStateByBehavior(
            state, std::vector<int>(), ego_fsagent->lat_behavior,
            forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
            &lane_target) != kSuccess) {
      return kWrongStatus;
    }
  }
  
  ego_fsagent->target_lane = lane_target;
  ego_fsagent->target_stf = common::StateTransformer(lane_target);
  
  // common::Lane lane_target;
  // if (map_itf_->GetRefLaneForStateByBehavior(
  //         state, std::vector<int>(), ego_fsagent->lat_behavior,
  //         forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
  //         &lane_target) != kSuccess) {
  //   return kWrongStatus;
  // }
  // ego_fsagent->target_lane = lane_target;
  // ego_fsagent->target_stf = common::StateTransformer(lane_target);

  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

  // * Gap finding for lane-changing behaviors
  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    common::VehicleSet other_vehicles;
    for (const auto& pv : other_fsagent.forward_sim_agents) {
      other_vehicles.vehicles.insert(
          std::make_pair(pv.first, pv.second.vehicle));
    }

    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ego_fsagent->target_lane, state, other_vehicles, &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    ego_fsagent->target_gap_ids(0) =
        has_front_vehicle ? front_vehicle.id() : -1;
    ego_fsagent->target_gap_ids(1) = has_rear_vehicle ? rear_vehicle.id() : -1;

    if (cfg_.safety().rss_for_layers_enable()) {
      // * Strict RSS check here
      // * Disable the action that is apparently not valid
      common::FrenetState ego_fs;
      if (kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(
                          ego_fsagent->vehicle.state(), &ego_fs)) {
        return kWrongStatus;
      }
      decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();
      decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();

      if (has_front_vehicle) {
        decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                    front_vehicle.param().length() / 2.0 +
                                    front_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            front_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Front,
            rss_config_strict_as_rear_, &rss_dist);

        if (s_front_rbumper - s_ego_fbumper < rss_dist) {
          // violate strict RSS
          return kWrongStatus;
        }
      }

      if (has_rear_vehicle) {
        decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            rear_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Rear,
            rss_config_strict_as_front_, &rss_dist);

        if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
          // violate strict RSS
          return kWrongStatus;
        }
      }
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::SimulateScenario(
    const common::Vehicle& ego_vehicle,
    const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id,
    const int& sub_seq_id, const std::vector<int>& impeding_vehicle_ids,
    std::vector<int>* sub_sim_res, std::vector<int>* sub_risky_res, 
    std::vector<std::string>* sub_sim_info,
    std::vector<std::vector<CostStructure>>* sub_progress_cost,
    std::vector<CostStructure>* sub_tail_cost,
    std::vector<std::vector<CostStructure>>* sub_cooperate_cost,
    vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
    std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
    std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
        sub_surround_trajs) {
  // * declare variables which will be used to track traces from multiple layers
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};

  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto& p_fsa : surrounding_fsagents.forward_sim_agents) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        p_fsa.first, vec_E<common::Vehicle>({p_fsa.second.vehicle})));
  }

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;
  std::set<int> risky_ids_multilayers;
  std::vector<CostStructure> cost_coop_multilayers;

  // // * Setup ego longitudinal sim config
  // ForwardSimEgoAgent ego_fsagent_this_layer;
  // ego_fsagent_this_layer.vehicle = ego_vehicle;
  // UpdateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);
  ForwardSimEgoAgent ego_fsagent_this_layer = ego_fsagent;
  ForwardSimAgentSet surrounding_fsagents_this_layer = surrounding_fsagents;

  int action_ref_lane_id = ego_lane_id_;
  bool lat_action_finished = false;
  bool is_sub_seq_risky = false;
  std::vector<DcpAction> action_seq_sim = action_seq;

  // * For each action in action sequence
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];

    // For each action, we can update context here.
    // such as lane, stf, desired_vel, social_force_masks
    // For each action, the context info will not change, so we can use it in
    // every step. A low-level reactive lane-changing controller can be
    // implemented without a lot of computation cost.
    // * update setup for this layer
    if (kSuccess != UpdateSimSetupForLayer(action_this_layer,
                                           surrounding_fsagents_this_layer,
                                           lat_action_finished,
                                           &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // TODO(lu.zhang): MOBIL/RSS check here?
    // * Disable the action that is apparently not valid
    // if (ego_lat_behavior_this_layer != LateralBehavior::kLaneKeeping) {
    //   if (!CheckLaneChangingFeasibilityUsingMobil(
    //           ego_semantic_vehicle_this_layer,
    //           semantic_vehicle_set_this_layer)) {
    //     return kWrongStatus;
    //   }
    // }

    // * simulate this action (layer)
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // * update ForwardSimAgent
    ego_fsagent_this_layer.vehicle.set_state(
        ego_traj_multisteps.back().state());
    for (auto it = surrounding_fsagents_this_layer.forward_sim_agents.begin();
         it != surrounding_fsagents_this_layer.forward_sim_agents.end(); ++it) {
      it->second.vehicle.set_state(
          surround_trajs_multisteps.at(it->first).back().state());
    }

    // * enforce strict safety check
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }
    // LOG(INFO) << "[RssTime]safety check time per action: " << timer.toc();

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // * If lateral action finished, update simulation action sequence
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_fsagent_this_layer.vehicle.state(), action_ref_lane_id,
            ego_fsagent_this_layer.lat_behavior, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      lat_action_finished = true;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
      ego_fsagent_this_layer.lat_behavior_longterm =
          LateralBehavior::kLaneKeeping;
    }

    // * trace
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.push_back(ego_fsagent_this_layer.lon_behavior);

    for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    // ego vehicle cost
    CostStructure ego_cost;
    bool verbose = false;
    std::set<int> risky_ids;
    bool is_risky_action = false;
    EgoCostFunction(action_this_layer, ego_fsagent_this_layer,
                 surrounding_fsagents_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, verbose, &ego_cost, &is_risky_action,
                 &risky_ids);
    if (is_risky_action) {
      is_sub_seq_risky = true;
      for (const auto& id : risky_ids) {
        risky_ids_multilayers.insert(id);
      }
    }
    ego_cost.weight = ego_cost.weight * pow(cfg_.cost().discount_factor(), i);
    ego_cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(ego_cost);

    // cooperative cost
    CostStructure coop_cost;
    bool is_risky_coop = false;
    CooperativeCostFunction(action_this_layer, ego_fsagent_this_layer,
                  surrounding_fsagents_this_layer, ego_traj_multisteps,
                  surround_trajs_multisteps, impeding_vehicle_ids,
                  &coop_cost, &is_risky_coop);
    coop_cost.weight = coop_cost.weight * pow(cfg_.cost().discount_factor(), i);
    coop_cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_coop_multilayers.push_back(coop_cost);
  }

  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids_multilayers.begin();
         it != risky_ids_multilayers.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_cooperate_cost)[sub_seq_id] = cost_coop_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

ErrorType EudmPlanner::GetNearestImpedeLCBehavioralVehicles(
    const ForwardSimEgoAgent& ego_fsagent, 
    const ForwardSimAgentSet& surrounding_fsagents,
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    int *lead_vehicle_id,
    int *follow_vehicle_id) {
  // 如果长期目标是保持车道，则完全没必要考虑implicit coop，因为不需要低优先级提前减速避让
  if (ego_fsagent.lat_behavior_longterm == common::LateralBehavior::kLaneKeeping) return kSuccess;

  bool has_leading_vehicle = false, has_following_vehicle = false;
  common::Vehicle leading_vehicle, following_vehicle;
  common::FrenetState leading_fs, following_fs;
  decimal_t distance_residual_ratio = 0.0;
  
  common::VehicleSet BehavioralVehicleSet;
  for (const auto& sv : surrounding_semantic_vehicles.semantic_vehicles) {
    if (behavioral_key_vehicle_ids_.find(sv.first) != behavioral_key_vehicle_ids_.end()) {
      BehavioralVehicleSet.vehicles.insert(std::pair<int, common::Vehicle>(sv.first, sv.second.vehicle));
    }
  }

  if (map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(ego_fsagent.longterm_lane, 
          ego_fsagent.vehicle.state(), BehavioralVehicleSet, &has_leading_vehicle, 
          &leading_vehicle, &leading_fs, &has_following_vehicle, &following_vehicle,
          &following_fs) != kSuccess) {
    return kWrongStatus;
  }

  if (has_leading_vehicle
      && CheckIfCoopImpedeLaneChange(ego_fsagent, surrounding_fsagents, surrounding_semantic_vehicles, leading_vehicle.id())) {
    *lead_vehicle_id = leading_vehicle.id();
  }

  if (has_following_vehicle 
      && CheckIfCoopImpedeLaneChange(ego_fsagent, surrounding_fsagents, surrounding_semantic_vehicles, following_vehicle.id())) {
    *follow_vehicle_id = following_vehicle.id();
  }
  
  return kSuccess;
}

bool EudmPlanner::CheckIfCoopImpedeLaneChange(
    const ForwardSimEgoAgent& ego_fsagent, 
    const ForwardSimAgentSet& surrounding_fsagents,
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    const int& potential_vehicle_id) {
  // First, check the priority.
  bool is_ego_prior = false;
  if (map_itf_->IsEgoPriorityHigher(ego_fsagent.vehicle.id(), potential_vehicle_id, &is_ego_prior) != kSuccess) {
    return false;
  }

  if (!is_ego_prior) return false;

  // Second, check wether the lower priority CAV impede ego to change lane
  common::SemanticVehicle potential_sm_vehicle = surrounding_semantic_vehicles.semantic_vehicles.at(potential_vehicle_id);
  common::FrenetState potential_fs, ego_fs;
  if (ego_fsagent.longterm_stf.GetFrenetStateFromState(potential_sm_vehicle.vehicle.state(), &potential_fs) != kSuccess) {
    return false;
  }

  if (ego_fsagent.longterm_stf.GetFrenetStateFromState(ego_fsagent.vehicle.state(), &ego_fs) != kSuccess) {
    return false;
  }

  // TODO for MLAD：应该是各自open simulation，查看是否会碰撞，但这里先不这样做，只看frenet下，投影距离是否小于指定距离
  decimal_t impede_distance = ego_fsagent.vehicle.param().length()/2 + potential_sm_vehicle.vehicle.param().length()/2;
  impede_distance += ego_fsagent.sim_param.idm_param.kMinimumSpacing;
  impede_distance += surrounding_fsagents.forward_sim_agents.at(potential_vehicle_id).sim_param.idm_param.kMinimumSpacing;
  impede_distance += ego_fsagent.sim_param.idm_param.kDesiredHeadwayTime
                     * fmin(ego_fsagent.vehicle.state().velocity, surrounding_fsagents.forward_sim_agents.at(potential_vehicle_id).vehicle.state().velocity) / 4;
  //LOG(ERROR) << "impede_distance: " << impede_distance;
  if (fabs(ego_fs.vec_s[0] - potential_fs.vec_s[0]) < impede_distance) {
    return true;
  }

  return false;
}

ErrorType EudmPlanner::UpdateSurroundingSimSetupForScenario(const std::unordered_map<int, common::LongitudinalBehavior>& lon_actions,
                                                            ForwardSimAgentSet* other_fsagent) {
  for (const auto& lon_a : lon_actions) {
    if (other_fsagent->forward_sim_agents.find(lon_a.first) == other_fsagent->forward_sim_agents.end()) {
      continue;
    }
    decimal_t desired_vel = std::floor(other_fsagent->forward_sim_agents.at(lon_a.first).sim_param.idm_param.kDesiredVelocity); // 默认是当前速度
    simulator::IntelligentDriverModel::Param idm_param_tmp;
    idm_param_tmp = ego_sim_param_.idm_param;
    // TODO for MLAD: 默认所有CAV的desired_velocity_都是一样的。后期要改的话，怎么改？
    switch (lon_a.second) {
      case common::LongitudinalBehavior::kAccelerate: {
        idm_param_tmp.kDesiredVelocity = std::min(
            desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
        idm_param_tmp.kMinimumSpacing *=
            (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
        idm_param_tmp.kDesiredHeadwayTime *=
            (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
        break;
      }
      case common::LongitudinalBehavior::kDecelerate: {
        idm_param_tmp.kDesiredVelocity =
            std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                    desired_velocity_);
        break;
      }
    }

    other_fsagent->forward_sim_agents.at(lon_a.first).sim_param.idm_param = idm_param_tmp;
  }

  return kSuccess;
}

ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    const std::vector<DcpAction>& action_seq, const int& seq_id) {
  // if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
  //   sim_res_[seq_id] = 0;
  //   sim_info_[seq_id] = std::string("(Pre-deleted)");
  //   return kWrongStatus;
  // }

  // ~ For each ego sequence, we may further branch here, which will create
  // ~ multiple sub threads. Currently, we use n_sub_threads = 1
  // TODO(@lu.zhang) Preliminary safety assessment here

  // * Setup ego longitudinal sim config
  ForwardSimEgoAgent ego_fsagent;
  ego_fsagent.vehicle = ego_vehicle;
  UpdateEgoSimSetupForScenario(action_seq, &ego_fsagent);

  int leading_vehicle_id = kInvalidAgentId;
  int follow_vehicle_id = kInvalidAgentId;
  if (cfg_.cost().cooperation().cooperative_cost_enable()) {
    GetNearestImpedeLCBehavioralVehicles(ego_fsagent, surrounding_fsagents,
                      surrounding_semantic_vehicles, &leading_vehicle_id, &follow_vehicle_id);
  }

  // TODO for MLAD: 在这里获取ego_vehicle leading vehicle可能的longt动作，包含vehicle id

  // TODO for MLAD: 在这里获取rear vehicle可能的longt动作，包含id（如果不是CAV，则返回kWrong?）
  int n_sub_threads = 1;

  std::vector<int> impeding_vehicle_ids;
  std::vector<common::LongitudinalBehavior> leading_lon_actions{common::LongitudinalBehavior::kMaintain};
  if (leading_vehicle_id != kInvalidAgentId) {
    n_sub_threads *= 2;
    leading_lon_actions.push_back(common::LongitudinalBehavior::kAccelerate);
    impeding_vehicle_ids.push_back(leading_vehicle_id);
  }

  std::vector<common::LongitudinalBehavior> following_lon_actions{common::LongitudinalBehavior::kMaintain};
  if (follow_vehicle_id != kInvalidAgentId) {
    n_sub_threads *= 2;
    following_lon_actions.push_back(common::LongitudinalBehavior::kDecelerate);
    impeding_vehicle_ids.push_back(follow_vehicle_id);
  }

  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
  std::vector<CostStructure> sub_tail_cost(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_cooperate_cost(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  std::unordered_map<int, std::string> sub_thread_info(n_sub_threads);
  
  if (n_sub_threads == 1) {
    sub_thread_info[0] = "no cooperative vehicle need to be considered.";
    SimulateScenario(ego_vehicle, ego_fsagent, surrounding_fsagents, action_seq, seq_id, 0, impeding_vehicle_ids,
                    &sub_sim_res, &sub_risky_res, &sub_sim_info,
                    &sub_progress_cost, &sub_tail_cost, &sub_cooperate_cost, &sub_forward_trajs,
                    &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                    &sub_surround_trajs);
  }
  else {
    // * prepare for multi-threading
    std::vector<std::thread> thread_set(n_sub_threads);
    int sub_thread_id = 0;

    for (const auto& leading_lon_a : leading_lon_actions) {
      for (const auto& following_lon_a : following_lon_actions) {
        sub_thread_info[sub_thread_id] = "";
        ForwardSimAgentSet new_surrounding_fsagents = surrounding_fsagents;
        // update nearest behavirol vehicle
        std::unordered_map<int, common::LongitudinalBehavior> lon_actions;
        if (leading_vehicle_id != kInvalidAgentId) {
          lon_actions[leading_vehicle_id] = leading_lon_a;
          sub_thread_info[sub_thread_id] += "leading CAV " + std::to_string(leading_vehicle_id) 
                                            + " longtitude: " + DcpTree::RetLonActionName(leading_lon_a) + ". ";
        }

        if (follow_vehicle_id != kInvalidAgentId) {
          lon_actions[follow_vehicle_id] = following_lon_a;
          sub_thread_info[sub_thread_id] += "following CAV " + std::to_string(follow_vehicle_id) 
                                            + " longtitude: " + DcpTree::RetLonActionName(following_lon_a) + ". ";
        }

        UpdateSurroundingSimSetupForScenario(lon_actions, &new_surrounding_fsagents);

        thread_set[sub_thread_id] =
          std::thread(&EudmPlanner::SimulateScenario, this, ego_vehicle, ego_fsagent, new_surrounding_fsagents, 
                      action_seq, seq_id, sub_thread_id, impeding_vehicle_ids,
                      &sub_sim_res, &sub_risky_res, &sub_sim_info,
                      &sub_progress_cost, &sub_tail_cost, &sub_cooperate_cost, &sub_forward_trajs,
                      &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                      &sub_surround_trajs);
        
        sub_thread_id+=1;
      }
    }
    for (int i = 0; i < n_sub_threads; ++i) {
      thread_set[i].join();
    }
  }
  
  // TODO for MLAD: 在这里选出最优的sub thread
  std::vector<int> valid_sub_sim_ids;
  for (int i = 0; i < n_sub_threads; ++i) {
    if (sub_sim_res[i] == 1) {
      valid_sub_sim_ids.push_back(i);
    }
  }

  if (valid_sub_sim_ids.empty()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  int sub_winner_id;
  EvaluateSubThreadSimResults(sub_progress_cost, sub_tail_cost, sub_cooperate_cost,
                              valid_sub_sim_ids, cfg_.cost().cooperation().sub_thread_coop_weight(),
                              &sub_winner_id);

  // ~ Here use the default scenario
  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res[sub_winner_id];
  sim_info_[seq_id] = sub_sim_info[sub_winner_id];
  cooperative_info_[seq_id] = sub_thread_info[sub_winner_id];
  progress_cost_[seq_id] = sub_progress_cost[sub_winner_id];
  cooperative_cost_[seq_id] = sub_cooperate_cost[sub_winner_id];
  tail_cost_[seq_id] = sub_tail_cost[sub_winner_id];
  forward_trajs_[seq_id] = sub_forward_trajs[sub_winner_id];
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors[sub_winner_id];
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors[sub_winner_id];
  surround_trajs_[seq_id] = sub_surround_trajs[sub_winner_id];

  return kSuccess;
}

ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // * no need to update
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * LLLLL -> LLKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * LLKKK -> LLRRR
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * LLRRR -> x
          return kWrongStatus;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * RRRRR -> RRKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * RRKKK -> RRLLL
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * RRLLL -> x
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      //LOG(ERROR) << "[Eudm]Error - Invalid lateral behavior";
      assert(false);
    }
  }

  return kSuccess;
}

bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  decimal_t current_lane_dist;
  decimal_t arc_len;
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);
  // //LOG(WARNING) << "[Eudm] current_lane_id: " << *current_lane_id;
  // //LOG(WARNING) << "[Eudm] action_ref_lane_id: " << action_ref_lane_id <<
  // std::endl;

  std::vector<int> potential_lane_ids;
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  // ~ Lane change
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

ErrorType EudmPlanner::RunOnce() {
  TicToc timer_runonce;
  // * Get current nearest lane id
  if (!map_itf_) {
    //LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    //LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;

  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp_
               << " ------- ";

  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    //LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }

  ego_lane_id_ = ego_lane_id_by_pos;

  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    //LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  LateralBehavior task_lat_behavior = LateralBehavior::kLaneKeeping;
  if (lc_info_.recommend_lc_left) {
    task_lat_behavior = LateralBehavior::kLaneChangeLeft;
  }
  else if (lc_info_.recommend_lc_right) {
    task_lat_behavior = LateralBehavior::kLaneChangeRight;
  }

  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(), task_lat_behavior,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &task_target_lane_) != kSuccess) {
    return kWrongStatus;
  }

  if (task_target_lane_.IsValid()) {
    task_target_stf_ = common::StateTransformer(task_target_lane_);
  }

  //LOG(ERROR) << std::fixed << std::setprecision(3)
              //  << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
              //  << ego_vehicle_.state().vec_position[0] << ","
              //  << ego_vehicle_.state().vec_position[1] << ","
              //  << ego_vehicle_.state().angle << ","
              //  << ego_vehicle_.state().velocity << ","
              //  << ego_vehicle_.state().acceleration << ","
              //  << ego_vehicle_.state().curvature << ")"
              //  << " lane id:" << ego_lane_id_by_pos;
  // LOG(ERROR) << "[Eudm][Setup]Desired vel:" << desired_velocity_
  //              << " sim_time total:" << sim_time_total_
  //              << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r,recomm_l,recomm_r]:"
  //              << lc_info_.forbid_lane_change_left << ","
  //              << lc_info_.forbid_lane_change_right << ","
  //              << lc_info_.lane_change_left_unsafe_by_occu << ","
  //              << lc_info_.lane_change_right_unsafe_by_occu << ","
  //              << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane << ","
  //              << lc_info_.recommend_lc_left << "," << lc_info_.recommend_lc_right;

  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
      }
      if (lc_info_.forbid_lane_change_left && 
          action_seq[j].lat == DcpLatAction::kLaneChangeLeft) {
        pre_deleted_seq_ids_.insert(i);
      }
      if (lc_info_.forbid_lane_change_right && 
          action_seq[j].lat == DcpLatAction::kLaneChangeRight) {
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  TicToc timer;
  if (RunEudm() != kSuccess) {
    //LOG(ERROR) << std::fixed << std::setprecision(4)
    //           << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp_
      //         << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  //LOG(ERROR) << line_info.str();

  time_cost_ = timer_runonce.toc();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSubThreadSimResults(
    const std::vector<std::vector<CostStructure>>& sub_progress_cost,
    const std::vector<CostStructure>& sub_tail_cost,
    const std::vector<std::vector<CostStructure>>& sub_cooperate_cost,
    const std::vector<int>& valid_sub_sim_ids,
    const decimal_t& coop_weight,
    int* sub_winner_id) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_costs = static_cast<int>(sub_progress_cost.front().size());
  for (const int& id:valid_sub_sim_ids) {
    decimal_t cost_tmp = 0.0;

    for (int i = 0; i < num_costs; i++) {
      cost_tmp += sub_progress_cost[id][i].ave();
      cost_tmp += sub_cooperate_cost[id][i].ave() * coop_weight;
    }

    cost_tmp += sub_tail_cost[id].ave();
    if (cost_tmp < min_cost) {
      min_cost = cost_tmp;
      best_id = id;
    }
  }
  *sub_winner_id = best_id;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const std::vector<CostStructure>& cooperative_cost,
    const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
    const decimal_t& coop_weight,
    decimal_t* score) {
  decimal_t score_tmp = 0.0;
  int num_costs = static_cast<int>(progress_cost.size());
  for (int i = 0; i < num_costs; i++) {
    score_tmp += progress_cost[i].ave();
    score_tmp += cooperative_cost[i].ave() * coop_weight;
  }
  *score = score_tmp + tail_cost.ave();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    decimal_t cost = 0.0;
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    EvaluateSinglePolicyTrajs(progress_cost_[i], cooperative_cost_[i], tail_cost_[i], action_seq,
                              cfg_.cost().cooperation().policy_coop_weight(), &cost);
    final_cost_[i] = cost;
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;
  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    common::RssChecker::RssCheck(traj_a[i], traj_b[i], rss_stf_, rss_config_,
                                 &is_rss_safe, &type, &rss_vel_low,
                                 &rss_vel_up);
    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          cost_tmp +=
              cfg_.cost().safety().rss_over_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          cost_tmp +=
              cfg_.cost().safety().rss_lack_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }
  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }
  // strict collision check
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      //LOG(WARNING) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);
      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }
  *is_safe = true;
  return kSuccess;
}

ErrorType EudmPlanner:: EgoCostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool verbose, CostStructure* cost, bool* is_risky,
    std::set<int>* risky_ids) {
  decimal_t duration = action.t;

  auto ego_lon_behavior_this_layer = ego_fsagent.lon_behavior;
  auto ego_lat_behavior_this_layer = ego_fsagent.lat_behavior;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  common::VehicleSet vehicle_set;
  for (const auto& v : other_fsagent.forward_sim_agents) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  decimal_t ego_velocity = ego_fsagent.vehicle.state().velocity;
  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  CostStructure cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_); 
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
  // > v_leading
  // unit of this cost is velocity (finally multiplied by duration)
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_fsagent.target_lane, ego_fsagent.vehicle.state(), vehicle_set,
          ego_fsagent.lat_range, &leading_vehicle,
          &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_fsagent.vehicle.state().vec_position)
            .norm();
    if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      decimal_t ego_blocked_by_leading_velocity =
          ego_fsagent.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_fsagent.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          std::max(cfg_.cost().effciency().min_distance_ratio(),
                   distance_residual_ratio) *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity); 
    } 
  }

  // * safety
  for (const auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // * navigation
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          cfg_.cost().user().cancel_operation_unit_cost();
    } 
    else {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          (seq_lat_behavior == LateralBehavior::kLaneChangeLeft
               ? cfg_.cost().navigation().lane_change_left_unit_cost()
               : cfg_.cost().navigation().lane_change_right_unit_cost());
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_left_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
        
      } 
      else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_right_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
        
      }
    }
  }
  
  if (!lc_info_.recommend_lc_left && !lc_info_.recommend_lc_right && 
      seq_lat_behavior != LateralBehavior::kLaneKeeping) {
    // 当任务层级，既不推荐左传，也不推荐右转时，语义动作为变道，需要增加cost
    cost_tmp.navigation.lane_change_preference += 
            std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
            ego_velocity) *
            cfg_.cost().user().late_operate_unit_cost();
  }
  // common::FrenetState ego_fs;
  // if (kSuccess != task_target_stf_.GetFrenetStateFromState(
  //                     ego_fsagent.vehicle.state(), &ego_fs)) {
  //   return kWrongStatus;
  // }
  // cost_tmp.navigation.lane_change_preference += std::fabs(ego_fs.vec_dt[0]) * 5; // TODO for MLAD: 後續寫入配置文件
  // printf("[CostFunction] abs: ego_fs.vec_dt[0]: %f\n",std::fabs(ego_fs.vec_dt[0]));
  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner:: CooperativeCostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    const std::vector<int>& impeding_vehicle_ids,
    CostStructure* coop_cost, bool* is_risky) {
  if (impeding_vehicle_ids.empty()) return kSuccess;

  decimal_t duration = action.t;

  ForwardSimAgent ego_sim_fsagent = ego_fsagent.GetForwardSimAgent();
  ForwardSimAgentSet all_fsagent = other_fsagent;
  all_fsagent.forward_sim_agents.insert(std::pair<int, ForwardSimAgent>(ego_sim_fsagent.id, ego_sim_fsagent));
  std::unordered_map<int, vec_E<common::Vehicle>> all_trajs = surround_trajs;
  all_trajs.insert(std::pair<int, vec_E<common::Vehicle>>(ego_sim_fsagent.id, ego_traj));

  CostStructure cost_tmp;

  for (const int& coop_id : impeding_vehicle_ids) {
    ForwardSimAgent coop_fsagent = all_fsagent.forward_sim_agents.at(coop_id);
    vec_E<common::Vehicle> coop_traj;

    common::VehicleSet vehicle_set;
    for (const auto& v : all_fsagent.forward_sim_agents) {
      if (v.first == coop_id) continue;

      vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
    }

    // efficiency
    // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
    // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
    // unit of this cost is velocity (finally multiplied by duration)
    decimal_t coop_velocity = coop_fsagent.vehicle.state().velocity;

    if (coop_velocity < desired_velocity_) {
      cost_tmp.efficiency.ego_to_desired_vel +=
          cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
          fabs(coop_velocity - desired_velocity_); 
    } 
    else {
      if (coop_velocity >
          desired_velocity_ +
              cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
        cost_tmp.efficiency.ego_to_desired_vel +=
            cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
            fabs(coop_velocity - desired_velocity_ -
                cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
      }
    }

    // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
    // > v_leading
    // unit of this cost is velocity (finally multiplied by duration)
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            coop_fsagent.lane, coop_fsagent.vehicle.state(), vehicle_set,
            coop_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      decimal_t distance_to_leading_vehicle =
          (leading_vehicle.state().vec_position -
          coop_fsagent.vehicle.state().vec_position).norm();

      if (coop_velocity < desired_velocity_ &&
          leading_vehicle.state().velocity < desired_velocity_ &&
          distance_to_leading_vehicle < cfg_.cost().effciency().leading_distance_th()) {
        decimal_t coop_blocked_by_leading_velocity =
            coop_velocity > leading_vehicle.state().velocity
                ? coop_velocity - leading_vehicle.state().velocity
                : 0.0;
        decimal_t leading_to_desired_velocity =
            leading_vehicle.state().velocity < desired_velocity_
                ? desired_velocity_ - leading_vehicle.state().velocity
                : 0.0;
        cost_tmp.efficiency.leading_to_desired_vel +=
            std::max(cfg_.cost().effciency().min_distance_ratio(), distance_residual_ratio) *
            (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
                coop_blocked_by_leading_velocity +
            cfg_.cost()
                    .effciency()
                    .leading_speed_blocked_desired_vel_unit_cost() *
                leading_to_desired_velocity); 
      } 
    }

    // * safety
    for (const auto& surround_traj : all_trajs) {
      if (surround_traj.first == coop_id) continue;

      decimal_t safety_cost = 0.0;
      bool is_safe = true;
      int risky_id = 0;
      EvaluateSafetyStatus(coop_traj, surround_traj.second, &safety_cost, &is_safe,
                          &risky_id);
      if (!is_safe) {
        *is_risky = true;
      }
      cost_tmp.safety.rss += safety_cost;
    }

  } 

  cost_tmp.weight = duration;
  *coop_cost = cost_tmp;

  return kSuccess;
}

ErrorType EudmPlanner::GetSimTimeSteps(const DcpAction& action,
                                       std::vector<decimal_t>* dt_steps) const {
  decimal_t sim_time_resolution = cfg_.sim().duration().step();
  decimal_t sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;

  return kSuccess;
}

ErrorType EudmPlanner::SimulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    const ForwardSimAgentSet& surrounding_fsagents_this_layer,
    vec_E<common::Vehicle>* ego_traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // ~ Prepare containers
  ego_traj->clear();
  surround_trajs->clear();
  for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  // ~ Simulation time steps
  std::vector<decimal_t> dt_steps;
  GetSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
  ForwardSimAgentSet surrounding_fsagents_this_step =
      surrounding_fsagents_this_layer;

  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];

    common::State ego_state_cache_this_step;
    std::unordered_map<int, State>
        others_state_cache_this_step;  // id - state_output

    common::VehicleSet all_sim_vehicles;  // include ego vehicle
    all_sim_vehicles.vehicles.insert(std::make_pair(
        ego_fsagent_this_step.vehicle.id(), ego_fsagent_this_step.vehicle));
    for (const auto& v : surrounding_fsagents_this_step.forward_sim_agents) {
      all_sim_vehicles.vehicles.insert(
          std::make_pair(v.first, v.second.vehicle));
    }

    // * For ego agent
    {
      all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);

      common::State state_output;
      if (kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,
                                         all_sim_vehicles, sim_time_step,
                                         &state_output)) {
        return kWrongStatus;
      }

      common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.set_state(state_output);
      ego_traj->push_back(v_tmp);
      ego_state_cache_this_step = state_output;

      all_sim_vehicles.vehicles.at(ego_id_).set_id(ego_id_);
    }

    // * For surrounding agents
    {
      for (const auto& p_fsa :
           surrounding_fsagents_this_step.forward_sim_agents) {
        common::State state_output;
        bool is_running = false;
        bool is_ego_prior = true;
        decimal_t next_time;

        if (cfg_.sim().explicit_cooperation_enable()
            && behavioral_key_vehicle_ids_.find(p_fsa.first) != behavioral_key_vehicle_ids_.end() 
            && map_itf_->IsEgoPriorityHigher(ego_id_, p_fsa.first, &is_ego_prior) == kSuccess) {
          if (!is_ego_prior) {
            next_time = p_fsa.second.vehicle.state().time_stamp + sim_time_step;
            // if (map_itf_->IsBehavioralVehicleRunningPlannedTraj(p_fsa.first, next_time, &is_running) != kSuccess) {
            //   printf("[Eudm] Get whether behavioral vehicle is running existing palnned trajectory failed. \n");
            // }
            is_running = true;
          }
        }
        // is_running = false;
        if (is_running && map_itf_->GetRunningBehavioralVehicleStateAtSpecTime(p_fsa.first, next_time, &state_output) == kSuccess) {
          // NOTE for MLAD: 在这里，对合作AGV的未来的轨迹，通过其已有的轨迹，计算出下一个时间的state
          // TODO for MLAD: 若当前时间为t，next time为t+10，获取t+10时刻状态时，因为障碍物而导致从t时刻开始的轨迹变化，之前9个时刻的预测全部错了，怎么处理？
          // printf("[Eudm] Get the state of behaviroal vehicle for next time from updated trajectory \n");
        }
        else {
          // NOTE for MLAD: 这里是对非合作AGV的未来轨迹进行预测。
          all_sim_vehicles.vehicles.at(p_fsa.first).set_id(kInvalidAgentId);
          
          if (kSuccess !=
              SurroundingAgentForwardSim(p_fsa.second, all_sim_vehicles,
                                        sim_time_step, &state_output)) {
            return kWrongStatus;
          }

          all_sim_vehicles.vehicles.at(p_fsa.first).set_id(p_fsa.first);
        }

        common::Vehicle v_tmp = p_fsa.second.vehicle;
        v_tmp.set_state(state_output); //Tips: state_output为使用IDM模拟出的每一步的状态
        surround_trajs->at(p_fsa.first).push_back(v_tmp);
        others_state_cache_this_step.insert(
            std::make_pair(p_fsa.first, state_output));
      }
    }

    // * update sim state after steps
    ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
    for (const auto& ps : others_state_cache_this_step) {
      surrounding_fsagents_this_step.forward_sim_agents.at(ps.first)
          .vehicle.set_state(ps.second);
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::SurroundingAgentForwardSim(
    const ForwardSimAgent& fsagent, const common::VehicleSet& all_sim_vehicles,
    const decimal_t& sim_time_step, common::State* state_out) const {
  common::Vehicle leading_vehicle;
  common::State state_output;
  decimal_t distance_residual_ratio = 0.0;
  map_itf_->GetLeadingVehicleOnLane(fsagent.lane, fsagent.vehicle.state(),
                                    all_sim_vehicles, fsagent.lat_range,
                                    &leading_vehicle, &distance_residual_ratio);
  if (planning::OnLaneForwardSimulation::PropagateOnce( 
          fsagent.stf, fsagent.vehicle, leading_vehicle, sim_time_step,
          fsagent.sim_param, &state_output) != kSuccess) {
    return kWrongStatus;
  }
  *state_out = state_output;
  return kSuccess;
}

ErrorType EudmPlanner::EgoAgentForwardSim(
    const ForwardSimEgoAgent& ego_fsagent,
    const common::VehicleSet& all_sim_vehicles, const decimal_t& sim_time_step,
    common::State* state_out) const {
  common::State state_output;
  // printf("[EgoAgentForwardSim] ego_fsagent.lat_behavior:%d\n",ego_fsagent.lat_behavior);
  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // * Lane keeping, only consider leading vehicle on ego lane
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.target_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          leading_vehicle.param(), leading_vehicle.state(), &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLK(
            ego_fsagent.target_stf, ego_fsagent.vehicle, leading_vehicle,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // * Lane changing, consider multiple vehicles
    common::Vehicle current_leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          current_leading_vehicle.param(), current_leading_vehicle.state(),
          &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    common::Vehicle gap_front_vehicle;
    if (ego_fsagent.target_gap_ids(0) != -1) {
      gap_front_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
    }
    common::Vehicle gap_rear_vehicle;
    if (ego_fsagent.target_gap_ids(1) != -1) {
      gap_rear_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;

    if (gap_rear_vehicle.id() != kInvalidAgentId &&
        cfg_.sim().ego().evasive().evasive_enable()) {
      common::FrenetState ego_on_tarlane_fs;
      common::FrenetState rear_on_tarlane_fs;
      if (ego_fsagent.target_stf.GetFrenetStateFromState(
              ego_fsagent.vehicle.state(), &ego_on_tarlane_fs) == kSuccess &&
          ego_fsagent.target_stf.GetFrenetStateFromState(
              gap_rear_vehicle.state(), &rear_on_tarlane_fs) == kSuccess) {
        // * rss check for evasive behavior
        bool is_rss_safe = true;
        common::RssChecker::LongitudinalViolateType type;
        decimal_t rss_vel_low, rss_vel_up;
        common::RssChecker::RssCheck(ego_fsagent.vehicle, gap_rear_vehicle,
                                     ego_fsagent.target_stf,
                                     rss_config_strict_as_rear_, &is_rss_safe,
                                     &type, &rss_vel_low, &rss_vel_up);
        if (!is_rss_safe) {
          if (type == common::RssChecker::LongitudinalViolateType::TooSlow) {
            sim_param.idm_param.kDesiredVelocity = std::max(
                sim_param.idm_param.kDesiredVelocity,
                rss_vel_low + cfg_.sim().ego().evasive().lon_extraspeed());
            sim_param.idm_param.kDesiredHeadwayTime =
                cfg_.sim().ego().evasive().head_time();
            sim_param.idm_param.kAcceleration =
                cfg_.sim().ego().evasive().lon_acc();
            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
          }
        }
        if (cfg_.sim().ego().evasive().virtual_barrier_enable()) {
          if (ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] <
              gap_rear_vehicle.param().length() +
                  cfg_.sim().ego().evasive().virtual_barrier_tic() *
                      gap_rear_vehicle.state().velocity) {
            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
          }
          common::FrenetState front_on_tarlane_fs;
          if (gap_front_vehicle.id() != kInvalidAgentId &&
              ego_fsagent.target_stf.GetFrenetStateFromState(
                  gap_front_vehicle.state(), &front_on_tarlane_fs) ==
                  kSuccess) {
            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] <
                ego_fsagent.vehicle.param().length() +
                    cfg_.sim().ego().evasive().virtual_barrier_tic() *
                        ego_fsagent.vehicle.state().velocity) {
              lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
            }
          }
        }
      }
    }
    
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(
            ego_fsagent.current_stf, ego_fsagent.target_stf,
            ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,
            gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  }

  *state_out = state_output;

  return kSuccess;
}

ErrorType EudmPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // ~ if routing information is available, here
    // ~ we still need to check whether the change is consist with the
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
  //                     &potential_lk_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
  //                     &potential_lcl_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_,
  // common::LateralBehavior::kLaneChangeRight,
  //                     &potential_lcr_lane_ids_);
  return kSuccess;
}

ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

int EudmPlanner::winner_id() const { return winner_id_; }

decimal_t EudmPlanner::time_cost() const { return time_cost_; }

void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning