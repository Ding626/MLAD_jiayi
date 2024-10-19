#include "semantic_map_manager/forward_planner.h"
#include <glog/logging.h>

namespace semantic_map_manager
{
  ForwardPlanner::ForwardPlanner() {
    // publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic_, 1);
  }

  std::string ForwardPlanner::Name()
  {
    return std::string("forward planner with keep lane behavior");
  }

  ErrorType ForwardPlanner::RunMpdm()
  {
    TicToc timer;
    LongitudinalBehavior mpdm_behavior;
    decimal_t mpdm_desired_velocity;
    if (MultiBehaviorJudge(behavior_.actual_desired_velocity, &mpdm_behavior,
                           &mpdm_desired_velocity) == kSuccess)
    {
      behavior_.forward_trajs.clear();
      behavior_.forward_behaviors.clear();
      behavior_.surround_trajs.clear();

      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      behavior_.actual_desired_velocity = mpdm_desired_velocity;

      if (winner_id_ != -1) {
        behavior_.forward_trajs.push_back(forward_trajs_[winner_id_]);
        behavior_.forward_behaviors.push_back(LateralBehavior::kLaneKeeping);
        behavior_.surround_trajs.push_back(surround_trajs_[winner_id_]);
      }
      
      LOG(ERROR) << "[MPDM]MPDM desired velocity " << behavior_.actual_desired_velocity 
                << " in " << reference_desired_velocity_;
      LOG(ERROR) << "[MPDM]Time multi behavior judged in " << timer.toc() << " ms.";
    }
    else
    {
      LOG(ERROR) << "[MPDM]MPDM failed.";
      LOG(ERROR) << "[MPDM]Time multi behavior judged in " << timer.toc() << " ms.";
      return kWrongStatus;
    }
    return kSuccess;
  }

  ErrorType ForwardPlanner::RunOnce(const common::Vehicle &ego_vehicle, const int &ego_lane_id_by_pos,
                                    const common::SemanticVehicleSet &semantic_vehicle_set, const int &aggressive_level,
                                    const bool &enable_openloop_prediction, common::SemanticLaneSet semantic_lane_set,
                                    const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs,
                                    common::SemanticBehavior *behavior)
  {
    // ~ The class which inherits this generic bp would probably need
    // ~ to implement the following logics.
    // ~ step 1: parse the navigation message if any
    // ~ step 2: decision making on the applicable behavior set
    // ~ step 3: construct complete semantic bahavior
    ego_vehicle_ = ego_vehicle;
    semantic_vehicle_set_ = semantic_vehicle_set;
    ego_id_ = ego_vehicle.id();
    ego_lane_id_ = ego_lane_id_by_pos;
    enable_openloop_prediction_ = enable_openloop_prediction;
    semantic_lane_set_ = semantic_lane_set;
    openloop_pred_trajs_ = openloop_pred_trajs;

    UpdateEgoLaneId(ego_lane_id_by_pos);
    LOG(INFO) << "[MPDM]ego lane id: " << ego_lane_id_;

    behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;

    behavior_.actual_desired_velocity = behavior->actual_desired_velocity;
    // reference_desired_velocity_ = behavior->actual_desired_velocity;
    // user_desired_velocity_ = behavior->actual_desired_velocity;
    ego_reflane_ = behavior->ref_lane;

    TicToc timer;
    planning::MultiModalForward::ParamLookUp(aggressive_level, &sim_param_);
    if (RunMpdm() != kSuccess)
    {
      LOG(ERROR) << "[Summary]Mpdm failed: " << timer.toc() << " ms.";
      // printf("[Stuck]Ego id %d on lane %d with behavior %d mpdm failed.\n",
      //        ego_vehicle.id(), ego_lane_id_,
      //        static_cast<int>(behavior_.lat_behavior));
      return kWrongStatus;
    }
    LOG(ERROR) << "[Summary]Mpdm time cost: " << timer.toc() << " ms.";
    std::ostringstream line_info;
    line_info << "[MPDM]selected forward traj: <stamp,x,y,theta,vec,acc,steer,curv>";
    decimal_t stamp = forward_trajs_[winner_id_].front().state().time_stamp;
    for (auto &v : forward_trajs_[winner_id_]) {
      line_info << std::fixed << std::setprecision(5) << "<"
                << v.state().time_stamp - stamp << ","  << v.state().vec_position[0]
                << "," << v.state().vec_position[1] << ","  << v.state().angle << "," 
                << v.state().velocity << "," << v.state().acceleration << "," 
                << v.state().steer << "," << v.state().curvature << ">";
    }
    LOG(WARNING) << line_info.str();

    behavior->lat_behavior = behavior_.lat_behavior;
    behavior->actual_desired_velocity = behavior_.actual_desired_velocity;
    behavior->forward_trajs = behavior_.forward_trajs;
    behavior->forward_behaviors = behavior_.forward_behaviors;
    behavior->surround_trajs = behavior_.surround_trajs;

    return kSuccess;
  }

  ErrorType ForwardPlanner::MultiBehaviorJudge(
      const decimal_t previous_desired_vel, LongitudinalBehavior *mpdm_behavior,
      decimal_t *mpdm_desired_velocity)
  {

    LOG(INFO) << "[ForwardPlanner]" << ego_vehicle_.id()
              << "\tsemantic_vehicle_set num:"
              << semantic_vehicle_set_.semantic_vehicles.size();

    // * clean the states
    forward_trajs_.clear();
    surround_trajs_.clear();
    winner_id_ = -1;

    // * collect potential behaviors
    std::vector<LongitudinalBehavior> potential_behaviors{
        common::LongitudinalBehavior::kAccelerate,
        common::LongitudinalBehavior::kMaintain,
        common::LongitudinalBehavior::kDecelerate};

    // * forward simulation
    std::vector<LongitudinalBehavior> valid_behaviors;
    vec_E<vec_E<common::Vehicle>> valid_forward_trajs;
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> valid_surround_trajs;
    int num_available_behaviors = static_cast<int>(potential_behaviors.size());

    // TicToc timer;
    for (int i = 0; i < num_available_behaviors; i++)
    {
      vec_E<common::Vehicle> traj;
      std::unordered_map<int, vec_E<common::Vehicle>> sur_trajs;
      if (SimulateEgoBehavior(ego_vehicle_, potential_behaviors[i],
                              semantic_vehicle_set_, &traj,
                              &sur_trajs) != kSuccess)
      {
        LOG(ERROR) <<"[MPDM]fail to sim " << DcpTree::RetLonActionName(potential_behaviors[i]) << " forward.\n";
        continue;
      }
      valid_behaviors.push_back(potential_behaviors[i]);
      valid_forward_trajs.push_back(traj);
      valid_surround_trajs.push_back(sur_trajs);
    }
    // printf("[Summary]Time in simulate all the behaviors: %lf ms.\n",
    // timer.toc());

    // * judge forward trajs
    int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
    if (num_valid_behaviors < 1)
    {
      LOG(ERROR) << "[MPDM]No valid behaviors";
      return kWrongStatus;
    }

    decimal_t winner_score, winner_desired_vel;
    if (EvaluateMultiPolicyTrajs(valid_behaviors, valid_forward_trajs,
                                 valid_surround_trajs, &winner_id_, &winner_score,
                                 &winner_desired_vel) != kSuccess)
    {
      LOG(ERROR) << "[MPDM]fail to evaluate multiple policy trajs.";
      return kWrongStatus;
    }
    
    LateralBehavior winner_behavior;
    vec_E<common::Vehicle> winner_forward_traj;

    // * output
    const decimal_t max_vel_cmd_gap = 5.0;
    if (fabs(winner_desired_vel - ego_vehicle_.state().velocity) >
        max_vel_cmd_gap)
    {
      if (winner_desired_vel > ego_vehicle_.state().velocity)
      {
        winner_desired_vel = ego_vehicle_.state().velocity + max_vel_cmd_gap;
      }
      else
      {
        winner_desired_vel = ego_vehicle_.state().velocity - max_vel_cmd_gap;
      }
    }

    *mpdm_behavior = valid_behaviors[winner_id_];
    *mpdm_desired_velocity = winner_desired_vel;

    // ! cache
    forward_trajs_ = valid_forward_trajs;
    surround_trajs_ = valid_surround_trajs;

    return kSuccess;
  }

  ErrorType ForwardPlanner::OpenloopSimForward(
      const common::SemanticVehicle &ego_semantic_vehicle,
      const common::SemanticVehicleSet &agent_vehicles,
      const planning::OnLaneForwardSimulation::Param &ego_sim_param,
      vec_E<common::Vehicle> *traj,
      std::unordered_map<int, vec_E<common::Vehicle>> *surround_trajs)
  {
    traj->clear();
    traj->push_back(ego_semantic_vehicle.vehicle);
    surround_trajs->clear();
    for (const auto v : agent_vehicles.semantic_vehicles)
    {
      surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
          v.first, vec_E<common::Vehicle>()));
      surround_trajs->at(v.first).push_back(v.second.vehicle);
    }

    int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);
    common::Vehicle cur_ego_vehicle = ego_semantic_vehicle.vehicle;
    common::SemanticVehicleSet semantic_vehicle_set_tmp = agent_vehicles;
    common::State ego_state;
    for (int i = 0; i < num_steps_forward; i++)
    {
      // sim_param_.idm_param.kDesiredVelocity = reference_desired_velocity_;
      if (planning::OnLaneForwardSimulation::PropagateOnce(
              common::StateTransformer(ego_semantic_vehicle.lane),
              cur_ego_vehicle, common::Vehicle(), sim_resolution_, ego_sim_param,
              &ego_state) != kSuccess)
      {
        return kWrongStatus;
      }
      std::unordered_map<int, State> state_cache;
      for (auto &v : semantic_vehicle_set_tmp.semantic_vehicles)
      {
        decimal_t desired_vel =
            agent_vehicles.semantic_vehicles.at(v.first).vehicle.state().velocity;
        sim_param_.idm_param.kDesiredVelocity = desired_vel;
        common::State agent_state;
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                common::StateTransformer(v.second.lane), v.second.vehicle,
                common::Vehicle(), sim_resolution_, sim_param_,
                &agent_state) != kSuccess)
        {
          return kWrongStatus;
        }
        state_cache.insert(std::make_pair(v.first, agent_state));
      }

      bool is_collision = false;
      CheckCollisionUsingStateAndVehicleParam(ego_semantic_vehicle.vehicle.param(), ego_state,
                                              &is_collision);
      if (is_collision)
        return kWrongStatus;

      // * update and trace
      cur_ego_vehicle.set_state(ego_state);
      for (auto &s : state_cache)
      {
        semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
            s.second);
        surround_trajs->at(s.first).push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      }
      traj->push_back(cur_ego_vehicle);
    }
    return kSuccess;
  }

  ErrorType ForwardPlanner::SimulateEgoBehavior(
      const common::Vehicle &ego_vehicle, const LongitudinalBehavior &ego_behavior,
      const common::SemanticVehicleSet &semantic_vehicle_set,
      vec_E<common::Vehicle> *traj,
      std::unordered_map<int, vec_E<common::Vehicle>> *surround_trajs)
  {
    planning::OnLaneForwardSimulation::Param ego_sim_param = sim_param_;

    switch (ego_behavior) {
      case LongitudinalBehavior::kAccelerate: {
        ego_sim_param.idm_param.kDesiredVelocity = std::min(
            ego_vehicle.state().velocity + acc_cmd_vel_gap_, reference_desired_velocity_);
        ego_sim_param.idm_param.kMinimumSpacing *=
            (1.0 - lon_aggressive_ratio_);
        ego_sim_param.idm_param.kDesiredHeadwayTime *=
            (1.0 - lon_aggressive_ratio_);
        break;
      }
      case LongitudinalBehavior::kDecelerate: {
        ego_sim_param.idm_param.kDesiredVelocity =
            std::min(std::max(ego_vehicle.state().velocity - dec_cmd_vel_gap_, 0.0),
                    reference_desired_velocity_);
        break;
      }
      case LongitudinalBehavior::kMaintain: {
        ego_sim_param.idm_param.kDesiredVelocity = std::min(ego_vehicle.state().velocity, reference_desired_velocity_);
        break;
      }
      default: {
        LOG(ERROR) << "[Eudm]Error - Lon action not valid";
        assert(false);
      }
    }

    common::SemanticVehicle ego_semantic_vehicle;
    {
      ego_semantic_vehicle.vehicle = ego_vehicle;
      ego_semantic_vehicle.lane = ego_reflane_;
    }

    common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;
    semantic_vehicle_set_tmp.semantic_vehicles.insert(
        std::make_pair(ego_vehicle.id(), ego_semantic_vehicle));

    // ~ multi-agent forward
    LOG(ERROR) << "[MPDM]simulating behavior " << DcpTree::RetLonActionName(ego_behavior);
    if (MultiAgentSimForward(ego_vehicle.id(), semantic_vehicle_set_tmp, ego_sim_param, traj,
                             surround_trajs) != kSuccess)
    {
      LOG(ERROR) << "[MPDM]multi agent forward under " << DcpTree::RetLonActionName(ego_behavior)
                  << " failed.";
      if (OpenloopSimForward(ego_semantic_vehicle, semantic_vehicle_set, ego_sim_param, traj,
                             surround_trajs) != kSuccess)
      {
        LOG(ERROR) << "[MPDM]open loop forward under " << DcpTree::RetLonActionName(ego_behavior)
                  << " failed.";
        return kWrongStatus;
      }
    }
    LOG(ERROR) << "[MPDM]behavior " << DcpTree::RetLonActionName(ego_behavior) 
              << " traj num of states: " << static_cast<int>(traj->size());
    return kSuccess;
  }

  ErrorType ForwardPlanner::EvaluateMultiPolicyTrajs(
      const std::vector<LongitudinalBehavior> &valid_behaviors,
      const vec_E<vec_E<common::Vehicle>> &valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> &
          valid_surround_trajs,
      int *winner_id, decimal_t *winner_score,
      decimal_t *desired_vel)
  {
    int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
    if (num_valid_behaviors < 1)
      return kWrongStatus;

    vec_E<common::Vehicle> traj;
    LongitudinalBehavior behavior = common::LongitudinalBehavior::kStopping;
    decimal_t min_score = kInf;
    decimal_t des_vel = 0.0;
    for (int i = 0; i < num_valid_behaviors; i++)
    {
      decimal_t score, vel;
      EvaluateSinglePolicyTraj(valid_behaviors[i], valid_forward_trajs[i],
                               valid_surround_trajs[i], &score, &vel);
      // printf("[Stuck]id: %d behavior %d with cost: %lf.\n", ego_id_,
      //        static_cast<int>(valid_behaviors[i]), score);

      if (score < min_score)
      {
        min_score = score;
        des_vel = vel;
        behavior = valid_behaviors[i];
        *winner_id = i;
      }
    }
    LOG(ERROR) << "[MPDM]choose behavior " << DcpTree::RetLonActionName(behavior) 
              << " with cost " << min_score;
    *winner_score = min_score;
    *desired_vel = des_vel;
    return kSuccess;
  }

  ErrorType ForwardPlanner::EvaluateSafetyCost(
      const vec_E<common::Vehicle> &traj_a, const vec_E<common::Vehicle> &traj_b,
      decimal_t *cost)
  {
    if (traj_a.size() != traj_b.size())
    {
      return kWrongStatus;
    }
    int num_states = static_cast<int>(traj_a.size());
    decimal_t cost_tmp = 0.0;
    for (int i = 0; i < num_states; i++)
    {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(traj_a[i], 1.0, 1.0,
                                                   &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(traj_b[i], 1.0, 1.0,
                                                   &inflated_b);
      bool is_collision = false;
      CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                               inflated_b.param(), inflated_b.state(),
                               &is_collision);
      if (is_collision)
      {
        cost_tmp +=
            0.01 * fabs(traj_a[i].state().velocity - traj_b[i].state().velocity) *
            0.5;
      }
    }
    *cost = cost_tmp;
    return kSuccess;
  }

  ErrorType ForwardPlanner::EvaluateSinglePolicyTraj(
      const LongitudinalBehavior &behavior, const vec_E<common::Vehicle> &forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>> &surround_traj,
      decimal_t *score, decimal_t *desired_vel)
  {
    // prepare
    std::unordered_map<int, vec_E<common::Vehicle>> tmp_surround_trajs;
    for (auto it = surround_traj.begin(); it != surround_traj.end(); ++it)
    {
      if (!it->second.empty() && it->second.size() == forward_traj.size())
      {
        tmp_surround_trajs.insert(std::make_pair(it->first, it->second));
      }
    }
    decimal_t all_cost{0.0};

    for (int i = 0; i < forward_traj.size(); i++) {
      // * efficiency
      common::Vehicle ego_vehicle_tmp = forward_traj[i];
      common::VehicleSet other_set_tmp;
      for (auto it = tmp_surround_trajs.begin(); it != tmp_surround_trajs.end(); ++it)
      {
        other_set_tmp.vehicles.insert(std::make_pair(it->first, it->second[i]));
      }

      decimal_t cost_efficiency_ego_to_desired_vel =
          fabs(ego_vehicle_tmp.state().velocity -
              reference_desired_velocity_) /
          10.0;
      common::Vehicle leading_vehicle;

      decimal_t cost_efficiency_leading_to_desired_vel = 0.0;
      decimal_t distance_residual_ratio = 0.0;
      const decimal_t lat_range = 2.2;
      if (GetLeadingVehicleOnLane(
              ego_reflane_, ego_vehicle_tmp.state(), other_set_tmp, lat_range,
              &leading_vehicle, &distance_residual_ratio) == kSuccess)
      {
        decimal_t distance_to_leading_vehicle =
            (leading_vehicle.state().vec_position -
            ego_vehicle_tmp.state().vec_position)
                .norm();

        if (ego_vehicle_tmp.state().velocity < reference_desired_velocity_ &&
            leading_vehicle.state().velocity < reference_desired_velocity_ &&
            distance_to_leading_vehicle < 10.0)
        {
          cost_efficiency_leading_to_desired_vel =
              0.5 * distance_residual_ratio *
              fabs(ego_vehicle_tmp.state().velocity -
                  reference_desired_velocity_) /
              std::max(2.0, distance_to_leading_vehicle);
        }
      }
      decimal_t cost_efficiency = 0.3 * (cost_efficiency_ego_to_desired_vel +
                                        cost_efficiency_leading_to_desired_vel);

      // * safety
      decimal_t cost_safety = 0.0;
      common::Vehicle inflated_a;
      common::SemanticsUtils::InflateVehicleBySize(ego_vehicle_tmp, 1.0, 1.0,
                                                   &inflated_a);
      for (const auto &v : other_set_tmp.vehicles) {
        common::Vehicle inflated_b;
        common::SemanticsUtils::InflateVehicleBySize(v.second, 1.0, 1.0,
                                                    &inflated_b);
        bool is_collision = false;
        CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                inflated_b.param(), inflated_b.state(),
                                &is_collision);
        if (is_collision)
        {
          cost_safety +=
              0.01 * fabs(ego_vehicle_tmp.state().velocity - v.second.state().velocity) *
              0.5;
        }

        // * cooperation
        // TODO for MLAD: 这里是否需要加合作cost，即计算CAV的安全得分？

        // TODO for MLAD: 需要加discount吗？eudm里面没加
        all_cost += (cost_efficiency + cost_safety);
      }
      
    }

    // // * action
    // decimal_t cost_action = 0.0;
    // if (behavior != LongitudinalBehavior::kMaintain)
    // {
    //   cost_action += 0.5;
    // }
    
    // *score = cost_action + cost_safety + cost_efficiency;
    *score = all_cost;
    // printf(
    //     "[CostDebug]behaivor %d: (action %lf, safety %lf, efficiency ego %lf, "
    //     "leading %lf).\n",
    //     static_cast<int>(behavior), cost_action, cost_safety,
    //     cost_efficiency_ego_to_desired_vel,
    //     cost_efficiency_leading_to_desired_vel);
    LOG(ERROR) << 
        "[CostDebug]behaivor " << DcpTree::RetLonActionName(behavior) << ": cost " << all_cost;
    GetDesiredVelocityOfTrajectory(forward_traj, desired_vel);
    return kSuccess;
  }

  ErrorType ForwardPlanner::GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t *vel)
  {
    decimal_t min_vel = kInf;
    decimal_t max_acc_normal = 0.0;
    for (auto &v : vehicle_vec)
    {
      auto state = v.state();
      auto acc_normal = fabs(state.curvature) * pow(state.velocity, 2);
      min_vel = acc_normal > max_acc_normal ? state.velocity : min_vel;
    }
    *vel = min_vel;
    return kSuccess;
  }

  ErrorType ForwardPlanner::MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet &semantic_vehicle_set,
      const planning::OnLaneForwardSimulation::Param &ego_sim_param,
      vec_E<common::Vehicle> *traj,
      std::unordered_map<int, vec_E<common::Vehicle>> *surround_trajs)
  {
    traj->clear();
    traj->push_back(semantic_vehicle_set.semantic_vehicles.at(ego_id).vehicle);

    surround_trajs->clear();
    for (const auto v : semantic_vehicle_set.semantic_vehicles)
    {
      if (v.first == ego_id)
        continue;
      surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
          v.first, vec_E<common::Vehicle>()));
      surround_trajs->at(v.first).push_back(v.second.vehicle);
    }

    int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);

    common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;

    TicToc timer;
    for (int i = 0; i < num_steps_forward; i++)
    {
      timer.tic();
      std::unordered_map<int, State> state_cache;
      for (auto &v : semantic_vehicle_set_tmp.semantic_vehicles)
      {
        decimal_t desired_vel = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                    .vehicle.state()
                                    .velocity;
        decimal_t init_stamp = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                   .vehicle.state()
                                   .time_stamp;
        
        common::VehicleSet vehicle_set;
        for (auto &v_other : semantic_vehicle_set_tmp.semantic_vehicles)
        {
          // ~ get the subset of vehicles excluding the simulating one
          if (v_other.first != v.first)
            vehicle_set.vehicles.insert(
                std::make_pair(v_other.first, v_other.second.vehicle));
        }

        // decimal_t speed_limit;
        // if (traffic_singal_manager_.GetSpeedLimit(v.second.vehicle.state(), v.second.lane,
        //                                           &speed_limit) == kSuccess)
        // {
        //   desired_vel = std::min(speed_limit * 0.9, desired_vel);
        // }
        
        common::Vehicle leading_vehicle;
        common::State state;
        decimal_t distance_residual_ratio = 0.0;
        const decimal_t lat_range = 2.2;
        if (GetLeadingVehicleOnLane(
                v.second.lane, v.second.vehicle.state(), vehicle_set, lat_range,
                &leading_vehicle, &distance_residual_ratio) == kSuccess)
        {
          bool is_collision = false;
          CheckCollisionUsingState(
              v.second.vehicle.param(), v.second.vehicle.state(),
              leading_vehicle.param(), leading_vehicle.state(), &is_collision);
          if (is_collision)
          {
            return kWrongStatus;
          }
        }

        if (v.first == ego_id) {
          if (planning::OnLaneForwardSimulation::PropagateOnce(
                  common::StateTransformer(v.second.lane), v.second.vehicle,
                  leading_vehicle, sim_resolution_, ego_sim_param,
                  &state) != kSuccess)
          {
            LOG(ERROR) << "[MPDM]fail to forward with leading vehicle.";
            return kWrongStatus;
          }
        }
        else {
          sim_param_.idm_param.kDesiredVelocity = desired_vel;
          if (planning::OnLaneForwardSimulation::PropagateOnce(
                  common::StateTransformer(v.second.lane), v.second.vehicle,
                  leading_vehicle, sim_resolution_, sim_param_,
                  &state) != kSuccess)
          {
            LOG(ERROR) << "[MPDM]fail to forward with leading vehicle.";
            return kWrongStatus;
          }
        }

        // update state
        state.time_stamp = init_stamp + (i + 1) * sim_resolution_;
        state_cache.insert(std::make_pair(v.first, state));
      }
      // printf("[Summary]single propogate once time: %lf.\n", timer.toc());

      // use state cache to update vehicle set
      for (auto &s : state_cache)
      {
        semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
            s.second);
        if (s.first == ego_id)
        {
          traj->push_back(
              semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
        }
        else
        {
          surround_trajs->at(s.first).push_back(
              semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
        }
      }
    } // end num_steps_forward for
    return kSuccess;
  }

  ErrorType ForwardPlanner::UpdateEgoLaneId(const int new_ego_lane_id)
  {
    ego_lane_id_ = new_ego_lane_id;
    GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
                        &potential_lk_lane_ids_);
    return kSuccess;
  }

  ErrorType ForwardPlanner::GetPotentialLaneIds(
      const int source_lane_id, const LateralBehavior &beh,
      std::vector<int> *candidate_lane_ids)
  {
    candidate_lane_ids->clear();
    if (beh == common::LateralBehavior::kUndefined ||
        beh == common::LateralBehavior::kLaneKeeping)
    {
      auto it = semantic_lane_set_.semantic_lanes.find(source_lane_id);
      if (it == semantic_lane_set_.semantic_lanes.end())
      {
        return kWrongStatus;
      }
      else
      {
        // ~ note this is an assign
        candidate_lane_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
      }
    }
    else
    {
      assert(false);
    }
    return kSuccess;
  }

  ErrorType ForwardPlanner::GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio)
  {
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
    if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess)
    {
      return kWrongStatus;
    }
    ref_lane.GetPositionByArcLength(ref_fs.vec_s[0], &lane_pt);
    // Vecf<2> offset = ref_state.vec_position - lane_pt;

    const decimal_t lane_width = 3.5;
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
         s += resolution)
    {
      decimal_t delta_s = s - ref_fs.vec_s[0];
      ref_lane.GetPositionByArcLength(s, &lane_pt);
      // lane_pt = lane_pt + offset;

      for (const auto &entry : vehicle_set.vehicles)
      {
        if (entry.second.id() == kInvalidAgentId)
          continue;
        if ((lane_pt - entry.second.state().vec_position).squaredNorm() <
            search_lat_radius * search_lat_radius)
        {
          find_leading_vehicle_in_set = true;
          leading_vehicle_id = entry.first;
          *distance_residual_ratio =
              (max_forward_search_dist - delta_s) / max_forward_search_dist;
          break;
        }
      }

      if (find_leading_vehicle_in_set)
        break;
    }

    if (find_leading_vehicle_in_set)
    {
      auto it = vehicle_set.vehicles.find(leading_vehicle_id);
      *leading_vehicle = it->second;
    }
    else
    {
      return kWrongStatus;
    }
    return kSuccess;
  }

  ErrorType ForwardPlanner::CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res)
  {
    common::OrientedBoundingBox2D obb_a, obb_b;
    common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
        param_a, state_a, &obb_a);
    common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
        param_b, state_b, &obb_b);
    *res = common::ShapeUtils::CheckIfOrientedBoundingBoxIntersect(obb_a, obb_b);
    return kSuccess;
  }

  ErrorType ForwardPlanner::CheckCollisionUsingStateAndVehicleParam(
      const common::VehicleParam &vehicle_param, const common::State &state,
      bool *res)
  {
    // check static collision
    {
      // TODO: (@denny.ding) add static collision checking
      common::Vehicle vehicle;
      vehicle.set_state(state);
      vehicle.set_param(vehicle_param);
      vec_E<Vec2f> vertices;
      common::ShapeUtils::GetVerticesOfOrientedBoundingBox(
          vehicle.RetOrientedBoundingBox(), &vertices);
      bool is_collision = false;
      for (auto &v : vertices)
      {
        CheckCollisionUsingGlobalPosition(v, &is_collision);
        if (is_collision)
        {
          *res = is_collision;
          return kSuccess;
        }
      }
    }

    // check dynamic collision
    if (enable_openloop_prediction_)
    {
      for (const auto &v : semantic_vehicle_set_.semantic_vehicles)
      {
        auto state_stamp = state.time_stamp;
        auto obstacle_init_stamp = v.second.vehicle.state().time_stamp;
        auto openloop_pred_traj = openloop_pred_trajs_.at(v.first);
        int access_index =
            std::round((state_stamp - obstacle_init_stamp) / pred_step_);
        int num_pred_states = static_cast<int>(openloop_pred_traj.size());
        if (access_index < 0 || access_index >= num_pred_states)
          continue;
        bool is_collision = false;
        CheckCollisionUsingState(vehicle_param, state, v.second.vehicle.param(),
                                 openloop_pred_traj[access_index], &is_collision);
        if (is_collision)
        {
          *res = is_collision;
          return kSuccess;
        }
      }
    }
    *res = false;
    return kSuccess;
  }

  ErrorType ForwardPlanner::CheckCollisionUsingGlobalPosition(
      const Vec2f &p_w, bool *res) const
  {
    std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
    return obstacle_map_.CheckIfEqualUsingGlobalPosition(p, GridMap2D::OCCUPIED,
                                                         res);
  }

  // void ForwardPlanner::VisualizeForwardTrajectories() {
  //   visualization_msgs::MarkerArray traj_list_marker;
  //   common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);
  //   double traj_z = 0.3;
  //   for (int i = 0; i < static_cast<int>(forward_trajs_.size()); ++i) {
  //     if (i == winner_id_) {
  //       traj_color = common::cmap.at("spring green");
  //       traj_z = 0.4;
  //     }
  //     else {
  //       traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);
  //       traj_z = 0.3;
  //     }
  //     std::vector<common::Point> points;
  //     for (const auto& v : forward_trajs_[i]) {
  //       common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
  //       // pt.z = v.state().time_stamp -
  //       // forward_trajs[i].front().state().time_stamp;
  //       pt.z = traj_z;
  //       points.push_back(pt);
  //       visualization_msgs::Marker point_marker;
  //       // point_marker.ns = "point";
  //       common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
  //           common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
  //           &point_marker);
  //       traj_list_marker.markers.push_back(point_marker);
  //     }
  //     visualization_msgs::Marker line_marker;
  //     // line_marker.ns = "line";
  //     common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
  //         points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
  //     traj_list_marker.markers.push_back(line_marker);
  //   }
  //   int num_markers = static_cast<int>(traj_list_marker.markers.size());
  //   common::VisualizationUtil::FillHeaderIdInMarkerArray(
  //       ros::Time::now(), std::string("map"), last_forward_trajs_marker_cnt_,
  //       &traj_list_marker);
  //   last_forward_trajs_marker_cnt_ = num_markers;

  //   publisher_.publish(traj_list_marker);
  // }
} // namespace planning
