#include "eudm_planner/eudm_manager.h"

#include <glog/logging.h>

namespace planning {

void EudmManager::Init(const std::string& config_path,
                       const decimal_t work_rate,
                       semantic_map_manager::SemanticMapManager *smm) {
  google::InitGoogleLogging("eudm");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::SetLogDestination(google::GLOG_INFO, "/home/pc/MLAD/src/Log/eudm/info_");
  google::InstallFailureSignalHandler();                             //!< Capture SIGSEGV info and out to stderr
	google::SetLogFilenameExtension(".log");  //在日志文件名中级别后添加一个扩展名。适用于所有严重级别
  FLAGS_logbufsecs = 0;  //设置实时输出日志，实时刷新(必须设置为0)
  FLAGS_max_log_size = 1024;  //max_log_size：1024MB 设置日志记录文件最大大小，MB为单位，默认值为1800，当前超过当前大小，则保存剩余数据至文件，并创建新的文件保存其他日志信息；
  smm_ = smm;
  map_adapter_.set_smm(smm);
  bp_.Init(config_path);
  bp_.set_map_interface(&map_adapter_);
  work_rate_ = work_rate;
  if (bp_.cfg().function().active_lc_enable()) {
    //LOG(ERROR) << "[HMI]HMI enabled with active lane change ON.";
  } else {
    //LOG(ERROR) << "[HMI]HMI enabled with active lane change OFF.";
  }
}

decimal_t EudmManager::GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                                     const decimal_t& delta) {
  // consider the nearest decision point (rounded by layer time)
  // w.r.t stamp + delta
  decimal_t past_decision_point =
      std::floor((stamp + delta) / bp_.cfg().sim().duration().layer()) *
      bp_.cfg().sim().duration().layer();
  return past_decision_point + bp_.cfg().sim().duration().layer();
}

bool EudmManager::IsTriggerAppropriate(const LateralBehavior& lat) {
#if 1
  return true;
#endif
  // check whether appropriate to conduct lat behavior right now,
  // if not, recommend a velocity instead.
  if (!last_snapshot_.valid) return false;
  // KXXXX
  // KKXXX
  // KKKLL
  // KKKKL
  const int kMinActionCheckIdx = 1;
  const int kMaxActionCheckIdx = 3;
  const int kMinMatchSeqs = 3;
  int num_action_seqs = last_snapshot_.action_script.size();
  int num_match_seqs = 0;
  for (int i = 0; i < num_action_seqs; i++) {
    if (!last_snapshot_.sim_res[i] || last_snapshot_.risky_res[i]) continue;
    auto action_seq = last_snapshot_.action_script[i];
    int num_actions = action_seq.size();
    for (int j = kMinActionCheckIdx; j <= kMaxActionCheckIdx; j++) {
      if (lat == LateralBehavior::kLaneChangeLeft &&
          action_seq[j].lat == DcpLatAction::kLaneChangeLeft &&
          (action_seq[j].lon == DcpLonAction::kAccelerate ||
           action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      } else if (lat == LateralBehavior::kLaneChangeRight &&
                 action_seq[j].lat == DcpLatAction::kLaneChangeRight &&
                 (action_seq[j].lon == DcpLonAction::kAccelerate ||
                  action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      }
    }
  }
  if (num_match_seqs < kMinMatchSeqs) return false;
  return true;
}

ErrorType EudmManager::GetCurEgoVehicleLastConfigs(const int &cur_ego_id) {
  map_adapter_.set_map(cur_ego_id);
  Reset();
  replane_task_ = false;

  if (map_adapter_.GetLastReplanningContextForCurEgoVehicle(cur_ego_id, &context_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last ReplanningContext for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  if (map_adapter_.GetLastTaskForCurEgoVehicle(cur_ego_id, &last_task_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last task for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  if (map_adapter_.GetLastSnapshotForCurEgoVehicle(cur_ego_id, &last_snapshot_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last Snapshot for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  if (map_adapter_.GetLastLaneChangeProposalForCurEgoVehicle(cur_ego_id, &last_lc_proposal_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last LaneChangeProposal for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  if (map_adapter_.GetLastPreliminaryActiveRequestsForCurEgoVehicle(cur_ego_id, &preliminary_active_requests_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last PreliminaryActiveRequests for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  if (map_adapter_.GetLastLaneChangeContextForCurEgoVehicle(cur_ego_id, &lc_context_) != kSuccess) {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager] Get last LaneChangeContext for vehicle "
              << cur_ego_id
              << " failed.";
    //LOG(WARNING) << line_info.str();
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType EudmManager::Prepare(
    const decimal_t stamp,
    const Task& task) {
  
  DcpAction desired_action;
  if (!GetReplanDesiredAction(stamp, &desired_action)) {
    desired_action.lat = DcpLatAction::kLaneKeeping;
    desired_action.lon = DcpLonAction::kMaintain;
    decimal_t fdp_stamp = GetNearestFutureDecisionPoint(stamp, 0.0);
    desired_action.t = fdp_stamp - stamp;
  }
  //LOG(ERROR) << "[Eudm][Manager] desired_action.lat: " << DcpTree::RetLatActionName(desired_action.lat);

  if (map_adapter_.map().GetEgoNearestLaneId(&ego_lane_id_) != kSuccess) {
    return kWrongStatus;
  }
  //LOG(ERROR) << "[Eudm][Manager] dGetEgoNearestLaneId " << ego_lane_id_;
  UpdateLaneChangeContextByTask(stamp, task);
  //LOG(ERROR) << "[Eudm][Manager] lc_context_.completed " << lc_context_.completed;
  if (lc_context_.completed) {
    desired_action.lat = DcpLatAction::kLaneKeeping;
  }

  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]Replan context <valid, stamp, seq>:<"
              << context_.is_valid << "," << stamp << "," << std::fixed << std::setprecision(3)
              << context_.seq_start_time << ",";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << ">";
    //LOG(WARNING) << line_info.str();
  }
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]LC context <completed, twa, tt, dt, l_id, "
                 "lat, type>:<"
              << lc_context_.completed << ","
              << lc_context_.trigger_when_appropriate << "," << std::fixed
              << std::setprecision(3) << lc_context_.trigger_time << ","
              << lc_context_.desired_operation_time << ","
              << lc_context_.ego_lane_id << ","
              << common::SemanticsUtils::RetLatBehaviorName(lc_context_.lat)
              << "," << static_cast<int>(lc_context_.type) << ">";
    //LOG(WARNING) << line_info.str();
  }

  bp_.UpdateDcpTree(desired_action);
  
  decimal_t ref_vel;
  EvaluateReferenceVelocity(task, &ref_vel);
  //LOG(WARNING) << "[Eudm][Manager]<task vel, ref_vel>:<"
             //  << task.user_desired_vel << "," << ref_vel << ">";
  bp_.set_desired_velocity(ref_vel);

  auto lc_info = task.lc_info;
  // augment lc info with lane context information
  if (!lc_context_.completed) {
    // TODO for MLAD: 若時間沒到desired_operation_time，仍然需要recommend_lc_left爲true
    // if (stamp >= lc_context_.desired_operation_time) {
    //   if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
    //     //LOG(WARNING) << std::fixed << std::setprecision(5)
    //                  << "[HMI]Recommending left at " << stamp
    //                  << " with desired time: "
    //                  << lc_context_.desired_operation_time;
    //     lc_info.recommend_lc_left = true;
    //   } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
    //     lc_info.recommend_lc_right = true;
    //   }
    // }
    if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
      //LOG(WARNING) << std::fixed << std::setprecision(5)
                    // << "[HMI]Recommending left at " << stamp
                    // << " with desired time: "
                    // << lc_context_.desired_operation_time;
      lc_info.recommend_lc_left = true;
    } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
      lc_info.recommend_lc_right = true;
    }
  }
  bp_.set_lane_change_info(lc_info);

  //LOG(WARNING) << "[Eudm][Manager]desired <lon,lat,t>:<"
              //  << DcpTree::RetLonActionName(desired_action.lon).c_str() << ","
              //  << DcpTree::RetLatActionName(desired_action.lat) << ","
              //  << desired_action.t << "> ego lane id:" << ego_lane_id_;

  return kSuccess;
}

ErrorType EudmManager::GenerateLaneChangeProposal(
    const decimal_t& stamp, const Task& task) {
  if (!bp_.cfg().function().active_lc_enable()) {
    preliminary_active_requests_.clear();
    //LOG(ERROR) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to disabled lc:"
                //  << stamp;
    return kSuccess;
  }

  if (!task.is_under_ctrl) {
    preliminary_active_requests_.clear();
    //LOG(ERROR) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to not under ctrl:"
                //  << stamp;
    return kSuccess;
  }

  if (!lc_context_.completed) {
    preliminary_active_requests_.clear();
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to not completed lc:"
                //  << stamp;
    return kSuccess;
  }

  // if stick not reset, will not try active lane change
  if (lc_context_.completed && task.user_perferred_behavior != 0) {
    preliminary_active_requests_.clear();
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to stick not rest:"
                //  << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time < 0.0) {
    last_lc_proposal_.valid = false;
    last_lc_proposal_.trigger_time = stamp;
    last_lc_proposal_.ego_lane_id = ego_lane_id_;
    last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
    preliminary_active_requests_.clear();
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to illegal stamp:"
                //  << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time <
      bp_.cfg().function().active_lc().cold_duration()) {
    preliminary_active_requests_.clear();
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to cold down:" << stamp
                //  << " < " << last_lc_proposal_.trigger_time << " + "
                //  << bp_.cfg().function().active_lc().cold_duration();
    return kSuccess;
  }

  if (last_snapshot_.plan_state.velocity <
          bp_.cfg().function().active_lc().activate_speed_lower_bound() ||
      last_snapshot_.plan_state.velocity >
          bp_.cfg().function().active_lc().activate_speed_upper_bound()) {
    preliminary_active_requests_.clear();
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to illegal spd:"
                //  << last_snapshot_.plan_state.velocity << " at time " << stamp;
    return kSuccess;
  }

  if (bp_.cfg()
          .function()
          .active_lc()
          .enable_clear_accumulation_by_forbid_signal() &&
      not preliminary_active_requests_.empty()) {
    auto last_request = preliminary_active_requests_.back();
    if (last_request.lat == LateralBehavior::kLaneChangeLeft &&
        task.lc_info.forbid_lane_change_left) {
      //LOG(WARNING) << std::fixed << std::setprecision(5)
                  //  << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                  //  << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat == LateralBehavior::kLaneChangeRight &&
        task.lc_info.forbid_lane_change_right) {
      //LOG(WARNING) << std::fixed << std::setprecision(5)
                  //  << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                  //  << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
  }

  common::LateralBehavior lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  bp_.ClassifyActionSeq(
      last_snapshot_.action_script[last_snapshot_.original_winner_id],
      &operation_at_seconds, &lat_behavior, &is_cancel_behavior);
  if (lat_behavior == LateralBehavior::kLaneKeeping || is_cancel_behavior) {
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]Clear request due to not ideal behavior:"
                //  << stamp;
    preliminary_active_requests_.clear();
    return kSuccess;
  }

  ActivateLaneChangeRequest this_request;
  this_request.trigger_time = stamp;
  this_request.desired_operation_time = stamp + operation_at_seconds;
  this_request.ego_lane_id = ego_lane_id_;
  this_request.lat = lat_behavior;
  if (preliminary_active_requests_.empty()) {
    preliminary_active_requests_.push_back(this_request);
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                //  << " Init requesting "
                //  << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                //  << " at " << this_request.desired_operation_time
                //  << " with lane id " << this_request.ego_lane_id;
  } else {
    //LOG(WARNING) << std::fixed << std::setprecision(5)
                //  << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                //  << " Consequent requesting "
                //  << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                //  << " at " << this_request.desired_operation_time
                //  << " with lane id " << this_request.ego_lane_id;
    auto last_request = preliminary_active_requests_.back();
    if (last_request.ego_lane_id != this_request.ego_lane_id) {
      //LOG(WARNING)
          // << "[Eudm][ActiveLc]Invalid this request due to lane id inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat != this_request.lat) {
      //LOG(WARNING) << "[Eudm][ActiveLc]Invalid this request due to behavior "
                      // "inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (fabs(last_request.desired_operation_time -
             this_request.desired_operation_time) >
        bp_.cfg().function().active_lc().consistent_operate_time_min_gap()) {
      //LOG(WARNING)
          // << "[Eudm][ActiveLc]Invalid this request due to time inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    preliminary_active_requests_.push_back(this_request);
    //LOG(WARNING) << "[Eudm][ActiveLc]valid this request. Queue size "
                //  << preliminary_active_requests_.size() << " and operate at "
                //  << operation_at_seconds;
  }

  if (preliminary_active_requests_.size() >=
      bp_.cfg().function().active_lc().consistent_min_num_frame()) {
    if (operation_at_seconds <
        bp_.cfg().function().active_lc().activate_max_duration_in_seconds() +
            kEPS) {
      last_lc_proposal_.valid = true;
      last_lc_proposal_.trigger_time = stamp;
      last_lc_proposal_.operation_at_seconds =
          operation_at_seconds > bp_.cfg()
                                     .function()
                                     .active_lc()
                                     .active_min_operation_in_seconds()
              ? operation_at_seconds
              : GetNearestFutureDecisionPoint(
                    stamp, bp_.cfg()
                               .function()
                               .active_lc()
                               .active_min_operation_in_seconds()) -
                    stamp;
      last_lc_proposal_.ego_lane_id = ego_lane_id_;
      last_lc_proposal_.lat = lat_behavior;
      preliminary_active_requests_.clear();
      //LOG(WARNING) << std::fixed << std::setprecision(5)
                  //  << "[HMI]Gen proposal with trigger time "
                  //  << last_lc_proposal_.trigger_time << " lane id "
                  //  << last_lc_proposal_.ego_lane_id << " behavior "
                  //  << static_cast<int>(last_lc_proposal_.lat) << " operate at "
                  //  << last_lc_proposal_.operation_at_seconds;
    } else {
      preliminary_active_requests_.clear();
      // //LOG(ERROR) << "[HMI]Abandan Queue due to change time not legal.";
    }
  }

  return kSuccess;
}

void EudmManager::UpdateLaneChangeContextByTask(const decimal_t stamp, const Task& task) {
  // TODO for MLAD: 這裏需要調整，不能直接取消任務
  // last_task_ = task;
  //LOG(WARNING) << "UpdateLaneChangeContextByTask:lc_context_.completed" << lc_context_.completed;
  if (!last_task_.is_under_ctrl && task.is_under_ctrl) {
    //LOG(WARNING) << "[HMI]Autonomous mode activated!";
    lc_context_.completed = false;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (last_task_.is_under_ctrl && !task.is_under_ctrl) {
    //LOG(ERROR) << "[HMI]Autonomous mode deactivated!";
    lc_context_.completed = false;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (map_adapter_.map().GetEgoNearestLaneId(&ego_lane_id_) != kSuccess) {
    return;
  }
  //LOG(ERROR) << "task.target_lane_id." << task.target_lane_id;
  //LOG(ERROR) << "map_adapter_.map().GetEgoNearestLaneId(&ego_lane_id_))" << ego_lane_id_;  
  if (task.target_lane_id != ego_lane_id_) {

    lc_context_.completed = true;
    // lc_context_.lat =  LateralBehavior::kLaneChangeRight;
    lc_context_.trigger_when_appropriate = true;
  }
  // else if(task.target_lane_id > map_adapter_.map().GetEgoNearestLaneId(&ego_lane_id_)) {
  //   lc_context_.completed = false;
  //   // lc_context_.lat =  LateralBehavior::kLaneChangeLeft;
  //   lc_context_.trigger_when_appropriate = true;
  // }
  else{
        //LOG(WARNING) << "[HMI]already on target lane "
                //  << task.target_lane_id  << " to "
                //  << task.user_perferred_behavior;
    lc_context_.completed = false;
    lc_context_.lat =  LateralBehavior::kLaneKeeping;
    lc_context_.trigger_when_appropriate = false;
  }

  if (task.user_perferred_behavior != last_task_.user_perferred_behavior) {
    //LOG(WARNING) << "[HMI]stick state change from "
                //  << last_task_.user_perferred_behavior << " to "
                //  << task.user_perferred_behavior;
  }

  if ((task.lc_info.forbid_lane_change_left !=
       last_task_.lc_info.forbid_lane_change_left) ||
      (task.lc_info.forbid_lane_change_right !=
       last_task_.lc_info.forbid_lane_change_right)) {
  //   //LOG(ERROR) << "[HMI]lane change forbid signal [left] "
  //                << task.lc_info.forbid_lane_change_left << " [right] "
  //                << task.lc_info.forbid_lane_change_right;
  }

  if (task.is_under_ctrl) {
    if (!lc_context_.completed
        && task.user_perferred_behavior != 0 
        && last_task_.user_perferred_behavior != 0) {
      // NOTE for MLAD: 上一個lc_context沒有完成，但是變道的方向變成另外一邊了，需要手動設置爲完成
      //LOG(WARNING) << "[HMI]directly finish lane change due to the different user_perferred_behavior from"
                // << last_task_.user_perferred_behavior << " to " << task.user_perferred_behavior
                // << " while last task was not completed ";
      // lc_context_.completed = true;
      lc_context_.trigger_when_appropriate = true;
      last_lc_proposal_.trigger_time = stamp;
      preliminary_active_requests_.clear();
      // last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
      // // replane_task_ = true;
    }

    if (!lc_context_.completed) {
      if (!map_adapter_.IsLaneConsistent(lc_context_.ego_lane_id, ego_lane_id_)) {
        // in progress lane change and lane id change
        //LOG(WARNING) << "[HMI]lane change completed due to different lane id "
                    //  << lc_context_.ego_lane_id << " to " << ego_lane_id_
                    //  << ". Cd alc.";
        lc_context_.completed = true;
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } 
      else {
        if (task.user_perferred_behavior != 1 && 
            last_task_.user_perferred_behavior == 1) {
          // receive a lane cancel trigger
          //LOG(WARNING) << "[HMI]lane change cancel by stick "
                      //  << last_task_.user_perferred_behavior << " to "
                      //  << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } 
        else if (task.user_perferred_behavior != -1 &&
                   last_task_.user_perferred_behavior == -1) {
          // receive a lane cancel trigger
          //LOG(WARNING) << "[HMI]lane change cancel by stick "
                      //  << last_task_.user_perferred_behavior << " to "
                      //  << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } 
        else if (lc_context_.type == LaneChangeTriggerType::kActive) {
          if (bp_.cfg()
                  .function()
                  .active_lc()
                  .enable_auto_cancel_by_outdate_time() &&
              stamp > lc_context_.desired_operation_time +
                          bp_.cfg()
                              .function()
                              .active_lc()
                              .auto_cancel_if_late_for_seconds()) {
            if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
              //LOG(WARNING)
                  // << "[HMI]ACTIVE [Left] auto cancel due to outdated for "
                  // << stamp - lc_context_.desired_operation_time
                  // << " s. Cd alc.";
            } 
            else {
              //LOG(WARNING)
                  // << "[HMI]ACTIVE [Right] auto cancel due to outdated for "
                  // << stamp - lc_context_.desired_operation_time
                  // << " s. Cd alc.";
            }
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } 
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_left &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
            //LOG(WARNING) << "[HMI]ACTIVE [Left] canceled due to forbidden "
                            // "signal. Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } 
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_right &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight) {
            //LOG(WARNING)
                // << "[HMI]ACTIVE [Right] canceled due to forbidden signal. "
                //    "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } 
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
                     (task.user_perferred_behavior == 1 ||
                      task.user_perferred_behavior == 11)) {
            //LOG(WARNING)
                // << "[HMI]ACTIVE [left] canceled due to human opposite signal. "
                //    "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } 
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                     (task.user_perferred_behavior == -1 ||
                      task.user_perferred_behavior == 12)) {
            //LOG(WARNING) << "[HMI]ACTIVE canceled due to human active signal. "
                            // "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
        }
      }
    } 
    else {
      // lane change completed state: welcome new activations
      // handle user requirement first
      if (task.user_perferred_behavior != 1 &&
          last_task_.user_perferred_behavior == 1 &&
          lc_context_.trigger_when_appropriate) {
        //LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } 
      else if (task.user_perferred_behavior != -1 &&
                 last_task_.user_perferred_behavior == -1 &&
                 lc_context_.trigger_when_appropriate) {
        //LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      }

      if (task.user_perferred_behavior == 1 &&
          last_task_.user_perferred_behavior != 1) {
        // receive a lane change right trigger and previous action has been
        // completed
        if (task.lc_info.forbid_lane_change_right) {
          //LOG(WARNING)
              // << "[HMI]cannot stick [Right]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
        } 
        else {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            //LOG(WARNING) << std::fixed << std::setprecision(5)
                        //  << "[HMI]stick [Right] triggered "
                        //  << last_task_.user_perferred_behavior << "->"
                        //  << task.user_perferred_behavior << " in "
                        //  << bp_.cfg().function().stick_lane_change_in_seconds()
                        //  << " s. Trigger time " << lc_context_.trigger_time
                        //  << " and absolute action time: "
                        //  << lc_context_.desired_operation_time << ". Cd alc.";
          } 
          else {
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            //LOG(WARNING)
                // << std::fixed << std::setprecision(5)
                // << "[HMI]stick [Right] triggered "
                // << last_task_.user_perferred_behavior << "->"
                // << task.user_perferred_behavior
                // << " but not a good time. Will trigger when appropriate.";
          }
        }
      } 
      else if (task.user_perferred_behavior == -1 &&
                 last_task_.user_perferred_behavior != -1) {
        if (task.lc_info.forbid_lane_change_left) {
          //LOG(WARNING)
              // << "[HMI]cannot stick [Left]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
        } 
        else {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            //LOG(WARNING) << std::fixed << std::setprecision(5)
                        //  << "[HMI]stick [Left] triggered "
                        //  << last_task_.user_perferred_behavior << "->"
                        //  << task.user_perferred_behavior << " in "
                        //  << bp_.cfg().function().stick_lane_change_in_seconds()
                        //  << " s. Trigger time " << lc_context_.trigger_time
                        //  << " and absolute action time: "
                        //  << lc_context_.desired_operation_time << ". Cd alc.";
          } 
          else {
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            //LOG(WARNING)
                // << std::fixed << std::setprecision(5)
                // << "[HMI]stick [Left] triggered "
                // << last_task_.user_perferred_behavior << "->"
                // << task.user_perferred_behavior
                // << " but not a good time. Will trigger when appropriate.";
          }
        }
      } 
      else if (lc_context_.trigger_when_appropriate) {
        if (lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
            !task.lc_info.forbid_lane_change_left) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            //LOG(WARNING) << std::fixed << std::setprecision(5)
                        //  << "[HMI][[cached]] stick [Left] appropriate in "
                        //  << bp_.cfg().function().stick_lane_change_in_seconds()
                        //  << " s. Trigger time " << lc_context_.trigger_time
                        //  << " and absolute action time: "
                        //  << lc_context_.desired_operation_time << ". Cd alc.";
          }
        } 
        else if (lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                   !task.lc_info.forbid_lane_change_right) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            //LOG(WARNING) << std::fixed << std::setprecision(5)
                        //  << "[HMI][[cached]] stick [Right] triggered in "
                        //  << bp_.cfg().function().stick_lane_change_in_seconds()
                        //  << " s. Trigger time " << lc_context_.trigger_time
                        //  << " and absolute action time: "
                        //  << lc_context_.desired_operation_time << ". Cd alc.";
          }
        }
      } 
      else {
        if (last_lc_proposal_.valid &&
            map_adapter_.IsLaneConsistent(last_lc_proposal_.ego_lane_id,
                                          ego_lane_id_) &&
            stamp > last_lc_proposal_.trigger_time &&
            last_lc_proposal_.lat != LateralBehavior::kLaneKeeping) {
          if ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft &&
               !task.lc_info.forbid_lane_change_left) ||
              (last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight &&
               !task.lc_info.forbid_lane_change_right)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time =
                last_lc_proposal_.trigger_time +
                last_lc_proposal_.operation_at_seconds;
            lc_context_.ego_lane_id = last_lc_proposal_.ego_lane_id;
            lc_context_.lat = last_lc_proposal_.lat;
            lc_context_.type = LaneChangeTriggerType::kActive;
            last_lc_proposal_.trigger_time = stamp;
            if (last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) {
              //LOG(WARNING) << std::fixed << std::setprecision(5)
                          //  << "[HMI][[Active]] [Left] triggered in "
                          //  << last_lc_proposal_.operation_at_seconds
                          //  << " s. Trigger time " << lc_context_.trigger_time
                          //  << " and absolute action time: "
                          //  << lc_context_.desired_operation_time << ". Cd alc.";
            } 
            else {
              //LOG(WARNING) << std::fixed << std::setprecision(5)
                          //  << "[HMI][[Active]] [Right] triggered in "
                          //  << last_lc_proposal_.operation_at_seconds
                          //  << " s. Trigger time " << lc_context_.trigger_time
                          //  << " and absolute action time: "
                          //  << lc_context_.desired_operation_time << ". Cd alc.";
            }
          }
        }
      }
    }
  }  // if under control

  // any proposal will not last for more than one cycle
  //LOG(WARNING) << "UpdateLaneChangeContextByTask:lc_context_.completed end" << lc_context_.completed;
  last_lc_proposal_.valid = false;
  last_task_ = task;
}  // namespace planning

void EudmManager::SaveSnapshot(Snapshot* snapshot) {
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->final_cost = bp_.final_cost();
  snapshot->progress_cost = bp_.progress_cost();
  snapshot->tail_cost = bp_.tail_cost();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();
  snapshot->surround_trajs = bp_.surround_trajs();

  snapshot->plan_stamp = map_adapter_.map().time_stamp();
  snapshot->time_cost = bp_.time_cost();
}

void EudmManager::ConstructBehavior(common::SemanticBehavior* behavior) {
  if (not last_snapshot_.valid) return;
  int selected_seq_id = last_snapshot_.processed_winner_id;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_final;
  surround_trajs_final.emplace_back(
      last_snapshot_.surround_trajs[selected_seq_id]);
  behavior->lat_behavior =
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front();
  behavior->lon_behavior =
      last_snapshot_.forward_lon_behaviors[selected_seq_id].front();
  behavior->forward_trajs = vec_E<vec_E<common::Vehicle>>{
      last_snapshot_.forward_trajs[selected_seq_id]};
  behavior->forward_behaviors = std::vector<LateralBehavior>{
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front()};
  behavior->surround_trajs = surround_trajs_final;
  behavior->state = last_snapshot_.plan_state;
  behavior->ref_lane = last_snapshot_.ref_lane;
  behavior->ref_lane_id = last_snapshot_.ref_lane_id;
}

ErrorType EudmManager::EvaluateReferenceVelocity(
    const Task& task, decimal_t* ref_vel) {
  if (!last_snapshot_.ref_lane.IsValid()) {
    *ref_vel = task.user_desired_vel;
    return kSuccess;
  }
  common::StateTransformer stf(last_snapshot_.ref_lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(last_snapshot_.plan_state, &current_fs);

  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;

  decimal_t a_comfort = bp_.cfg().sim().ego().lon().limit().soft_brake();
  decimal_t t_forward = last_snapshot_.plan_state.velocity / a_comfort;
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * last_snapshot_.plan_state.velocity),
               last_snapshot_.ref_lane.end());
  decimal_t resolution = 0.2;

  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (last_snapshot_.ref_lane.GetCurvatureByArcLength(s, &c, &cc) ==
        kSuccess) {
      v_max_by_curvature =
          sqrt(bp_.cfg().sim().ego().lat().limit().acc() / fabs(c));
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  // *ref_vel = std::floor(std::min(std::max(v_ref, 0.0), task.user_desired_vel));
  *ref_vel = std::min(std::max(v_ref, 0.0), task.user_desired_vel); // NOTE for MLAD: 不需要速度爲整數

  //LOG(WARNING) << "[Eudm][Desired]User ref vel: " << task.user_desired_vel
              //  << ", final ref vel: " << *ref_vel;
  return kSuccess;
}

ErrorType EudmManager::ReselectByContext(const decimal_t stamp,
                                         const Snapshot& snapshot,
                                         int* new_seq_id) {
  // *new_seq_id = snapshot.original_winner_id;
  int selected_seq_id;
  int num_seqs = snapshot.action_script.size();
  bool find_match = false;
  decimal_t cost = kInf;

  for (int i = 0; i < num_seqs; i++) {
    if (!snapshot.sim_res[i]) continue;
    common::LateralBehavior lat_behavior;
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    bp_.ClassifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
                          &lat_behavior, &is_cancel_behavior);
    // printf("[ReselectByContext] lat_behavior: %d\n", static_cast<int>(lat_behavior));
    if ((lc_context_.completed &&
         lat_behavior == common::LateralBehavior::kLaneKeeping) ||
        (!lc_context_.completed && stamp < lc_context_.desired_operation_time &&
         (lat_behavior == common::LateralBehavior::kLaneKeeping || 
          lat_behavior == lc_context_.lat)) ||
        (!lc_context_.completed &&
         stamp >= lc_context_.desired_operation_time &&
         (lat_behavior == lc_context_.lat ||
          lat_behavior == common::LateralBehavior::kLaneKeeping))) {
      find_match = true;
      if (snapshot.final_cost[i] < cost) {
        cost = snapshot.final_cost[i];
        selected_seq_id = i;
      }
    }
    // TODO for MLAD: stamp < lc_context_.desired_operation_time 怎麼解決？？？
    // if ((lc_context_.completed &&
    //      lat_behavior == common::LateralBehavior::kLaneKeeping) ||
    //     (!lc_context_.completed &&
    //      stamp >= lc_context_.desired_operation_time &&
    //      (lat_behavior == lc_context_.lat ||
    //       lat_behavior == common::LateralBehavior::kLaneKeeping))) {
    //   find_match = true;
    //   if (snapshot.final_cost[i] < cost) {
    //     cost = snapshot.final_cost[i];
    //     selected_seq_id = i;
    //   }
    // }
  }

  if (!find_match) {
    return kWrongStatus;
  }
  *new_seq_id = selected_seq_id;
  return kSuccess;
}

ErrorType EudmManager::Run(
    const decimal_t stamp,
    const int cur_ego_id,
    const Task& task) {
  LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN START: " << stamp
               << " for id:" << cur_ego_id << "******************";
  static TicToc eudm_timer;
  eudm_timer.tic();

  // * I : Prepare
  static TicToc prepare_timer;
  prepare_timer.tic();

  if (Prepare(stamp, task) != kSuccess) {
    return kWrongStatus;
  }
  auto t_prepare = prepare_timer.toc();
  //LOG(ERROR) << std::fixed << std::setprecision(4)
              //  << "[Eudm]Prepare time cost " << t_prepare << " ms";

  // * II : RunOnce
  static TicToc runonce_timer;
  runonce_timer.tic();
  if (bp_.RunOnce() != kSuccess) {
    //LOG(ERROR) << "[Eudm][Fatal]BP runonce failed.";
    return kWrongStatus;
  }
  auto t_runonce = runonce_timer.toc();
  LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]RunOnce time cost " << t_runonce << " ms";

  static TicToc sum_reselect_timer;
  sum_reselect_timer.tic();
  // * III: Summarize
  Snapshot snapshot;
  SaveSnapshot(&snapshot);
  // * IV: Reselect
  if (ReselectByContext(stamp, snapshot, &snapshot.processed_winner_id) !=
      kSuccess) {
    //LOG(ERROR) << "[Eudm][Fatal]Reselect failed.";
    return kWrongStatus;
  }
  //LOG(WARNING) << "[Eudm]original id " << snapshot.original_winner_id
              //  << " reselect : " << snapshot.processed_winner_id;
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Output]Reselected <if_risky:"
              << snapshot.risky_res[snapshot.processed_winner_id] << ">[";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "<stamp,velocity,acceleration,curvature>";
    for (auto& v : snapshot.forward_trajs[snapshot.processed_winner_id]) {
      line_info << std::fixed << std::setprecision(5) << "<"
                << v.state().time_stamp - stamp << "," << v.state().velocity
                << "," << v.state().acceleration << "," << v.state().curvature
                << ">";
    }
    //LOG(WARNING) << line_info.str();
  }
  auto t_sum_reselect = sum_reselect_timer.toc();
  //LOG(WARNING) << std::fixed << std::setprecision(4)
              //  << "[Eudm]Sum & Reselect time cost " << t_sum_reselect << " ms";

  static TicToc lane_timer;
  lane_timer.tic();
  if (map_adapter_.map().GetRefLaneForStateByBehavior(
          snapshot.plan_state, std::vector<int>(),
          snapshot.forward_lat_behaviors[snapshot.processed_winner_id].front(),
          250.0, 20.0, true, &(snapshot.ref_lane), &(snapshot.ref_lane_id)) != kSuccess) {
    return kWrongStatus;
  }

  last_snapshot_ = snapshot;
  GenerateLaneChangeProposal(stamp, task);
  // * V: Update
  context_.is_valid = true;
  context_.seq_start_time = stamp;
  context_.action_seq = snapshot.action_script[snapshot.processed_winner_id];
  // if (UpdateCurEgoVehicleAfterPlanning(cur_ego_id) != kSuccess) {
  //   //LOG(ERROR) << "[Eudm] Update cur ego vehicle after planning failed ";
  // } // 规划结束时暂不更新，需要等到收到agv反馈后，再和轨迹一起更新
  auto t_lane = lane_timer.toc();
  //LOG(ERROR) << "[Eudm]Fit reflane & update cost: " << t_lane << " ms";

  auto t_sum = t_prepare + t_runonce + t_sum_reselect + t_lane;
  auto t_eudmrun = eudm_timer.toc();
  LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]Sum of time: " << t_sum
               << " ms, diff: " << t_eudmrun - t_sum << " ms";
  LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN FINISH: " << stamp << " +"
               << t_eudmrun << " ms ******************";
  return kSuccess;
}

ErrorType EudmManager::UpdateCurEgoVehicleConfigsAfterPlanning(const int &cur_ego_id) {
  // 对context_，last_snapshot_，lc_context_, last_lc_proposal_, preliminary_active_requests_进行更新。
  return map_adapter_.UpdateCurEgoVehiclePlanningConfigs(cur_ego_id, context_, last_snapshot_, last_lc_proposal_, lc_context_, preliminary_active_requests_);
}

bool EudmManager::GetReplanDesiredAction(const decimal_t current_time,
                                         DcpAction* desired_action) {
  if (!context_.is_valid) return false;
  decimal_t time_since_last_plan = current_time - context_.seq_start_time;
  if (time_since_last_plan < -kEPS) return false;
  decimal_t t_aggre = 0.0;
  bool find_match_action = false;
  int action_seq_len = context_.action_seq.size();
  for (int i = 0; i < action_seq_len; ++i) {
    t_aggre += context_.action_seq[i].t;
    if (time_since_last_plan + kEPS < t_aggre) {
      *desired_action = context_.action_seq[i];
      //LOG(WARNING) << "[Eudm::GetReplanDesiredAction] find match action: " << context_.action_seq[i];
      desired_action->t = t_aggre - time_since_last_plan;
      find_match_action = true;
      break;
    }
  }
  if (!find_match_action) {
    return false;
  }
  return true;
}

void EudmManager::Reset() {
  context_.Reset();
  last_task_.Reset();
  last_snapshot_.Reset();
  last_lc_proposal_.Reset();
  preliminary_active_requests_.clear();
  lc_context_.Reset();
}

EudmPlanner& EudmManager::planner() { return bp_; }

ErrorType EudmManager::GetCloudInfoMsg(const int &cur_ego_id, common::SemanticBehavior &behavior, vehicle_msgs::CloudInfo *msg) {
  return map_adapter_.GetCloudInfoMsgFromCurrentSemanticBehavior(cur_ego_id, behavior, msg);
}
}  // namespace planning