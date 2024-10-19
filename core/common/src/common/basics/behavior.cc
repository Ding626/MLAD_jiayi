/**
 * @file behavior.h
 * @author Yijun Mao
 * @brief
 * @version 0.1
 * @date 2022-3-11
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/behavior.h"

namespace common {

BehavioralVehicle::BehavioralVehicle() {}

BehavioralVehicle::BehavioralVehicle(const Vehicle &vehicle) {
	InitializeVehicle(vehicle);
}

void BehavioralVehicle::InitializeVehicle(const Vehicle &vehicle) {
	vehicle_ = vehicle;
	last_task_.is_under_ctrl = true;
}

ErrorType BehavioralVehicle::UpdateVehicle(const Vehicle &vehicle) {
	vehicle_.set_state(vehicle.state());
	vehicle_.set_ready_change_lane(vehicle.ready_change_lane());
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextTask(const Task & task) {
	last_task_ = task;
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextSemanticBehavior(const SemanticBehavior &behavior) {
	lat_behavior_ = behavior.lat_behavior;
	lon_behavior_ = behavior.lon_behavior;
	actual_desired_velocity_ = behavior.actual_desired_velocity;
	behavior_forward_trajs_ = behavior.forward_trajs;
	forward_behaviors_ = behavior.forward_behaviors;
	
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextTraj(const vec_E<State> &state_traj, 
																						const std::string &updated_behavior_uid) {
	// NOTE for MLAD: 注意多进程调用时，要加锁
	// std::lock_guard<std::mutex> guard(traj_update_mutex);
	// std::lock_guard<std::mutex> lock_guard_traj(traj_update_mutex_);
	// std::lock_guard<std::mutex> lock_guard_uid(updated_behavior_uid_mtx_); 

	vec_E<State> traj = state_traj;
	std::swap(motion_planned_forward_state_traj_, traj);
	updated_behavior_uid_ = updated_behavior_uid;
	return kSuccess;
}

ErrorType BehavioralVehicle::GetLastReplanningContext(ReplanningContext *context) {
	if (context == nullptr) return kWrongStatus;

	context->is_valid = is_last_action_valid_;
	if (is_last_action_valid_) {
		context->seq_start_time = last_action_seq_start_time_;
		context->action_seq = action_seq_;
	}

	return kSuccess; 
}

ErrorType BehavioralVehicle::GetLastTask(Task *task) {
	if (task == nullptr) return kWrongStatus;

	*task = last_task_;

	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextLaneChangeContext(const LaneChangeContext &lc_context) {
	lc_context_ = lc_context;
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextReplanningContext(const ReplanningContext &context) {
	is_last_action_valid_ = context.is_valid;
	last_action_seq_start_time_ = context.seq_start_time;
	action_seq_.clear();
	action_seq_ = context.action_seq;
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextSnapshotForPlanning(const Snapshot &snapshot) {
	last_snapshot_valid_ = snapshot.valid;
	original_winner_id_ = snapshot.original_winner_id;
	processed_winner_id_ = snapshot.processed_winner_id;
  plan_state_ = snapshot.plan_state;
	
  action_script_.clear();
	action_script_ = snapshot.action_script;
  sim_res_.clear();
	sim_res_ = snapshot.sim_res;
  risky_res_.clear();
	risky_res_ = snapshot.risky_res;
  sim_info_.clear();
	sim_info_ = snapshot.sim_info;
  final_cost_.clear();
	final_cost_ = snapshot.final_cost;
  progress_cost_.clear();
	progress_cost_ = snapshot.progress_cost;
  tail_cost_.clear();
	tail_cost_ = snapshot.tail_cost;
  forward_trajs_.clear();
	forward_trajs_ = snapshot.forward_trajs;
  forward_lat_behaviors_.clear();
	forward_lat_behaviors_ = snapshot.forward_lat_behaviors;
  forward_lon_behaviors_.clear();
	forward_lon_behaviors_ = snapshot.forward_lon_behaviors;
  surround_trajs_.clear();
	surround_trajs_ = snapshot.surround_trajs;

  ref_lane_ = snapshot.ref_lane;
	ref_lane_id_ = snapshot.ref_lane_id;

	plan_stamp_ = snapshot.plan_stamp;
  time_cost_ = snapshot.time_cost;

	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextLaneChangeProposal(const LaneChangeProposal &lc_proposal) {
	last_lc_proposal_ = lc_proposal;
	return kSuccess;
}

ErrorType BehavioralVehicle::UpdateNextPreliminaryActiveRequests(
			const std::vector<ActivateLaneChangeRequest> &preliminary_active_requests) {
	preliminary_active_requests_.clear();
	preliminary_active_requests_ = preliminary_active_requests;
	return kSuccess;
}

ErrorType BehavioralVehicle::GetLastSnapshotForPlanning(Snapshot *snapshot) {
	if (snapshot == nullptr) return kWrongStatus;

	if (last_snapshot_valid_) {
		snapshot->valid = last_snapshot_valid_;
		snapshot->original_winner_id = original_winner_id_;
		snapshot->processed_winner_id = processed_winner_id_;
		snapshot->plan_state = plan_state_;
		snapshot->action_script = action_script_;
		snapshot->sim_res = sim_res_;
		snapshot->risky_res = risky_res_;
		snapshot->sim_info = sim_info_;
		snapshot->final_cost = final_cost_;
		snapshot->progress_cost = progress_cost_;
		snapshot->tail_cost = tail_cost_;
		snapshot->forward_trajs = forward_trajs_;
		snapshot->forward_lat_behaviors = forward_lat_behaviors_;
		snapshot->forward_lon_behaviors = forward_lon_behaviors_;
		snapshot->surround_trajs = surround_trajs_;
		snapshot->ref_lane = ref_lane_;
		snapshot->ref_lane_id = ref_lane_id_;
		snapshot->plan_stamp = plan_stamp_;
		snapshot->time_cost = time_cost_;
	}
	
	return kSuccess; 
}

ErrorType BehavioralVehicle::GetLastLaneChangeProposal(LaneChangeProposal *lc_proposal) {
	if (lc_proposal == nullptr) return kWrongStatus;

	*lc_proposal = last_lc_proposal_;

	return kSuccess; 
}

ErrorType BehavioralVehicle::GetLastPreliminaryActiveRequests(std::vector<ActivateLaneChangeRequest> *preliminary_active_requests) {
	if (preliminary_active_requests == nullptr) return kWrongStatus;

	*preliminary_active_requests = preliminary_active_requests_;

	return kSuccess; 
}

ErrorType BehavioralVehicle::GetLastLaneChangeContext(LaneChangeContext *lc_context) {
	if (lc_context == nullptr) return kWrongStatus;

	*lc_context = lc_context_;

	return kSuccess; 
}

bool BehavioralVehicle::IsRunningPlannedTrajAtSpecTime(const decimal_t &time) {
	// std::lock_guard<std::mutex> lock_guard(traj_update_mutex_);

	if (motion_planned_forward_state_traj_.empty()) return false;

	decimal_t start_t = motion_planned_forward_state_traj_.front().time_stamp;
	decimal_t end_t = motion_planned_forward_state_traj_.back().time_stamp;

	if (start_t <= time && end_t >= time) return true;
	else return false;
}

ErrorType BehavioralVehicle::GetLastBehaviorUIDWithTime(std::string *behavior_uid, ros::Time *time_stamp) {
	*behavior_uid = behavior_uid_;
	*time_stamp = last_behavior_time_stamp_;
	return kSuccess;
}

ErrorType BehavioralVehicle::GetExecutingStateAtSpecTime(const decimal_t &time, State *state_output) {
	// std::lock_guard<std::mutex> guard(traj_update_mutex);
	// std::lock_guard<std::mutex> lock_guard(traj_update_mutex_);

	if (motion_planned_forward_state_traj_.empty()) return kWrongStatus;

	state_output->time_stamp = time;
	// 若指定时间为当前的时间，则直接返回当前的真实state；否则返回时间最近的那一个state（主要由于上传的轨迹应该很稠密，所以这样做了）
	if (std::fabs(time - vehicle_.state().time_stamp) <= kEPS) {
		state_output->vec_position = vehicle_.state().vec_position;
		state_output->angle = vehicle_.state().angle;
		state_output->curvature = vehicle_.state().curvature;
		state_output->velocity = vehicle_.state().velocity;
		state_output->acceleration = vehicle_.state().acceleration;
		state_output->steer = vehicle_.state().steer;
		state_output->time_stamp = vehicle_.state().time_stamp;

		return kSuccess;
	}

	if (time < motion_planned_forward_state_traj_.front().time_stamp || time > motion_planned_forward_state_traj_.back().time_stamp) {
		return kWrongStatus;
	}

	for (int i=1; i < static_cast<int>(motion_planned_forward_state_traj_.size()); ++i) {
		if (time >= motion_planned_forward_state_traj_[i-1].time_stamp && time < motion_planned_forward_state_traj_[i].time_stamp) {
			decimal_t dt_start = time - motion_planned_forward_state_traj_[i-1].time_stamp;
			decimal_t dt_end = motion_planned_forward_state_traj_[i].time_stamp - time;
			State target_state = dt_start < dt_end ? motion_planned_forward_state_traj_[i-1] : motion_planned_forward_state_traj_[i];

			state_output->vec_position = target_state.vec_position;
			state_output->angle = target_state.angle;
			state_output->curvature = target_state.curvature;
			state_output->velocity = target_state.velocity;
			state_output->acceleration = target_state.acceleration;
			state_output->steer = target_state.steer;
			state_output->time_stamp = target_state.time_stamp;

			return kSuccess;
		}
	}

	return kWrongStatus;
}

BehavioralLane::BehavioralLane() {}

BehavioralLane::BehavioralLane(const LaneRaw &raw_lane){
	Initialize(raw_lane);
}

void BehavioralLane::Initialize(const LaneRaw &raw_lane) {

	id_ = raw_lane.id;
	dir_ = raw_lane.dir;
	child_id_ = raw_lane.child_id;
	father_id_ = raw_lane.father_id;
	l_lane_id_ = raw_lane.l_lane_id;
	l_change_avbl_ = raw_lane.l_change_avbl;
	r_lane_id_ = raw_lane.r_lane_id;
	r_change_avbl_ = raw_lane.r_change_avbl;
	dir_changeable_ = raw_lane.dir_changeable;
	behavior_ = raw_lane.behavior;
	length_ = raw_lane.length;
	buffer_ = raw_lane.buffer;
	group_ = raw_lane.group;

	// agv数量的初始化默认为0

	// vec_Vecf<2> samples;
	// for (const auto &pt : raw_lane.lane_points) {
	// 	samples.push_back(pt);
	// }
	// if (LaneGenerator::GetLaneBySamplePoints(samples, &lane_) != kSuccess) {
	// 	return;
	// }
}

BehavioralLaneSet::BehavioralLaneSet() {}

BehavioralLaneSet::BehavioralLaneSet(const LaneNet &whole_lane_net) { Initialize(whole_lane_net); }

void BehavioralLaneSet::Initialize(const LaneNet &whole_lane_net) {
	behavioral_lanes.clear();
	all_lane_ids.clear();
	// intialize hahavioral lane
	for (const auto &pe : whole_lane_net.lane_set) {
		if (pe.second.is_multi_lane_route) {
			BehavioralLane behavioral_lane;
			behavioral_lane.Initialize(pe.second);
			behavioral_lanes.insert(std::make_pair(pe.first, behavioral_lane));
			all_lane_ids.emplace_back(pe.first);
		}
	}
	// // 初始化车道方向，不同group分别初始化
	// for (auto iter = groups_behavioral_lane_ids.begin(); iter != groups_behavioral_lane_ids.end(); iter++) {
	// 	int lane_num = iter->second.size();
	// 	int main_dir_lane_num = (lane_num + 1) >> 1;
	// 	std::sort(iter->second.begin(), iter->second.end(), std::less());
	// 	// 车道id越小，说明其在主方向上越靠右，最右边的车道始终是主方向
	// 	for (int i=0; i<iter->second.size(); i++) {
	// 		behavioral_lanes[iter->second[i]].set_cur_pass_dir_straight(i < main_dir_lane_num ? true : false);
	// 	}
	// }

	int lane_num = all_lane_ids.size();
	int main_dir_lane_num = (lane_num + 1) >> 1;
	std::sort(all_lane_ids.begin(), all_lane_ids.end(), std::less<int>());
	// 车道id越小，说明其在主方向上越靠右，最右边的车道始终是主方向
	for (int i=0; i<lane_num; i++) {
		behavioral_lanes[all_lane_ids[i]].set_cur_pass_dir_straight(i < main_dir_lane_num ? true : false);
	}
}

}