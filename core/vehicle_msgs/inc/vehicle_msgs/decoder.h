#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__

#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/basics/behavior.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/ArenaInfo.h"
#include "vehicle_msgs/ArenaInfoDynamic.h"
#include "vehicle_msgs/ArenaInfoStatic.h"
#include "vehicle_msgs/Behavior.h"
#include "vehicle_msgs/CloudInfo.h"
#include "vehicle_msgs/Circle.h"
#include "vehicle_msgs/CircleObstacle.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/FreeState.h"
#include "vehicle_msgs/Lane.h"
#include "vehicle_msgs/LaneNet.h"
#include "vehicle_msgs/ObstacleSet.h"
#include "vehicle_msgs/OccupancyGridFloat.h"
#include "vehicle_msgs/OccupancyGridUInt8.h"
#include "vehicle_msgs/PolygonObstacle.h"
#include "vehicle_msgs/State.h"
#include "vehicle_msgs/Vehicle.h"
#include "vehicle_msgs/VehicleTraj.h"
#include "vehicle_msgs/VehicleParam.h"
#include "vehicle_msgs/VehicleSet.h"
#include "vehicle_msgs/Traj.h"
#include "vehicle_msgs/TrajUpdateInfo.h"
#include "vehicle_msgs/Task.h"
#include "vehicle_msgs/TaskSet.h"

namespace vehicle_msgs {
class Decoder {
 public:
  static ErrorType GetFreeStateMsgFromRosFreeState(
      const vehicle_msgs::FreeState &in_state, common::FreeState *state) {
    state->time_stamp = in_state.header.stamp.toSec();
    state->position[0] = in_state.pos.x;
    state->position[1] = in_state.pos.y;
    state->velocity[0] = in_state.vel.x;
    state->velocity[1] = in_state.vel.y;
    state->acceleration[0] = in_state.acc.x;
    state->acceleration[1] = in_state.acc.y;
    state->angle = in_state.angle;
    return kSuccess;
  }

  static ErrorType GetStateFromRosStateMsg(const vehicle_msgs::State &in_state,
                                           common::State *state) {
    state->time_stamp = in_state.stamp;
    state->vec_position[0] = in_state.vec_position.x;
    state->vec_position[1] = in_state.vec_position.y;
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  static ErrorType GetVehicleSetFromRosVehicleSet(
      const vehicle_msgs::VehicleSet &msg, common::VehicleSet *p_vehicle_set) {
    p_vehicle_set->vehicles.clear();
    for (int i = 0; i < (int)msg.vehicles.size(); ++i) {
      common::Vehicle vehicle;
      GetVehicleFromRosVehicle(msg.vehicles[i], &vehicle);
      p_vehicle_set->vehicles.insert(
          std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
    }
    return kSuccess;
  }

  static ErrorType GetTaskSetFromRosTaskSet(
      const vehicle_msgs::TaskSet &msg, common::TaskSet *p_task_set) {
    p_task_set->lc_tasks.clear();
    for (int i = 0; i < (int)msg.tasks.size(); ++i) {
      common::Task task;
      GetTaskFromRosTask(msg.tasks[i], &task);
      p_task_set->lc_tasks.insert(
          std::pair<int, common::Task>(msg.tasks[i].vehicle_id, task));
    }
    return kSuccess;
  }

  static ErrorType GetVehicleVectorFromRosVehicleTraj(
      const vehicle_msgs::VehicleTraj &msg, vec_E<common::Vehicle> *p_vehicle_set) {
    p_vehicle_set->clear();

    common::VehicleParam param;
    GetVehicleParamFromRosVehicleParam(msg.param, &param);

    for (int i = 0; i < (int)msg.states.size(); ++i) {
      common::Vehicle vehicle;
      vehicle.set_id(msg.id.data);
      vehicle.set_subclass(msg.subclass.data);
      vehicle.set_type(msg.type.data);
      vehicle.set_ready_change_lane(msg.ready_change_lane);
      vehicle.set_param(param);

      common::State state;
      GetStateFromRosStateMsg(msg.states[i], &state);
      vehicle.set_state(state);

      p_vehicle_set->push_back(vehicle);
    }
    return kSuccess;
  }

  static ErrorType GetVehicleFromRosVehicle(const vehicle_msgs::Vehicle &msg,
                                            common::Vehicle *p_vehicle) {
    p_vehicle->set_id(msg.id.data);
    p_vehicle->set_subclass(msg.subclass.data);
    p_vehicle->set_type(msg.type.data);
    p_vehicle->set_ready_change_lane(msg.ready_change_lane);
    common::VehicleParam param;
    GetVehicleParamFromRosVehicleParam(msg.param, &param);
    common::State state;
    GetStateFromRosStateMsg(msg.state, &state);
    p_vehicle->set_param(param);
    p_vehicle->set_state(state);
    return kSuccess;
  }

  static ErrorType GetTaskFromRosTask(const vehicle_msgs::Task &msg,
                                            common::Task *p_task) {
    p_task->is_under_ctrl = true;
    p_task->user_desired_vel = msg.user_desired_speed;
    p_task->user_perferred_behavior = msg.user_desired_behavior; // 1为向right右变道，-1为向left变道
    // LaneChangeInfo lc_info;

    p_task->target_lane_id = msg.target_lane;
    return kSuccess;
  }

  static ErrorType GetVehicleParamFromRosVehicleParam(
      const vehicle_msgs::VehicleParam &msg,
      common::VehicleParam *p_vehicle_param) {
    p_vehicle_param->set_width(msg.width);
    p_vehicle_param->set_length(msg.length);
    p_vehicle_param->set_wheel_base(msg.wheel_base);
    p_vehicle_param->set_front_suspension(msg.front_suspension);
    p_vehicle_param->set_rear_suspension(msg.rear_suspension);
    p_vehicle_param->set_max_steering_angle(msg.max_steering_angle);
    p_vehicle_param->set_max_longitudinal_acc(msg.max_longitudinal_acc);
    p_vehicle_param->set_max_lateral_acc(msg.max_lateral_acc);
    p_vehicle_param->set_d_cr(msg.d_cr);
    return kSuccess;
  }

  static ErrorType GetLaneNetFromRosLaneNet(const vehicle_msgs::LaneNet &msg,
                                            common::LaneNet *p_lane_net) {
    p_lane_net->lane_set.clear();
    for (const auto lane_msg : msg.lanes) {
      common::LaneRaw lane_raw;
      GetLaneRawFromRosLane(lane_msg, &lane_raw);
      p_lane_net->lane_set.insert(
          std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
    }
    return kSuccess;
  }

  static ErrorType GetLaneRawFromRosLane(const vehicle_msgs::Lane &msg,
                                         common::LaneRaw *p_lane) {
    p_lane->id = msg.id;
    p_lane->dir = msg.dir;

    p_lane->child_id = msg.child_id;
    p_lane->father_id = msg.father_id;
    p_lane->l_lane_id = msg.l_lane_id;
    p_lane->l_change_avbl = msg.l_change_avbl;
    p_lane->r_lane_id = msg.r_lane_id;
    p_lane->r_change_avbl = msg.r_change_avbl;
    p_lane->behavior = msg.behavior;
    p_lane->length = msg.length;

    p_lane->start_point(0) = msg.start_point.x;
    p_lane->start_point(1) = msg.start_point.y;
    p_lane->final_point(0) = msg.final_point.x;
    p_lane->final_point(1) = msg.final_point.y;
    for (const auto pt : msg.points) {
      p_lane->lane_points.push_back(Vec2f(pt.x, pt.y));
    }
    return kSuccess;
  }


  static ErrorType GetObstacleSetFromRosObstacleSet(
      const vehicle_msgs::ObstacleSet &msg, common::ObstacleSet *obstacle_set) {
    obstacle_set->obs_circle.clear();
    obstacle_set->obs_polygon.clear();
    for (const auto obs : msg.obs_circle) {
      common::CircleObstacle obs_temp;
      GetCircleObstacleFromRosCircleObstacle(obs, &obs_temp);
      obstacle_set->obs_circle.insert(
          std::pair<int, common::CircleObstacle>(obs_temp.id, obs_temp));
    }
    for (const auto obs : msg.obs_polygon) {
      common::PolygonObstacle obs_temp;
      GetPolygonObstacleFromRosPolygonObstacle(obs, &obs_temp);
      obstacle_set->obs_polygon.insert(
          std::pair<int, common::PolygonObstacle>(obs_temp.id, obs_temp));
    }
    return kSuccess;
  }

  static ErrorType GetCircleObstacleFromRosCircleObstacle(
      const vehicle_msgs::CircleObstacle &msg, common::CircleObstacle *circle) {
    circle->id = msg.id;
    GetCircleFromRosCircle(msg.circle, &circle->circle);
    return kSuccess;
  }

  static ErrorType GetPolygonObstacleFromRosPolygonObstacle(
      const vehicle_msgs::PolygonObstacle &msg, common::PolygonObstacle *poly) {
    poly->id = msg.id;
    GetPolygonFromRosPolygon(msg.polygon, &poly->polygon);
    return kSuccess;
  }

  static ErrorType GetCircleFromRosCircle(const vehicle_msgs::Circle &msg,
                                          common::Circle *circle) {
    circle->center.x = msg.center.x;
    circle->center.y = msg.center.y;
    circle->radius = msg.radius;
    return kSuccess;
  }

  static ErrorType GetPolygonFromRosPolygon(const geometry_msgs::Polygon &msg,
                                            common::Polygon *poly) {
    for (const auto p : msg.points) {
      common::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      poly->points.push_back(pt);
    }
    return kSuccess;
  }

  static ErrorType GetSimulatorDataFromRosArenaInfo(
      const vehicle_msgs::ArenaInfo &msg, ros::Time *time_stamp,
      common::LaneNet *lane_net, common::VehicleSet *vehicle_set,
      common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  static ErrorType GetSimulatorDataFromRosArenaInfoStatic(
      const vehicle_msgs::ArenaInfoStatic &msg, ros::Time *time_stamp,
      common::LaneNet *lane_net, common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  static ErrorType GetSimulatorDataFromRosArenaInfoDynamic(
      const vehicle_msgs::ArenaInfoDynamic &msg, ros::Time *time_stamp,
      common::VehicleSet *vehicle_set) {
    *time_stamp = msg.header.stamp;
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    return kSuccess;
  }

  static ErrorType GetTaskFromTaskSet(const vehicle_msgs::TaskSet &msg,
                                      ros::Time *time_stamp,
                                      common::TaskSet *taskset) {
    *time_stamp = msg.header.stamp;
    GetTaskSetFromRosTaskSet(msg, taskset);
    return kSuccess;
  }

  static ErrorType GetControlSignalFromRosControlSignal(
      const vehicle_msgs::ControlSignal &msg,
      common::VehicleControlSignal *ctrl) {
    ctrl->acc = msg.acc;
    ctrl->steer_rate = msg.steer_rate;
    ctrl->is_openloop = msg.is_openloop.data;
    GetStateFromRosStateMsg(msg.state, &(ctrl->state));
    return kSuccess;
  }

  static ErrorType GetSurroundTrajsFromRosSurroundTrajs(
    const vehicle_msgs::SurroundTraj &msg, std::unordered_map<int, vec_E<common::Vehicle>> *surround_trajs){
      surround_trajs->clear();
      for(int i = 0; i < msg.ids.size(); ++i) {
        vec_E<common::Vehicle> straj;
        GetVehicleVectorFromRosVehicleTraj(msg.surround_traj[i], &straj);
        surround_trajs->insert(
          std::pair<int, vec_E<common::Vehicle>>(msg.ids[i], straj)
        );
      }
    return kSuccess;
  }

  static ErrorType GetBehaviorsetFromBehavior(const vehicle_msgs::Behavior &msg,
                                            common::SemanticBehavior *p_behavior){
    p_behavior->forward_trajs.clear();
    p_behavior->forward_behaviors.clear();
    p_behavior->surround_trajs.clear();
    switch (msg.lat_behavior)
    {
      case 0:
        p_behavior->lat_behavior = common::LateralBehavior::kUndefined;
        break;
      case 1:
        p_behavior->lat_behavior = common::LateralBehavior::kLaneKeeping;
        break;
      case 2:
        p_behavior->lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        break;
      case 3:
        p_behavior->lat_behavior = common::LateralBehavior::kLaneChangeRight;
        break;
    
      default:
        break;
    }

    switch (msg.lon_behavior)
    {
      case 0:
        p_behavior->lon_behavior = common::LongitudinalBehavior::kMaintain;
        break;
      case 1:
        p_behavior->lon_behavior = common::LongitudinalBehavior::kAccelerate;
        break;
      case 2:
        p_behavior->lon_behavior = common::LongitudinalBehavior::kDecelerate;
        break;
      case 3:
        p_behavior->lon_behavior = common::LongitudinalBehavior::kStopping;
        break;
    
      default:
        break;
    }

    p_behavior->actual_desired_velocity = msg.actual_desired_velocity;
    for (const auto ftraj : msg.forward_trajs) {
      vec_E<common::Vehicle> traj_temp;
      GetVehicleVectorFromRosVehicleTraj(ftraj, &traj_temp);
      p_behavior->forward_trajs.push_back(traj_temp);
    }

    for (int i = 0; i < (int)msg.forward_behaviors.size(); ++i) {
      common::LateralBehavior lateralBehavior_temp;
      switch (msg.forward_behaviors[i])
      {
        case 0:
          lateralBehavior_temp = common::LateralBehavior::kUndefined;
          break;
        case 1:
          lateralBehavior_temp = common::LateralBehavior::kLaneKeeping;
          break;
        case 2:
          lateralBehavior_temp = common::LateralBehavior::kLaneChangeLeft;
          break;
        case 3:
          lateralBehavior_temp = common::LateralBehavior::kLaneChangeRight;
          break;
      
        default:
          break;
      }
      p_behavior->forward_behaviors.push_back(lateralBehavior_temp);
    }

    for(const auto straj : msg.surround_trajs) {
      std::unordered_map<int, vec_E<common::Vehicle>> straj_temp;
      GetSurroundTrajsFromRosSurroundTrajs(straj, &straj_temp);
      p_behavior->surround_trajs.push_back(straj_temp);
    }

    common::State state;
    GetStateFromRosStateMsg(msg.state, &state);
    p_behavior->state = state;

    return kSuccess;
  }


  // static ErrorType GetCloudDataFromSscBpInfo(
  //     const vehicle_msgs::CloudInfo &msg, ros::Time *time_stamp,
  //     common::Vehicle *ego_vehicle, 
  //     common::LaneNet *lane_net, common::VehicleSet *surround_vehicles,
  //     common::ObstacleSet *obstacle_set, common::SemanticBehavior *behavior_set, int *lane_id) {
  //   *time_stamp = msg.header.stamp;
  //   *lane_id = msg.behavior.lane_id;
  //   GetBehaviorsetFromBehavior(msg.behavior, behavior_set);
  //   GetVehicleFromRosVehicle(msg.ego_vehicle, ego_vehicle);
  //   GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
  //   GetVehicleSetFromRosVehicleSet(msg.surrounding_vehicles, surround_vehicles);
  //   GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
  //   return kSuccess;
  // }

  static ErrorType GetCloudDataFromCloudInfo(
      const vehicle_msgs::CloudInfo &msg, ros::Time *time_stamp,
      common::SemanticBehavior *behavior, std::string *uid) {
    *time_stamp = msg.header.stamp;
    GetBehaviorsetFromBehavior(msg.behavior, behavior);
    *uid = msg.uid.data;
    return kSuccess;
  }

  static ErrorType GetUpdatedTrajFromTrajUpdateInfo(
      const vehicle_msgs::TrajUpdateInfo &msg, ros::Time *time_stamp,
      vec_E<common::State> *traj, std::string *behavior_uid, int *vehicle_id) {
    *time_stamp = msg.header.stamp;
    for(int i = 0; i < msg.states.size(); ++i) {
      common::State state;
      GetStateFromRosStateMsg(msg.states[i], &state);
      traj->push_back(state);
    }
    *behavior_uid = msg.uid.data;
    *vehicle_id = msg.id.data;
    return kSuccess;
  }
  
};

}  // namespace vehicle_msgs

#endif  //_VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__