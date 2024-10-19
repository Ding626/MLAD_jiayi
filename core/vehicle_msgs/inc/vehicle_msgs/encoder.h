#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__

#include <algorithm>
#include <vector>
#include <unordered_map>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include "vehicle_msgs/ArenaInfo.h"
#include "vehicle_msgs/ArenaInfoDynamic.h"
#include "vehicle_msgs/ArenaInfoStatic.h"
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
#include "vehicle_msgs/SurroundTraj.h"
#include "vehicle_msgs/Behavior.h"
#include "vehicle_msgs/CloudInfo.h"
#include "vehicle_msgs/Traj.h"
#include "vehicle_msgs/TrajUpdateInfo.h"

#include <kdl/frames.hpp>

namespace vehicle_msgs {
class Encoder {
 public:
  static ErrorType GetRosFreeStateMsgFromFreeState(
      const common::FreeState &in_state, const ros::Time &timestamp,
      vehicle_msgs::FreeState *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = std::string("map");
    state->pos.x = in_state.position[0];
    state->pos.y = in_state.position[1];
    state->vel.x = in_state.velocity[0];
    state->vel.y = in_state.velocity[1];
    state->acc.x = in_state.acceleration[0];
    state->acc.y = in_state.acceleration[1];
    state->angle = in_state.angle;
    return kSuccess;
  }

  static ErrorType GetRosStateMsgFromState(const common::State &in_state,
                                           const ros::Time &timestamp,
                                           vehicle_msgs::State *state) {
    state->header.stamp = timestamp;
    state->stamp = timestamp.toSec();
    state->header.frame_id = std::string("map");
    state->vec_position.x = in_state.vec_position[0];
    state->vec_position.y = in_state.vec_position[1];
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  static ErrorType GetRosStateMsgFromState(const common::State &in_state,
                                           const ros::Time &timestamp,
                                           const std::string &frame_id,
                                           vehicle_msgs::State *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = frame_id;
    state->stamp = in_state.time_stamp;
    state->vec_position.x = in_state.vec_position[0];
    state->vec_position.y = in_state.vec_position[1];
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  static ErrorType GetRosVehicleSetFromVehicleSet(
      const common::VehicleSet &vehicle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::VehicleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto &p : vehicle_set.vehicles) {
      vehicle_msgs::Vehicle vehicle_msg;
      GetRosVehicleFromVehicle(p.second, timestamp, frame_id, &vehicle_msg);
      msg->vehicles.push_back(vehicle_msg);
    }
    return kSuccess;
  }

  static ErrorType GetRosVehicleSetFromVehicleSetForCloud(
      const common::VehicleSet &vehicle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::VehicleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto &p : vehicle_set.vehicles) {
      vehicle_msgs::Vehicle vehicle_msg;
      GetRosVehicleFromVehicleForCloud(p.second, timestamp, frame_id, &vehicle_msg);
      msg->vehicles.push_back(vehicle_msg);
    }
    return kSuccess;
  }

  static ErrorType GetRosVehicleFromVehicleForCloud(const common::Vehicle &vehicle,
                                            const ros::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::Vehicle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id.data = vehicle.id();
    msg->subclass.data = vehicle.subclass();
    msg->type.data = vehicle.type();
    vehicle_msgs::VehicleParam param;
    GetRosVehicleParamFromVehicleParam(vehicle.param(), &param);
    msg->param = param;
    vehicle_msgs::State state;
    GetRosStateMsgFromState(vehicle.state(), timestamp, frame_id, &state);
    msg->state = state;
    return kSuccess;
  }

  static ErrorType GetRosVehicleFromVehicle(const common::Vehicle &vehicle,
                                            const ros::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::Vehicle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id.data = vehicle.id();
    msg->subclass.data = vehicle.subclass();
    msg->type.data = vehicle.type();
    msg->ready_change_lane = vehicle.ready_change_lane();
    vehicle_msgs::VehicleParam param;
    GetRosVehicleParamFromVehicleParam(vehicle.param(), &param);
    msg->param = param;
    vehicle_msgs::State state;
    GetRosStateMsgFromState(vehicle.state(), timestamp, &state);
    msg->state = state;
    return kSuccess;
  }

  static ErrorType GetRosVehicleTrajFromVehicleVector(
      const vec_E<common::Vehicle> &vehicle_vec, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::VehicleTraj *msg) {
    // msg->header.frame_id = frame_id;
    // msg->header.stamp = timestamp;
    if (vehicle_vec.empty()) return kSuccess;

    msg->id.data = vehicle_vec.front().id();
    msg->subclass.data = vehicle_vec.front().subclass();
    msg->type.data = vehicle_vec.front().type();
    msg->ready_change_lane = vehicle_vec.front().ready_change_lane();
    vehicle_msgs::VehicleParam param;
    GetRosVehicleParamFromVehicleParam(vehicle_vec.front().param(), &param);
    msg->param = param;

    for (const auto &p : vehicle_vec) {
      vehicle_msgs::State state;
      GetRosStateMsgFromState(p.state(), timestamp, frame_id, &state);
      msg->states.push_back(state);
    }
    return kSuccess;
  }

  static ErrorType GetRosSurroundTrajFromTraj(const std::unordered_map<int, vec_E<common::Vehicle>> &surround_traj,
                                              const ros::Time &timestamp,
                                              const std::string &frame_id,
                                              vehicle_msgs::SurroundTraj *msg) {
    for (const auto &p : surround_traj) {
      msg->ids.push_back(p.first);
      vehicle_msgs::VehicleTraj vehicle_traj;
      GetRosVehicleTrajFromVehicleVector(p.second, timestamp, frame_id, &vehicle_traj);
      msg->surround_traj.push_back(vehicle_traj);
    }
    return kSuccess;
  }

  static ErrorType GetRosVehicleParamFromVehicleParam(
      const common::VehicleParam &vehicle_param,
      vehicle_msgs::VehicleParam *msg) {
    msg->width = vehicle_param.width();
    msg->length = vehicle_param.length();
    msg->wheel_base = vehicle_param.wheel_base();
    msg->front_suspension = vehicle_param.front_suspension();
    msg->rear_suspension = vehicle_param.rear_suspension();
    msg->max_steering_angle = vehicle_param.max_steering_angle();
    msg->max_longitudinal_acc = vehicle_param.max_longitudinal_acc();
    msg->max_lateral_acc = vehicle_param.max_lateral_acc();
    msg->d_cr = vehicle_param.d_cr();
    return kSuccess;
  }

  static ErrorType GetRosLaneNetFromLaneNet(const common::LaneNet &lane_net,
                                            const ros::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::LaneNet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto lane_raw : lane_net.lane_set) {
      vehicle_msgs::Lane lane;
      GetRosLaneFromLaneRaw(lane_raw.second, timestamp, frame_id, &lane);
      msg->lanes.push_back(lane);
    }
    return kSuccess;
  }

  static ErrorType GetRosLaneFromLaneRaw(const common::LaneRaw &lane,
                                         const ros::Time &timestamp,
                                         const std::string &frame_id,
                                         vehicle_msgs::Lane *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = lane.id;
    msg->dir = lane.dir;
    msg->child_id = lane.child_id;
    msg->father_id = lane.father_id;
    msg->l_lane_id = lane.l_lane_id;
    msg->l_change_avbl = lane.l_change_avbl;
    msg->r_lane_id = lane.r_lane_id;
    msg->r_change_avbl = lane.r_change_avbl;
    msg->behavior = lane.behavior;
    msg->length = lane.length;
    msg->start_point.x = lane.start_point(0);
    msg->start_point.y = lane.start_point(1);
    msg->start_point.z = 0.0;
    msg->final_point.x = lane.final_point(0);
    msg->final_point.y = lane.final_point(1);
    msg->final_point.z = 0.0;
    for (const auto pt : lane.lane_points) {
      geometry_msgs::Point p;
      p.x = pt(0);
      p.y = pt(1);
      p.z = 0.0;
      msg->points.push_back(p);
    }
    return kSuccess;
  }

  static ErrorType GetRosObstacleSetFromObstacleSet(
      const common::ObstacleSet &obstacle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ObstacleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto obs : obstacle_set.obs_circle) {
      vehicle_msgs::CircleObstacle obs_temp;
      GetRosCircleObstacleFromCircleObstacle(obs.second, timestamp, frame_id,
                                             &obs_temp);
      msg->obs_circle.push_back(obs_temp);
    }
    for (const auto obs : obstacle_set.obs_polygon) {
      vehicle_msgs::PolygonObstacle obs_temp;
      GetRosPolygonObstacleFromPolygonObstacle(obs.second, timestamp, frame_id,
                                               &obs_temp);
      msg->obs_polygon.push_back(obs_temp);
    }
    return kSuccess;
  }

  static ErrorType GetRosCircleObstacleFromCircleObstacle(
      const common::CircleObstacle &circle, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::CircleObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = circle.id;
    GetRosCircleFromCircle(circle.circle, &msg->circle);
    return kSuccess;
  }

  static ErrorType GetRosPolygonObstacleFromPolygonObstacle(
      const common::PolygonObstacle &poly, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::PolygonObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = poly.id;
    GetRosPolygonFromPolygon(poly.polygon, &msg->polygon);
    return kSuccess;
  }

  static ErrorType GetRosCircleFromCircle(const common::Circle &circle,
                                          vehicle_msgs::Circle *msg) {
    msg->center.x = circle.center.x;
    msg->center.y = circle.center.y;
    msg->center.z = 0.0;
    msg->radius = circle.radius;
    return kSuccess;
  }

  static ErrorType GetRosPolygonFromPolygon(const common::Polygon &poly,
                                            geometry_msgs::Polygon *msg) {
    for (const auto p : poly.points) {
      geometry_msgs::Point32 pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      msg->points.push_back(pt);
    }
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoFromSimulatorData(
      const common::LaneNet &lane_net, const common::VehicleSet &vehicle_set,
      const common::ObstacleSet &obstacle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ArenaInfo *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoStaticFromSimulatorData(
      const common::LaneNet &lane_net, const common::ObstacleSet &obstacle_set,
      const ros::Time &timestamp, const std::string &frame_id,
      vehicle_msgs::ArenaInfoStatic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoDynamicFromSimulatorData(
      const common::VehicleSet &vehicle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ArenaInfoDynamic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    return kSuccess;
  }

  static ErrorType GetRosControlSignalFromControlSignal(
      const common::VehicleControlSignal &ctrl, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ControlSignal *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->acc = ctrl.acc;
    msg->steer_rate = ctrl.steer_rate;
    msg->is_openloop.data = ctrl.is_openloop;
    GetRosStateMsgFromState(ctrl.state, timestamp, &msg->state);
    return kSuccess;
  }

  static ErrorType GetBehaviorFromSemanticBehaviorData(
      const common::SemanticBehavior &behavior, const ros::Time &timestamp, 
      const std::string &frame_id, vehicle_msgs::Behavior *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;

    for (const vec_E<common::Vehicle> b: behavior.forward_trajs) {
      vehicle_msgs::VehicleTraj vtraj;
      GetRosVehicleTrajFromVehicleVector(b, timestamp, frame_id, &vtraj);
      msg->forward_trajs.push_back(vtraj);
    }

    for (const std::unordered_map<int, vec_E<common::Vehicle>> st : behavior.surround_trajs) {
      vehicle_msgs::SurroundTraj straj;
      GetRosSurroundTrajFromTraj(st, timestamp, frame_id, &straj);
      msg->surround_trajs.push_back(straj);
    }

    GetRosStateMsgFromState(behavior.state, timestamp, frame_id, &msg->state);

    for (const auto fb : behavior.forward_behaviors) {
      msg->forward_behaviors.push_back(static_cast<int>(fb));
    }

    msg->actual_desired_velocity = behavior.actual_desired_velocity;
    msg->lat_behavior = static_cast<int>(behavior.lat_behavior);
    msg->lon_behavior = static_cast<int>(behavior.lon_behavior);

    return kSuccess;
  }

  static ErrorType GetCloudInfoFromSemanticBehaviorData(
      const common::SemanticBehavior &behavior, const ros::Time &timestamp, 
      const std::string &frame_id, const std::string &uid, vehicle_msgs::CloudInfo *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetBehaviorFromSemanticBehaviorData(behavior, timestamp, frame_id, &msg->behavior);
    msg->uid.data = uid;
    return kSuccess;
  }

  static ErrorType GetTrajUpdateInfoFromStateTrajData(
      const vec_E<common::State> &traj, const std::string &cloud_behavior_uid, const int &vehicle_id,
      const ros::Time &timestamp, const std::string &frame_id, vehicle_msgs::TrajUpdateInfo *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id.data = vehicle_id;
    for (const common::State &s:traj) {
      vehicle_msgs::State state;
      GetRosStateMsgFromState(s, timestamp, frame_id, &state);
      msg->states.push_back(state);
    }
    msg->uid.data = cloud_behavior_uid;
    return kSuccess;
  }

  static ErrorType GetTwistInfoFromControlState(const common::State &state, 
                                                geometry_msgs::Twist *msg) {
    msg->linear.x = state.velocity;
    msg->angular.z = state.curvature * state.velocity;
    return kSuccess;
  }

  static ErrorType GetQuaternionFromControlState(const common::State &state, 
                                                geometry_msgs::Quaternion *msg) {
    decimal_t pitch = 0.0;
    decimal_t roll = 0.0;
    decimal_t yaw = state.angle;
    decimal_t cy = cos(yaw * 0.5);
    decimal_t sy = sin(yaw * 0.5);
    decimal_t cp = cos(pitch * 0.5);
    decimal_t sp = sin(pitch * 0.5);
    decimal_t cr = cos(roll * 0.5);
    decimal_t sr = sin(roll * 0.5);

    msg->w = cy * cp * cr + sy * sp * sr;
    msg->x = cy * cp * sr - sy * sp * cr;
    msg->y = sy * cp * sr + cy * sp * cr;
    msg->z = sy * cp * cr - cy * sp * sr;
    return kSuccess;
  }

  static ErrorType transformToCurrentState(const common::State& traj_state,
                                        const common::State& cur_state, KDL::Frame* F) {
    geometry_msgs::Quaternion traj_quaternion;
    geometry_msgs::Quaternion cur_quaternion;
    decimal_t z = 0;
    GetQuaternionFromControlState(traj_state, &traj_quaternion);
    GetQuaternionFromControlState(cur_state, &cur_quaternion);

    KDL::Frame F_traj(KDL::Rotation::Quaternion(traj_quaternion.x,
                                                  traj_quaternion.y,
                                                  traj_quaternion.z,
                                                  traj_quaternion.w),
                        KDL::Vector(traj_state.vec_position[0],
                                    traj_state.vec_position[1],
                                    z));

  // Robot (base_link) in global (map) frame
    KDL::Frame F_cur(KDL::Rotation::Quaternion(cur_quaternion.x,
                                                  cur_quaternion.y,
                                                  cur_quaternion.z,
                                                  cur_quaternion.w),
                        KDL::Vector(cur_state.vec_position[0],
                                    cur_state.vec_position[1],
                                    z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

    *F = F_cur.Inverse()*F_traj; 
  }

  // static ErrorType GetVelocity(const common::State& traj_state,
  //                               const common::State& cur_state, geometry_msgs::Twist *msg) {
  //   geometry_msgs::TransformStamped lookahead_;
  //   KDL::Frame *F_bl_ld;
  //   decimal_t ld_ = 1.0;
  //   if (sqrt(pow(traj_state.vec_position[0] - cur_state.vec_position[0],2) + pow(traj_state.vec_position[1] - cur_state.vec_position[1],2)) > ld_) {
  //     transformToCurrentState(traj_state, cur_state, F_bl_ld);
  //     lookahead_.transform.translation.x = F_bl_ld->p.x();
  //     lookahead_.transform.translation.y = F_bl_ld->p.y();
  //     lookahead_.transform.translation.z = F_bl_ld->p.z();
  //     F_bl_ld->M.GetQuaternion(lookahead_.transform.rotation.x,
  //                             lookahead_.transform.rotation.y,
  //                             lookahead_.transform.rotation.z,
  //                             lookahead_.transform.rotation.w);
  //   }

  //   KDL::Frame *F_bl_end;
  //   transformToCurrentState(traj_state, cur_state, F_bl_end);
  //   double roll, pitch, yaw;
  //   F_bl_end->M.GetRPY(roll, pitch, yaw);
  //   double k_end = tan(yaw); // Slope of line defined by the last path pose
  //   double l_end = F_bl_end->p.y() - k_end * F_bl_end->p.x();
  //   double a = 1 + k_end * k_end;
  //   double b = 2 * l_end;
  //   double c = l_end * l_end - ld_ * ld_;
  //   double D = sqrt(b*b - 4*a*c);
  //   double x_ld = (-b + D) / (2*a);
  //   double y_ld = k_end * x_ld + l_end;
        
  //   lookahead_.transform.translation.x = x_ld;
  //   lookahead_.transform.translation.y = y_ld;
  //   lookahead_.transform.translation.z = F_bl_end->p.z();
  //   F_bl_end->M.GetQuaternion(lookahead_.transform.rotation.x,
  //                                lookahead_.transform.rotation.y,
  //                                lookahead_.transform.rotation.z,
  //                                lookahead_.transform.rotation.w);
  //   double yt = lookahead_.transform.translation.y;
  //   double ld_2 = ld_ * ld_;
  //   decimal_t wheel_base = 0.05;
  //   decimal_t steer = atan2(2 * yt * wheel_base, ld_2);
  //   decimal_t curvature = tan(steer) * 1.0 / wheel_base;
  //   msg->linear.x = traj_state.velocity;
  //   msg->angular.z = curvature * msg->linear.x;
  // }

  
};  // class Encoder

}  // namespace vehicle_msgs

#endif  // _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__