#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/decoder.h"
#include "vehicle_msgs/Obstacle.h"
#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"

#include "phy_env_manager/phy_manager.h"
#include "phy_env_manager/ros_adapter.h"
#include "phy_env_manager/visualizer.h"

using namespace phy_env_manager;

DECLARE_BACKWARD;
const double simulation_rate = 500.0;
const double gt_msg_rate = 100.0;
const double gt_static_msg_rate = 10.0;
const double visualization_msg_rate = 20.0;

struct EulerAngle{
  double roll, pitch, yaw;
};

// 四元数转欧拉角
static EulerAngle toEulerAngle(const double x, const double y, const double z, const double w) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (w * x + y * z);
  double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (w * y - z * x);
  double pitch = 0;
  if (fabs(sinp) >= 1)
      pitch = copysign(kPi / 2, sinp); // use 90 degrees if out of range
  else
      pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (w * z + x * y);
  double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(siny_cosp, cosy_cosp);
  //    return yaw;
  EulerAngle res = {
    res.roll = roll < 0 ? roll + 2 * kPi : roll,
    res.pitch = pitch < 0 ? pitch + 2* kPi : pitch,
    res.yaw = yaw < 0 ? yaw + 2* kPi : yaw
  };
  return res;
}

// common::VehicleControlSignalSet _signal_set;
std::unordered_map<int, common::State> _state_set;
std::vector<ros::Subscriber> _ros_sub;

Vec3f initial_state(0, 0, 0);
bool flag_rcv_initial_state = false;

Vec3f goal_state(0, 0, 0);
bool flag_rcv_goal_state = false;

void OdomCallback(const nav_msgs::OdometryConstPtr &msg,
                  int index) {
  common::State state;
  geometry_msgs::Quaternion cur_orientation = msg->pose.pose.orientation;
  state.vec_position[0] = msg->pose.pose.position.x;
  state.vec_position[1] = msg->pose.pose.position.y;
  state.angle = toEulerAngle(cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w).yaw;
  
  // if (!_inits.at(index)) {
  //   _inits.at(index) = true;
  //   state.velocity = 0.4;
  // }
  // else {
  state.velocity = sqrt(msg->twist.twist.linear.x*msg->twist.twist.linear.x+msg->twist.twist.linear.y*msg->twist.twist.linear.y);
  if(state.velocity == 0){
    state.curvature = msg->twist.twist.angular.z;
  } 
  else {
    state.curvature = msg->twist.twist.angular.z/state.velocity;
  }
  
  //}
  state.steer = atan(state.curvature*0.05);
  // state.steer = 0;
  _state_set.at(index) = state;
}

void ObstacleCallback(const vehicle_msgs::ObstacleConstPtr &msg,
                  int index) {
  common::State state;
  state.vec_position[0] = msg->x;
  state.vec_position[1] = msg->y;
  state.angle = msg->angle;
  
  // if (!_inits.at(index)) {
  //   _inits.at(index) = true;
  //   state.velocity = 0.4;
  // }
  // else {
  state.velocity = msg->velocity;
  state.curvature = 0;
  //}
  state.steer = atan(state.curvature*0.05);
  // state.steer = 0;
  _state_set.at(index) = state;
}

// void CtrlSignalCallback(const vehicle_msgs::ControlSignal::ConstPtr& msg,
//                         int index) {
//   common::VehicleControlSignal ctrl;
//   vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
//   _signal_set.signal_set[index] = ctrl;
// }

// void InitialPoseCallback(
//     const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
//   common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose,
//                                                      &initial_state);
//   flag_rcv_initial_state = true;
// }

// void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//   common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
//   flag_rcv_goal_state = true;
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string vehicle_info_path;
  if (!nh.getParam("vehicle_info_path", vehicle_info_path)) {
    ROS_ERROR("Failed to get param %s", vehicle_info_path.c_str());
    assert(false);
  }
  std::string map_path;
  if (!nh.getParam("map_path", map_path)) {
    ROS_ERROR("Failed to get param %s", map_path.c_str());
    assert(false);
  }
  std::string lane_net_path;
  if (!nh.getParam("lane_net_path", lane_net_path)) {
    ROS_ERROR("Failed to get param %s", lane_net_path.c_str());
    assert(false);
  }
  double management_rate;
  if (!nh.getParam("management_rate", management_rate)) {
    ROS_ERROR("Failed to get param management_rate");
    assert(false);
  }

  PhyManager phy_manager(vehicle_info_path, map_path, lane_net_path);

  RosAdapter ros_adapter(nh);
  ros_adapter.set_phy_manager(&phy_manager);

  Visualizer visualizer(nh);
  visualizer.set_phy_manager(&phy_manager);

  auto vehicle_ids = phy_manager.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    std::string topic_name = std::string("/localization_") + std::to_string(vehicle_id);
    printf("subscribing to %s\n", topic_name.c_str());
    _ros_sub[i] = nh.subscribe<nav_msgs::Odometry>(
        topic_name, 10, boost::bind(OdomCallback, _1, vehicle_id));
    // std::string obstacle_topic_name = std::string("/obstacle_") + std::to_string(vehicle_id);
    // printf("subscribing to %s\n", obstacle_topic_name.c_str());
    // _ros_sub[i] = nh.subscribe<vehicle_msgs::Obstacle>(
    //     obstacle_topic_name, 10, boost::bind(ObstacleCallback, _1, vehicle_id));
  }

  for (auto& vehicle_id : vehicle_ids) {
    common::State default_state;
    _state_set.insert(std::pair<int, common::State>(
        vehicle_id, default_state));
  }

  ros::Rate rate(management_rate);
  ros::Time next_gt_pub_time = ros::Time::now();
  ros::Time next_gt_static_pub_time = next_gt_pub_time;
  ros::Time next_vis_pub_time = ros::Time::now();

  std::cout << "[PhyManager] Initialization finished, waiting for callback"
            << std::endl;

  int gt_msg_counter = 0;
  while (ros::ok()) {
    ros::spinOnce();

    phy_manager.UpdateVehicleSetUsingStateSet(_state_set);

    ros::Time tnow = ros::Time::now();
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += ros::Duration(1.0 / gt_msg_rate);
      ros_adapter.PublishDynamicDataWithStamp(tnow); // Tips: 这里发送所有其余agv的位置信息
    }

    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += ros::Duration(1.0 / gt_static_msg_rate);
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }

    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += ros::Duration(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    rate.sleep();
  }

  _ros_sub.clear();
  ros::shutdown();
  return 0;
}
