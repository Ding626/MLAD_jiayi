/**
 * @file test_ssc_with_eudm.cc
 * @author HKUST Aerial Robotics Group
 * @brief test ssc planner with eudm behavior planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"

DECLARE_BACKWARD;
double bp_work_rate = 20.0;

planning::EudmPlannerServer* p_bp_server_{nullptr};

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param agent_config_path %s",
              agent_config_path.c_str());
    assert(false);
  }

  std::string lane_config_path;
  if (!nh.getParam("lane_net_path", lane_config_path)) {
    ROS_ERROR("Failed to get param lane_config_path %s",
              lane_config_path.c_str());
    assert(false);
  }

  std::string bp_config_path;
  if (!nh.getParam("bp_config_path", bp_config_path)) {
    ROS_ERROR("Failed to get param bp_config_path %s", bp_config_path.c_str());
    assert(false);
  }

  decimal_t uncertain_avoid_replan_interval;
  if (!nh.getParam("uncertain_avoidance_replan_interval", uncertain_avoid_replan_interval)) {
    ROS_ERROR("Failed to get param uncertain_avoid_replan_interval %d", uncertain_avoid_replan_interval);
    uncertain_avoid_replan_interval = 10.0;
  }

  decimal_t change_lane_threshold;
  if (!nh.getParam("change_lane_threshold", change_lane_threshold)) {
    ROS_ERROR("Failed to get param change_lane_threshold %d", change_lane_threshold);
    change_lane_threshold = 0.1;
  }

  decimal_t max_wait_update_time;
  if (!nh.getParam("max_wait_update_time", max_wait_update_time)) {
    ROS_ERROR("Failed to get param max_wait_update_time %d", max_wait_update_time);
    max_wait_update_time = 0.2;
  }

  semantic_map_manager::SemanticMapManager semantic_map_manager(
      nh, agent_config_path, lane_config_path, uncertain_avoid_replan_interval, change_lane_threshold, max_wait_update_time);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  // smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

  // Declare bp
  p_bp_server_ = new planning::EudmPlannerServer(nh, &semantic_map_manager, bp_work_rate);

  p_bp_server_->Init(bp_config_path);
  smm_ros_adapter.Init();

  p_bp_server_->Start();

  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
