/**
 * @file visualizer.h
 * @author Yijun Mao
 * @brief
 * @version 0.1
 * @date 2022-06-08
 *
 * @copyright Copyright (c) 2022
 */
#ifndef _UTIL_PHY_ENV_MANAGER_INC_PHY_ENV_MANAGER_VISUALIZER_H_
#define _UTIL_PHY_ENV_MANAGER_INC_PHY_ENV_MANAGER_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "phy_env_manager/phy_manager.h"

namespace phy_env_manager {

class Visualizer {
 public:
  Visualizer() {}
  Visualizer(ros::NodeHandle nh);
  ~Visualizer() {}

  void set_phy_manager(PhyManager *p_phy_manager) { p_phy_manager_ = p_phy_manager; }

  void VisualizeData();
  void VisualizeDataWithStamp(const ros::Time &stamp);
  void SendTfWithStamp(const ros::Time &stamp);

 private:
  void VisualizeVehicleSet(const ros::Time &stamp,
                           const common::VehicleSet &vehicle_set);
  void VisualizeLaneNet(const ros::Time &stamp,
                        const common::LaneNet &lane_net);
  void VisualizeObstacleSet(const ros::Time &stamp,
                            const common::ObstacleSet &Obstacle_set);


  ros::NodeHandle nh_;

  ros::Publisher vehicle_set_pub_;
  ros::Publisher lane_net_pub_;
  ros::Publisher obstacle_set_pub_;

  PhyManager *p_phy_manager_;
};  // Visualizer

}  // namespace phy_env_manager

#endif  // _UTIL_PHY_ENV_MANAGER_INC_PHY_ENV_MANAGER_VISUALIZER_H_