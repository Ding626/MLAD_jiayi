#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <iostream>
#include <vector>
#include <unordered_map>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

class EudmPlannerVisualizer {
 public:
  EudmPlannerVisualizer(ros::NodeHandle nh, EudmManager* p_bp_manager) 
      : nh_(nh) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  EudmPlannerVisualizer(ros::NodeHandle nh, EudmManager* p_bp_manager, const int &max_num)
      : nh_(nh), max_num_(max_num) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  void Init() {
    for (int i=0; i<max_num_; i++) {
      std::string forward_traj_topic = std::string("/vis/agent_") +
                                      std::to_string(i) +
                                      std::string("/forward_trajs");

      publishers_.insert(std::make_pair(i, 
                        nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic, 1))); 
    }
    
    // std::string forward_traj_topic1 = std::string("/vis/agent_") +
    //                                  std::to_string(0) +
    //                                  std::string("/forward_trajs");
    // // TODO for MLAD: 後續根據所有的key vehicle，爲所有都設置一個pub函數
    // forward_traj_vis_pub_1 =
    //     nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic1, 1);
    
    // std::string forward_traj_topic2 = std::string("/vis/agent_") +
    //                                  std::to_string(1) +
    //                                  std::string("/forward_trajs");
    // // TODO for MLAD: 後續根據所有的key vehicle，爲所有都設置一個pub函數
    // forward_traj_vis_pub_2 =
    //     nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic2, 1);
  }

  void PublishDataWithStamp(const ros::Time& stamp, const int &cur_ego_id) {
    VisualizeForwardTrajectories(stamp, cur_ego_id);
  }

  void VisualizeForwardTrajectories(const ros::Time& stamp, const int &cur_ego_id) {
    auto forward_trajs = p_bp_manager_->planner().forward_trajs();
    int processed_winner_id = p_bp_manager_->processed_winner_id();
    int original_winner_id = p_bp_manager_->original_winner_id();
    visualization_msgs::MarkerArray traj_list_marker;
    common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);
    double traj_z = 0.3;
    for (int i = 0; i < static_cast<int>(forward_trajs.size()); ++i) {
      if (i == processed_winner_id) {
        traj_color = common::cmap.at("gold");
        traj_z = 0.4;
      } else if (i == original_winner_id) {
        traj_color = common::cmap.at("spring green");
        traj_z = 0.4;
      } else {
        traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);
        traj_z = 0.3;
      }
      std::vector<common::Point> points;
      for (const auto& v : forward_trajs[i]) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        // pt.z = v.state().time_stamp -
        // forward_trajs[i].front().state().time_stamp;
        pt.z = traj_z;
        points.push_back(pt);
        visualization_msgs::Marker point_marker;
        // point_marker.ns = "point";
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      visualization_msgs::Marker line_marker;
      // line_marker.ns = "line";
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;

    if (publishers_.count(cur_ego_id)==0) {
      //新建此id对应的publisher
      std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(cur_ego_id) +
                                     std::string("/forward_trajs");

      publishers_.insert(std::make_pair(cur_ego_id, 
                        nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic, 1))); 
    }
    publishers_.at(cur_ego_id).publish(traj_list_marker);
    
  }

  void set_use_sim_state(bool use_sim_state) { use_sim_state_ = use_sim_state; }

 private:
  ros::NodeHandle nh_;
  int max_num_{0};
  
  bool use_sim_state_ = true;

  int last_forward_trajs_marker_cnt_ = 0;

  std::unordered_map<int, ros::Publisher> publishers_;

  EudmManager* p_bp_manager_{nullptr};
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_