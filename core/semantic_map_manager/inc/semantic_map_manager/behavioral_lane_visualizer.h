#ifndef _CORE_SEMANTIC_MAP_INC_BEHAVIORAL_LANE_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_BEHAVIORAL_LANE_VISUALIZER_H_

#include <assert.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/lane/lane.h"
#include "common/visualization/common_visualization_util.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

class BehavioralLaneVisualizer {
 public:

  BehavioralLaneVisualizer(ros::NodeHandle nh): nh_(nh) {
		std::string behavioral_lane_topic = std::string("/vis/behavioral_lanes");
		behavioral_lane_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      behavioral_lane_topic, 1);
	}
  ~BehavioralLaneVisualizer() {}

  void VisualizeDataWithStamp(const ros::Time &stamp,
                              const std::vector<common::Lane> &ref_lanes,
                              const std::vector<bool> &cur_pass_dir_straights) {
		VisualizeLaneWithDirection(stamp, ref_lanes, cur_pass_dir_straights);
	}
 private:
  void VisualizeLaneWithDirection(const ros::Time &stamp,
																	const std::vector<common::Lane> &ref_lanes,
																	const std::vector<bool> &cur_pass_dir_straights) {
		visualization_msgs::MarkerArray behavioral_lane_marker_arr;

		for (int i=0; i<ref_lanes.size(); i++) {
			common::VisualizationUtil::GetRosMarkerArrUsingLaneWithDirection(
			ref_lanes[i], cur_pass_dir_straights[i], &behavioral_lane_marker_arr);
		}
		
		common::VisualizationUtil::FillStampInMarkerArray(stamp, &behavioral_lane_marker_arr);

		int num_markers = static_cast<int>(behavioral_lane_marker_arr.markers.size());
		common::VisualizationUtil::FillHeaderIdInMarkerArray(
				stamp, std::string("map"), last_behavioral_lane_marker_cnt_,
				&behavioral_lane_marker_arr);
		last_behavioral_lane_marker_cnt_ = num_markers;

		behavioral_lane_pub_.publish(behavioral_lane_marker_arr);
	}


  ros::NodeHandle nh_;

	int last_behavioral_lane_marker_cnt_;

  ros::Publisher behavioral_lane_pub_;
};  // Visualizer

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_BEHAVIORAL_LANE_VISUALIZER_H_