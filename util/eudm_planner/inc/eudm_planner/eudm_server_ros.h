#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#include <sensor_msgs/Joy.h>

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/basics/id_queue.h"
#include "common/visualization/common_visualization_util.h"
#include "common/planning/dcp_tree.h"
#include "common/planning/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "eudm_planner/cloud_info_publisher.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "ros/ros.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {
class EudmPlannerServer { // TODO for MLAD: DCP树的生成语义动作，需要改成每个自由路径中的车辆都有单独一个类，还是可以公用？ 
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = common::DcpTree::DcpAction;
  using DcpLonAction = common::DcpTree::DcpLonAction;
  using DcpLatAction = common::DcpTree::DcpLatAction;

  struct Config {
    int kInputBufferSize{100};
  };

  EudmPlannerServer(ros::NodeHandle nh, SemanticMapManager *smm);

  EudmPlannerServer(ros::NodeHandle nh, SemanticMapManager *smm, double work_rate);

  void PushSemanticMap(const SemanticMapManager &smm); 

  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);
  /**
   * @brief set desired velocity
   */
  void set_user_desired_velocity(const decimal_t desired_vel); // TODO for MLAD: 后续加在SemanticMapManager里？

  decimal_t user_desired_velocity() const;
  common::Task task;

  void Init(const std::string &bp_config_path);

  void Start();

 private:
  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  void Replan();

  void PublishData();

  void MainThread(); // MainThread改为检测id queue是否有id？

  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  Config config_;

  int cur_ego_id_;

  EudmManager bp_manager_;
  EudmPlannerVisualizer *p_visualizer_;

  SemanticMapManager *smm_; // 全局唯一的语义地图指针。
  // TODO for MLAD: 启动时，一定要语义地图的部分先启动；结束时，也需要语义地图部分最后结束

  common::Task task_; // ! Deprecate
  // TODO for MLAD: 决定好变道后，新实例化一个Task
  bool use_sim_state_ = true;
  // bool task_updated_ = false;
  // ros related
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_; 
  // NOTE for MLAD: 需要在这里写一个发送msg的
  EudmPlannerPublisher *p_cloud_info_pubs_;

  double work_rate_{20.0};
  int ego_id_; // ! Deprecate

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_; // ! Deprecate NOTE for MLAD: 现在不使用这种方式来解决语义地图线程和当前规划线程规划的问题 

  bool has_callback_binded_ = false;// ! Deprecate
  std::function<int(const SemanticMapManager &)> private_callback_fn_;// ! Deprecate

  common::PriorityIdQueue *id_queue;
};

}  // namespace planning

#endif