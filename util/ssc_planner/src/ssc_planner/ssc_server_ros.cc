/**
 * @file ssc_server.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner server
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_server_ros.h"

#include <glog/logging.h>

namespace planning
{

  SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, int ego_id)
      : nh_(nh), work_rate_(20.0), ego_id_(ego_id)
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
    p_ssc_vis_ = new SscVisualizer(nh, ego_id);
  }

  SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id)
      : nh_(nh), work_rate_(work_rate), ego_id_(ego_id), use_sim_state_(true)
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
    p_ssc_vis_ = new SscVisualizer(nh, ego_id);
  }

  SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id, bool use_sim_state)
      : nh_(nh), work_rate_(work_rate), ego_id_(ego_id), use_sim_state_(use_sim_state)
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
    p_ssc_vis_ = new SscVisualizer(nh, ego_id);
  }

  void SscPlannerServer::PushSemanticMap(const SemanticMapManager &smm)
  {
    if (p_input_smm_buff_)
      p_input_smm_buff_->try_enqueue(smm);
  }

  void SscPlannerServer::PublishTrajectory()
  {
    if (!is_replan_on_)
      return;

    if (!is_map_updated_)
      return;

    using common::VisualizationUtil;
    auto current_time = ros::Time::now().toSec();

    // ssc visualization
    {
      TicToc timer;
      p_ssc_vis_->VisualizeDataWithStamp(ros::Time(current_time), planner_);
      LOG(ERROR) << "[SscPlannerServer]ssc vis all time cost: " << timer.toc() << " ms";
    }

    if (executing_traj_ == nullptr || !executing_traj_->IsValid())
      return;

    // trajectory update
    {
      vec_E<common::State> traj_in_states;
      for (decimal_t s = executing_traj_->begin(); s < executing_traj_->end(); s += 0.05)
      {
        common::State state;
        if (executing_traj_->GetState(s, &state) == kSuccess)
        {
          traj_in_states.push_back(state);
        }
      }
      if (!traj_in_states.empty())
      {
        vehicle_msgs::TrajUpdateInfo msg;
        vehicle_msgs::Encoder::GetTrajUpdateInfoFromStateTrajData(
            traj_in_states, last_smm_.cloud_behavior_uid(), ego_id_, ros::Time::now(), "updated trajectory", &msg);
        executing_traj_update_pub_.publish(msg);
      }
    }

    // trajectory visualization
    {
      auto color = common::cmap["magenta"];
      if (require_intervention_signal_)
        color = common::cmap["yellow"];
      visualization_msgs::MarkerArray traj_mk_arr;
      common::VisualizationUtil::GetMarkerArrayByTrajectory(
          *executing_traj_, 0.15, Vecf<3>(0.3, 0.3, 0.3), color, 0.5,
          &traj_mk_arr);
      if (require_intervention_signal_)
      {
        visualization_msgs::Marker traj_status;
        common::State state_begin;
        executing_traj_->GetState(executing_traj_->begin(), &state_begin);
        Vec3f pos = Vec3f(state_begin.vec_position[0],
                          state_begin.vec_position[1], 5.0);
        common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
            pos, std::string("Intervention Needed!"), common::cmap["red"],
            Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
        traj_mk_arr.markers.push_back(traj_status);
      }
      int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
      common::VisualizationUtil::FillHeaderIdInMarkerArray(
          ros::Time(current_time), std::string("map"), last_trajmk_cnt_,
          &traj_mk_arr);
      last_trajmk_cnt_ = num_traj_mks;
      executing_traj_vis_pub_.publish(traj_mk_arr);
    }
  }

  void SscPlannerServer::PublishCtrlCommand()
  {
    if (!is_replan_on_)
      return;

    if (!is_map_updated_)
      return;

    geometry_msgs::Twist ctrl_cmd_msg;
    common::State state;

    vehicle_msgs::ControlSignal ctrl_msg;

    if (executing_traj_ == nullptr || !executing_traj_->IsValid())
    {
      ctrl_cmd_pub_.publish(ctrl_cmd_msg);
      LOG(ERROR) << "[SscPlannerServer] Publish control command (linear.x: " << ctrl_cmd_msg.linear.x
                << ", angular.z: " << ctrl_cmd_msg.angular.z << ")";
      return;
    }

    auto current_time = ros::Time::now().toSec();

    if (current_time >= executing_traj_->begin() && current_time <= executing_traj_->end() && !require_intervention_signal_)
    {
      // decimal_t plan_horizon = 1.0 / work_rate_;
      // int num_cycles = std::floor((current_time - executing_traj_->begin()) / plan_horizon);
      // decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon;
      if (executing_traj_->GetState(current_time, &state) == kSuccess)
      {
        FilterSingularityState(ctrl_state_hist_, &state); // TODO for MLAD: 这里这个也是需要的，实际运行的时候
        ctrl_state_hist_.push_back(state);
        if (ctrl_state_hist_.size() > 100)
          ctrl_state_hist_.erase(ctrl_state_hist_.begin());

        vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
            common::VehicleControlSignal(state), ros::Time(current_time),
            std::string("map"), &ctrl_msg);

        vehicle_msgs::Encoder::GetTwistInfoFromControlState(state, &ctrl_cmd_msg);
      }
    }

    ctrl_signal_pub_.publish(ctrl_msg);
    LOG(ERROR) << "[SscPlannerServer] publish ControlSignal <x,y,v,acc,angle,steer,curvature>:<" 
              << state.vec_position[0] << ", " 
              << state.vec_position[1] << ", " 
              << state.velocity << ", "
              << state.acceleration << ", "
              << state.angle << ", "
              << state.steer << ", "
              << state.curvature
              << ">";
    ctrl_cmd_pub_.publish(ctrl_cmd_msg);
    LOG(ERROR) << "[PublishCtrlCommand] Publish control command (linear.x: " << ctrl_cmd_msg.linear.x
              << ", angular.z: " << ctrl_cmd_msg.angular.z << ")";
    return;
  }

  void SscPlannerServer::PublishData()
  {
    if (!is_map_updated_)
      return;

    using common::VisualizationUtil;
    auto current_time = ros::Time::now().toSec();

    // smm visualization
    {
      // NOTE for MLAD: 这部分应该是在ssc规划前，展示
      p_smm_vis_->VisualizeDataWithStamp(ros::Time(current_time), last_smm_);
      p_smm_vis_->SendTfWithStamp(ros::Time(current_time), last_smm_);
    }

    // trajectory feedback
    {
      if (executing_traj_ == nullptr || !executing_traj_->IsValid())
        return;

      if (use_sim_state_)
      {
        LOG(ERROR) << "[SscPlannerServer]use_sim_state_: true. \n";
        decimal_t lf_dis = 0.7;
        decimal_t plan_horizon = 1.0 / work_rate_; 
        int num_cycles =
             std::floor((current_time - executing_traj_->begin()) / plan_horizon);
        decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon; // TODO for MLAD: 仿真时不需要+0.9，实机跑时需要，取未来的路径点
        if(ct > executing_traj_->end()) {
          ct = executing_traj_->end();
        }
        {
          common::State state;
          if (executing_traj_->GetState(ct, &state) == kSuccess)
          {
            FilterSingularityState(ctrl_state_hist_, &state); // TODO for MLAD: 这里这个也是需要的，实际运行的时候
            ctrl_state_hist_.push_back(state);
            if (ctrl_state_hist_.size() > 100)
              ctrl_state_hist_.erase(ctrl_state_hist_.begin());
            vehicle_msgs::ControlSignal ctrl_msg;
            vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
                common::VehicleControlSignal(state), ros::Time(ct),
                std::string("map"), &ctrl_msg);
            ctrl_signal_pub_.publish(ctrl_msg);
            LOG(ERROR) << "[SscPlannerServer] publish ControlSignal <x,y,v,acc,angle,steer,curvature>:<" 
                      << state.vec_position[0] << ", " 
                      << state.vec_position[1] << ", " 
                      << state.velocity << ", "
                      << state.acceleration << ", "
                      << state.angle << ", "
                      << state.steer << ", "
                      << state.curvature
                      << ">";
            // decimal_t wheel_base = 0.5;
            // decimal_t cur_to_dest_angle =
            //     vec2d_to_angle(state.vec_position - last_smm_.ego_vehicle().state().vec_position);
            // decimal_t angle_diff =
            //     cur_to_dest_angle - last_smm_.ego_vehicle().state().angle;
            // decimal_t steer = atan2(2.0 * wheel_base * sin(angle_diff), lf_dis);
            // decimal_t curvature = tan(steer) * 1.0 / wheel_base;
            // geometry_msgs::Twist ctrl_cmd_msg;
            // geometry_msgs::Twist ctrl_cmd_msg_2;
            // vehicle_msgs::Encoder::GetTwistInfoFromControlState(state, &ctrl_cmd_msg);
            // if(state.velocity < 0.4) {
            //   ctrl_cmd_msg_2.linear.x = state_1.velocity;
            // } 
            // else ctrl_cmd_msg_2.linear.x = 0.4;
            // ctrl_cmd_msg_2.angular.z = curvature * ctrl_cmd_msg_2.linear.x;
            // if(ctrl_cmd_msg_2.angular.z < 0.01 && ctrl_cmd_msg_2.angular.z > -0.01) {
            //   ctrl_cmd_msg_2.angular.z = 0;
            // }
            // if(ctrl_cmd_msg_2.angular.z > 0.01 && ctrl_cmd_msg_2.angular.z < 0.03) {
            //   ctrl_cmd_msg_2.angular.z = 0.03;
            // }
            // else if(ctrl_cmd_msg_2.angular.z > -0.03 && ctrl_cmd_msg_2.angular.z < -0.01) {
            //   ctrl_cmd_msg_2.angular.z = -0.03;
            // }
            // ctrl_cmd_pub_.publish(ctrl_cmd_msg_2);
            // printf("[PublishCtrlCommand] Publish control command (linear.x: %lf, angular.z: %lf)\n",
            //        ctrl_cmd_msg.linear.x, ctrl_cmd_msg.angular.z);
            // printf("[PublishCtrlCommand] Publish control command (linear.x: %lf, angular.z: %lf)\n",
            //        ctrl_cmd_msg_2.linear.x, ctrl_cmd_msg_2.angular.z);
          }
          else
          {
            LOG(ERROR) << "[SscPlannerServer]cannot evaluate state at " << ct << " with begin "
                      << executing_traj_->begin();
          }
        }
      }
      else
      {
        LOG(ERROR) << "[SscPlannerServer]use_sim_state_: false.";
      }
    }
  }

  ErrorType SscPlannerServer::FilterSingularityState(
      const vec_E<common::State> &hist, common::State *filter_state)
  {
    if (hist.empty())
    {
      return kWrongStatus;
    }
    decimal_t duration = filter_state->time_stamp - hist.back().time_stamp;
    decimal_t wheel_base = 2.85;
    decimal_t max_steer = M_PI / 4.0;
    decimal_t singular_velocity = kBigEPS;
    decimal_t max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
    decimal_t max_orientation_change = max_orientation_rate * duration;

    if (fabs(filter_state->velocity) < singular_velocity &&
        fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
            max_orientation_change) {
      LOG(ERROR) << "[SscPlannerServer]Detect singularity velocity "<< filter_state->velocity 
                << ", angle (" << hist.back().angle << ", " << filter_state->angle <<").";
      filter_state->angle = hist.back().angle;
      LOG(ERROR) << "[SscPlannerServer]Filter angle to " << hist.back().angle;
    }
    return kSuccess;
  }

  void SscPlannerServer::Init(const std::string &config_path)
  {
    planner_.Init(config_path);

    std::string traj_topic = std::string("/vis/agent_") +
                             std::to_string(ego_id_) +
                             std::string("/ssc/exec_traj");

    std::string traj_update_info_topic = std::string("/local/agent_") +
                                         std::to_string(ego_id_) +
                                         std::string("/traj_update_info_topic");
    // nh_.param("use_sim_state", use_sim_state_, true);

    ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20); // NOTE for MLAD: 在roslauch文件中，对ctrl这个topic进行了remap，这里不用再加agv id
    map_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("ssc_map", 1);
    executing_traj_vis_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(traj_topic, 1);
    executing_traj_update_pub_ =
        nh_.advertise<vehicle_msgs::TrajUpdateInfo>(traj_update_info_topic, 1);

    google::InitGoogleLogging(std::string("ssc_").append(std::to_string(ego_id_)).c_str());
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::SetLogDestination(google::GLOG_INFO, 
                std::string("/home/pc/MLAD/src/Log/ssc_").append(std::to_string(ego_id_) + std::string("/info_")).c_str());
    google::InstallFailureSignalHandler();                             //!< Capture SIGSEGV info and out to stderr
    google::SetLogFilenameExtension(".log");  //在日志文件名中级别后添加一个扩展名。适用于所有严重级别
    FLAGS_logbufsecs = 0;  //设置实时输出日志，实时刷新(必须设置为0)
    FLAGS_max_log_size = 2048;  //max_log_size：1024MB 设置日志记录文件最大大小，MB为单位，默认值为1800，当前超过当前大小，则保存剩余数据至文件，并创建新的文件保存其他日志信息；
  }

  void SscPlannerServer::Start()
  {
    if (is_replan_on_)
    {
      return;
    }
    planner_.set_map_interface(&map_adapter_);
    LOG(ERROR) << "[SscPlannerServer]Planner server started.";
    is_replan_on_ = true;

    std::thread(&SscPlannerServer::MainThread, this).detach();
  }

  void SscPlannerServer::MainThread()
  {
    using namespace std::chrono;
    system_clock::time_point current_start_time{system_clock::now()};
    system_clock::time_point next_start_time{current_start_time};
    const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
    while (true)
    {
      current_start_time = system_clock::now();
      next_start_time = current_start_time + interval;
      PlanCycleCallback();
      PublishTrajectory();
      if (!use_sim_state_)
      {
        PublishCtrlCommand();
      }
      std::this_thread::sleep_until(next_start_time);
    }
  }

  void SscPlannerServer::PlanCycleCallback()
  {
    if (!is_replan_on_)
    {
      return;
    }
    // printf("input buffer size: %d.\n", p_input_smm_buff_->size_approx());
    // is_map_updated_ = false;  // return when no new map
    while (p_input_smm_buff_->try_dequeue(last_smm_))
    {
      is_map_updated_ = true;
    }

    if (!is_map_updated_)
      return;

    // Update map
    auto map_ptr =
        std::make_shared<semantic_map_manager::EgoSemanticMapManager>(last_smm_);
    map_adapter_.set_map(map_ptr);

    PublishData();

    auto current_time = ros::Time::now().toSec();
    LOG(ERROR) << "[SscPlannerServer]>>>>>>>current time " << current_time;
    LOG(ERROR) << "[SscPlannerServer] current state <x,y,v,acc,angle,steer,curvature>:<" 
              << last_smm_.ego_vehicle().state().vec_position[0] << ", " 
              << last_smm_.ego_vehicle().state().vec_position[1] << ", " 
              << last_smm_.ego_vehicle().state().velocity << ", "
              << last_smm_.ego_vehicle().state().acceleration << ", "
              << last_smm_.ego_vehicle().state().angle << ", "
              << last_smm_.ego_vehicle().state().steer << ", "
              << last_smm_.ego_vehicle().state().curvature
              << ">";

    if (executing_traj_ == nullptr || !executing_traj_->IsValid() ||
        !use_sim_state_)
    {
      // Init planning
      // if (!use_sim_state_) {
      //   common::State desire_state = last_smm_.ego_vehicle().state();
      //   desire_state.acceleration = 0.0;
      //   desire_state.steer = 0.0;
      //   desire_state.curvature = 0.0;
      //   planner_.set_initial_state(desire_state);
      // }

      if (planner_.RunOnce() != kSuccess)
      {
        LOG(ERROR) << "[SscPlannerServer]Initial planning failed.\n";
        require_intervention_signal_ = true;
        return;
      }
      require_intervention_signal_ = false;
      desired_state_hist_.clear();
      desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
      ctrl_state_hist_.clear();
      ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());
      executing_traj_ = std::move(planner_.trajectory());
      global_init_stamp_ = executing_traj_->begin();
      // printf(
      //     "[SscPlannerServer]init plan success with stamp: %lf and angle %lf.\n",
      //     global_init_stamp_, last_smm_.ego_vehicle().state().angle);
      return;
    }

    if (current_time > executing_traj_->end())
    {
      LOG(ERROR) << "[SscPlannerServer]Current time " << current_time 
                << "out of [" << executing_traj_->begin() << ", " 
                << executing_traj_->end() << "].";
      LOG(ERROR) << "[SscPlannerServer]Mission complete.\n";
      // is_replan_on_ = false;
      executing_traj_.release();
      next_traj_.release();
      return;
    }

    // NOTE: comment this block if you want to disable replan
    {
      if (next_traj_ == nullptr || !next_traj_->IsValid())
      {
        Replan();
        return;
      }

      if (next_traj_->IsValid())
      {
        executing_traj_ = std::move(next_traj_);
        // next_traj_ = common::Trajectory();
        Replan();
        return;
      }
    }
  }

  void SscPlannerServer::Replan()
  {
    if (!is_replan_on_)
      return;
    if (executing_traj_ == nullptr || !executing_traj_->IsValid())
      return;

    decimal_t plan_horizon = 1.0 / work_rate_;
    common::State desired_state;
    decimal_t cur_time = ros::Time::now().toSec();

    int num_cycles_exec = std::floor(
        (executing_traj_->begin() - global_init_stamp_) / plan_horizon);
    int num_cycles_ahead =
        cur_time > executing_traj_->begin()
            ? std::floor((cur_time - global_init_stamp_) / plan_horizon) + 1
            : num_cycles_exec + 1;
    decimal_t t = global_init_stamp_ + plan_horizon * num_cycles_ahead;

    LOG(ERROR) << 
        "[SscPlannerServer]init stamp: " << global_init_stamp_ << ", plan horizon: "
        << plan_horizon << ", num cycles " << num_cycles_ahead;
    LOG(ERROR) << 
        "[SscPlannerServer]Replan at cur time " << cur_time << " with executing traj begin time: "
        << executing_traj_->begin() << " to rounded t: " << t;

    if (executing_traj_->GetState(t, &desired_state) != kSuccess)
    {
      LOG(ERROR) << "[SscPlannerServer]Cannot get desired state at " << t;
      return;
    }
    std::ostringstream line_info;
    line_info << "[SscPlannerServer]t " << t << ", desired state <x,y,v,a,theta>:<" 
              << desired_state.vec_position[0] << ", " 
              << desired_state.vec_position[1] << ", "
              << desired_state.velocity << ", "
              << desired_state.acceleration << ", " 
              << desired_state.angle << ">";
    LOG(ERROR) << line_info.str();

    FilterSingularityState(desired_state_hist_, &desired_state);
    desired_state_hist_.push_back(desired_state);
    if (desired_state_hist_.size() > 100)
      desired_state_hist_.erase(desired_state_hist_.begin());

    planner_.set_initial_state(desired_state);

    time_profile_tool_.tic();
    if (planner_.RunOnce() != kSuccess)
    {
      LOG(ERROR) << "[SscPlannerServer]Ssc planner core failed in " << time_profile_tool_.toc() << " ms.";
      require_intervention_signal_ = true;
      return;
    }

    require_intervention_signal_ = false;
    LOG(ERROR) << "[SscPlannerServer]Ssc planner succeed in " << time_profile_tool_.toc() << " ms.";

    next_traj_ = std::move(planner_.trajectory());
  }

} // namespace planning