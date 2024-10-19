#include "semantic_map_manager/config_loader.h"

namespace semantic_map_manager {

using Json = nlohmann::json;

ErrorType ConfigLoader::ParseAgentConfig(AgentConfigInfo *p_agent_config) {
  printf("\n[ConfigLoader] Loading vehicle set\n");

  std::fstream fs(agent_config_path_);
  Json root;
  fs >> root;

  Json agent_config_json = root["agent_config"];
  int num = static_cast<int>(agent_config_json["info"].size());
  for (int i = 0; i < num; ++i) {
    Json agent = agent_config_json["info"][i];
    if (agent["id"].get<int>() != ego_id_) continue;
    p_agent_config->obstacle_map_meta_info = common::GridMapMetaInfo(
        agent["obstacle_map_meta_info"]["width"].get<double>(),
        agent["obstacle_map_meta_info"]["height"].get<double>(),
        agent["obstacle_map_meta_info"]["resolution"].get<double>());
    p_agent_config->surrounding_search_radius =
        agent["surrounding_search_radius"].get<double>();
    p_agent_config->enable_openloop_prediction =
        agent["enable_openloop_prediction"].get<bool>();
    if (agent.count("enable_tracking_noise")) {
      p_agent_config->enable_tracking_noise =
          agent["enable_tracking_noise"].get<bool>();
    }
    if (agent.count("enable_log")) {
      p_agent_config->enable_log = agent["enable_log"].get<bool>();
      p_agent_config->log_file = agent["log_file"].get<std::string>();
    }
    if (agent.count("desire_velocity")) {
      p_agent_config->desire_velocity =
          agent["desire_velocity"].get<double>();
    }
    if (agent.count("advanced_time_for_keep_lane_plan")) {
      p_agent_config->advanced_time_for_keep_lane_plan =
      agent["advanced_time_for_keep_lane_plan"].get<double>();
    }
  }

  p_agent_config->PrintInfo();
  fs.close();
  return kSuccess;
}

ErrorType ConfigLoader::ParseAgentConfig(std::unordered_map<std::string, AgentConfigInfo> *agent_config_infos) {
  printf("\n[ConfigLoader] Loading vehicle set with different types\n");

  std::fstream fs(agent_config_path_);
  Json root;
  fs >> root;

  Json agent_config_json = root["agent_config"];

  if (agent_config_json.count("enable_log")) {
    enable_log_ = agent_config_json["enable_log"].get<bool>();
    log_file_ = agent_config_json["log_file"].get<std::string>();
  }

  if (agent_config_json.count("ego_type")) {
    ego_type_ = agent_config_json["ego_type"].get<std::string>();
  }

  int num = static_cast<int>(agent_config_json["info"].size());
  for (int i = 0; i < num; ++i) {
    Json agent = agent_config_json["info"][i];
    AgentConfigInfo info;
    info.obstacle_map_meta_info = common::GridMapMetaInfo(
        agent["obstacle_map_meta_info"]["width"].get<double>(),
        agent["obstacle_map_meta_info"]["height"].get<double>(),
        agent["obstacle_map_meta_info"]["resolution"].get<double>());
    info.surrounding_search_radius =
        agent["surrounding_search_radius"].get<double>();
    info.enable_openloop_prediction =
        agent["enable_openloop_prediction"].get<bool>();
    if (agent.count("enable_tracking_noise")) {
      info.enable_tracking_noise =
          agent["enable_tracking_noise"].get<bool>();
    }
    if (agent.count("desire_velocity")) {
      info.desire_velocity =
          agent["desire_velocity"].get<double>();
    }
    if (agent.count("max_num")) {
      info.max_num =
          agent["max_num"].get<int>();
    }
    if (agent.count("advanced_time_for_keep_lane_plan")) {
      info.advanced_time_for_keep_lane_plan =
          agent["advanced_time_for_keep_lane_plan"].get<double>();
    }

    std::string type_name = agent["type"].get<std::string>();
    info.type = type_name;
    if (agent_config_infos->count(type_name)==0) {
      vehicle_types_.push_back(type_name);
      agent_config_infos->insert(std::make_pair(type_name, info));
    }
    else {
      agent_config_infos->at(type_name) = info;
    }
    info.PrintInfo();
  }

  fs.close();
  return kSuccess;
}

ErrorType ConfigLoader::ParseLaneConfig(common::LaneNet *whole_lane_net) {
  printf("\n[ArenaLoader] Loading lane net info\n");

  std::fstream fs(lane_config_path_);
  Json root;
  fs >> root;

  Json lane_net_json = root["features"];

  for (int i = 0; i < static_cast<int>(lane_net_json.size()); ++i) {
    common::LaneRaw lane_raw;
    Json lane_json = lane_net_json[i];
    Json lane_meta = lane_json["properties"];

    lane_raw.id = lane_meta["id"].get<int>(); // id应该从1开始，0表示没有车道
    lane_raw.length = lane_meta["length"].get<double>();
    lane_raw.dir = lane_meta["dir"].get<int>();
    lane_raw.buffer = lane_meta["buffer"].get<double>();
    lane_raw.group = lane_meta["group"].get<double>();
    lane_raw.is_multi_lane_route = lane_meta["is_multi_lane_route"].get<bool>();

    printf("-Lane id %d.\n", lane_raw.id);
    std::string str_child_id = lane_meta["child_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_child_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.child_id.push_back(lane_id);
      }
    }

    std::string str_father_id = lane_meta["father_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_father_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.father_id.push_back(lane_id);
      }
    }
    // NOTE for MLAD: 需要确定一个主方向，load路径时标号顺序的左右，是以主方向为准的
    // NOTE for MLAD: 在主方向下，1代表最右边的车道，越大代表越靠左的车道，json文件中车道为0表示不存在车道
    lane_raw.l_lane_id = lane_meta["left_id"].get<int>();
    lane_raw.r_lane_id = lane_meta["right_id"].get<int>();

    int lchg_vld = lane_meta["lchg_vld"].get<int>(); // NOTE: rchg_vld为1，表示为可以向右变道，为0则不行
    int rchg_vld = lane_meta["rchg_vld"].get<int>();
    lane_raw.l_change_avbl = lane_raw.l_lane_id==0 ? false : static_cast<bool>(lchg_vld);
    lane_raw.r_change_avbl = lane_raw.r_lane_id==0 ? false : static_cast<bool>(rchg_vld);
    lane_raw.behavior = lane_meta["behavior"].get<std::string>();
    lane_raw.dir_changeable = lane_raw.is_multi_lane_route && lane_raw.l_change_avbl && lane_raw.r_change_avbl ? lane_meta["dir_changeable"].get<bool>() : false;
    // NOTE for MLAD: 只有当车道左右两边都可以变道时，才是通行方向可以变换的车道，否则是最边缘车道
    Json lane_coordinates = lane_json["geometry"]["coordinates"][0];
    int num_pts = static_cast<int>(lane_coordinates.size());
    for (int k = 0; k < num_pts; ++k) {
      Vec2f pt(lane_coordinates[k][0].get<double>(),
               lane_coordinates[k][1].get<double>());
      lane_raw.lane_points.emplace_back(pt);
    }
    lane_raw.start_point = *(lane_raw.lane_points.begin());
    lane_raw.final_point = *(lane_raw.lane_points.rbegin());
    whole_lane_net->lane_set.insert(
        std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
  }
  whole_lane_net->print();

  fs.close();
  return kSuccess;
}

}  // namespace semantic_map_manager