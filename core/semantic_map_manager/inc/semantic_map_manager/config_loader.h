#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>
#include <unordered_map>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/basics/behavior.h"
#include "semantic_map_manager/basics.h"

namespace semantic_map_manager {

class ConfigLoader {
 public:
  ConfigLoader() {}
  ConfigLoader(const std::string &agent_config_path)
      : agent_config_path_(agent_config_path) {}
  ConfigLoader(const std::string &agent_config_path, const std::string &lane_config_path)
      : agent_config_path_(agent_config_path), lane_config_path_(lane_config_path) {}
  ~ConfigLoader() {}

  inline std::string agent_config_path() const { return agent_config_path_; }
  inline void set_agent_config_path(const std::string &path) {
    agent_config_path_ = path;
  };

  inline std::string lane_config_path() const { return lane_config_path_; }
  inline void set_lane_config_path(const std::string &path) {
    lane_config_path_ = path;
  };

  inline int ego_id() const { return ego_id_; } // ! Deprecate
  inline void set_ego_id(const int &id) { ego_id_ = id; } // ! Deprecate

  inline std::vector<int> key_ids() const { return key_vehicle_ids_; } // ! Deprecate
  inline void set_key_ids(const std::vector<int> &ids) { key_vehicle_ids_ = ids; } // ! Deprecate

  inline std::vector<std::string> vehicle_types() const { return vehicle_types_; }
  inline std::string log_file() const { return log_file_; }
  inline std::string ego_type() const { return ego_type_; }
  inline void set_vehicle_types(const std::vector<std::string> &ids) { 
    for (const auto s:ids) vehicle_types_.emplace_back(s);
  }

  ErrorType ParseAgentConfig(AgentConfigInfo *p_agent_config);
  ErrorType ParseAgentConfig(std::unordered_map<std::string, AgentConfigInfo> *agent_config_infos);
  ErrorType ParseLaneConfig(common::LaneNet *whole_lane_net);

 private:
  int ego_id_;  // ! Deprecate
  std::vector<int> key_vehicle_ids_;  // ! Deprecate
  std::string agent_config_path_;
  std::vector<std::string> vehicle_types_;

  std::string lane_config_path_;

  bool enable_log_;
  std::string log_file_;
  std::string ego_type_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_