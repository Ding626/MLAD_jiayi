#include "phy_env_manager/phy_manager.h"

namespace phy_env_manager {

PhyManager::PhyManager() {
  p_arena_loader_ = new ArenaLoader();
  if (!GetDataFromArenaLoader()) assert(false);
}

PhyManager::PhyManager(const std::string &vehicle_set_path,
                       const std::string &map_path,
                       const std::string &lane_net_path) {
  std::cout << "[PhyManager] Constructing..." << std::endl;
  p_arena_loader_ = new ArenaLoader();

  p_arena_loader_->set_vehicle_set_path(vehicle_set_path);
  p_arena_loader_->set_map_path(map_path);
  p_arena_loader_->set_lane_net_path(lane_net_path);

  GetDataFromArenaLoader();
  for (const auto &p : vehicle_set_.vehicles) {
    vehicle_ids_.push_back(p.first);
  }
}

bool PhyManager::GetDataFromArenaLoader() {
  std::cout << "[PhyManager] Parsing simulation info..." << std::endl;
  p_arena_loader_->ParseVehicleSet(&vehicle_set_);
  p_arena_loader_->ParseMapInfo(&obstacle_set_);
  p_arena_loader_->ParseLaneNetInfo(&lane_net_);
  return true;
}

bool PhyManager::AddTemporaryObstacleToMap(const common::Point &pt,
                                              const double &s) {
  common::PolygonObstacle obs;
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y - s / 2));
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y - s / 2));
  obs.type = 1;

  obs.id = temp_obs_idx_offset_ + temp_obstacle_cnt_;
  obstacle_set_.obs_polygon.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  temp_obstacle_set_.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  ++temp_obstacle_cnt_;
  return true;
}

// bool PhyManager::SetupVehicleModelForVehicleSet() {
//   for (const auto &p : vehicle_set_.vehicles) {
//     simulator::VehicleModel vehicle_model(
//         p.second.param().wheel_base(), p.second.param().max_steering_angle());
//     vehicle_model.set_state(p.second.state());
//     vehicle_model_set_.insert(
//         std::pair<int, simulator::VehicleModel>(p.first, vehicle_model));

//     vehicle_ids_.push_back(p.first);
//   }
//   return true;
// }

// bool PhyManager::UpdateSimulatorUsingSignalSet(
//     const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
//   TicToc updata_vehicle_time;
//   UpdateVehicleStates(signal_set, dt);
//   return true;
// }

bool PhyManager::UpdateVehicleSetUsingStateSet(
    const std::unordered_map<int, common::State> &state_set) {
  if (state_set.size() != vehicle_set_.vehicles.size()) {
    std::cerr << "[PhyManager] ERROR - State number error." << std::endl;
    std::cerr << "[PhyManager] state_set num: "
              << state_set.size()
              << ", vehicle_set num: " << vehicle_set_.vehicles.size()
              << std::endl;
    assert(false);
  }
  for (auto iter = vehicle_set_.vehicles.begin();
       iter != vehicle_set_.vehicles.end(); ++iter) {
    int id = iter->first;
    if (state_set.count(id)==0) {
      std::cerr << "[PhyManager] ERROR - vehicle id is not in state set error." << std::endl;
      std::cerr << "[PhyManager] vehicle id: "
                << id
                << std::endl;
      return false;
    }
    common::State cur_state = state_set.at(id);

    iter->second.set_state(cur_state);
  }
  return true;
}
// bool PhyManager::UpdateVehicleStates(
//     const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
//   if (signal_set.signal_set.size() != vehicle_set_.vehicles.size()) {
//     std::cerr << "[PhyManager] ERROR - Signal number error." << std::endl;
//     std::cerr << "[PhyManager] signal_set num: "
//               << signal_set.signal_set.size()
//               << ", vehicle_set num: " << vehicle_set_.vehicles.size()
//               << std::endl;
//     assert(false);
//   }
//   for (auto iter = vehicle_set_.vehicles.begin();
//        iter != vehicle_set_.vehicles.end(); ++iter) {
//     int id = iter->first;
//     common::VehicleControlSignal signal = signal_set.signal_set.at(id);
//     decimal_t steer_rate = signal.steer_rate;
//     decimal_t acc = signal.acc;

//     auto model_iter = vehicle_model_set_.find(id);

//     if (!signal.is_openloop) {
//       model_iter->second.set_control(
//           simulator::VehicleModel::Control(steer_rate, acc));
//       model_iter->second.Step(dt);
//     } else {
//       model_iter->second.set_state(signal.state);
//     }
//     iter->second.set_state(model_iter->second.state());
//   }
//   return true;
// }

bool PhyManager::RemoveTemporaryObstacle(const common::Point &pt,
                                            const double &s) {
  for (auto it = temp_obstacle_set_.begin(); it != temp_obstacle_set_.end();) {
    decimal_t sum_x = 0, sum_y = 0;
    for (const auto &p : it->second.polygon.points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    decimal_t center_x = sum_x / it->second.polygon.points.size();
    decimal_t center_y = sum_y / it->second.polygon.points.size();
    decimal_t dx = center_x - pt.x;
    decimal_t dy = center_y - pt.y;
    decimal_t d = std::hypot(dx, dy);

    if (d < s) {
      auto it_obs_set = obstacle_set_.obs_polygon.find(it->first);
      if (it_obs_set != obstacle_set_.obs_polygon.end()) {
        obstacle_set_.obs_polygon.erase(it_obs_set);
      }
      it = temp_obstacle_set_.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

}  // namespace phy_env_manager