#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H
#include <vector>
#include <utility>
#include "spline.h"
#include "car.h"
#include "map.h"
#include "json.hpp"

class TrajectoryGenerator {
  const Map& map;
  Car car;
  std::vector<double> prev_path_x;
  std::vector<double> prev_path_y;
  std::vector<double> prev_path_x_car; //previous path x coord in car coordinates
  std::vector<double> prev_path_y_car; //previous path y coord in car coordinates
  std::vector<Agent> agents;
  const double dt = 0.02;
public:
  TrajectoryGenerator(const nlohmann::json& telemetry, const Map& map_);
  std::pair<std::vector<double>, std::vector<double>> getPath();
};

#endif
