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
  const int num_lanes = 3;
  const double lanes[3] = { 2.0, 6.0, 10.0 };
  double check_collision(const tk::spline& path, double last_x, double speed);
  bool check_collision(const std::vector<double>& path_x, const std::vector<double>& path_y);
  tk::spline getPath(int target_lane, int num_pts_prev);
  std::pair<std::vector<double>, std::vector<double>> generateTrajectory(int target_lane, double target_velocity);
  int getLane(double d);
  std::array<double, 3> getLaneSpeeds(double front_distance = 50., double back_distance = 20.);
  std::pair<double, double> findClosestCarInLane();
  bool isLaneClear(int target_lane, double margin_s);
  int fastestLane(const std::array<double, 3>& lane_speed);
public:
  TrajectoryGenerator(const nlohmann::json& telemetry, const Map& map_);
  std::pair<std::vector<double>, std::vector<double>> getOptimalTrajectory();
};

#endif
