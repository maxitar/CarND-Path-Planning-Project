#include "trajectory_generator.h"
#include <algorithm>
#include "json.hpp"

TrajectoryGenerator::TrajectoryGenerator(const nlohmann::json& j, const Map& map_) : map(map_) {
  // Main car's localization Data
  double car_x = j[1]["x"];
  double car_y = j[1]["y"];
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_yaw = j[1]["yaw"];
  double car_speed = j[1]["speed"];
  car.set_values(car_x, car_y, car_s, car_d, car_yaw, car_speed);

  // Previous path data given to the Planner
  std::vector<double> prev_path_x_ = j[1]["previous_path_x"];
  prev_path_x = std::move(prev_path_x_);
  std::vector<double> prev_path_y_ = j[1]["previous_path_y"];
  prev_path_y = std::move(prev_path_y_);
  // Previous path's end s and d values 
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = j[1]["sensor_fusion"];
  for (auto&& agent : sensor_fusion) {
    auto iter = agent.begin();
    int    id = *iter; ++iter;
    double x  = *iter; ++iter;
    double y  = *iter; ++iter;
    double vx = *iter; ++iter;
    double vy = *iter; ++iter;
    double s  = *iter; ++iter;
    double d  = *iter; 
    agents.emplace_back(id, x,y,vx,vy,s,d);
  }
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::getPath() {
  int path_pts_num = 50;
  double target_v = 22; //meters per second
  std::vector<double> next_x_vals;
  next_x_vals.reserve(path_pts_num);
  std::vector<double> next_y_vals;
  next_y_vals.reserve(path_pts_num);
  //int pts_to_copy = std::min(30, int(prev_path_x.size()));
  int pts_to_copy = prev_path_x.size();
  std::vector<double> temp_x;
  std::vector<double> temp_y;
  if (prev_path_x.size() == 0) {
    temp_x.reserve(4);
    temp_y.reserve(4);
    temp_x.push_back(car.x());
    temp_y.push_back(car.y());
    pts_to_copy = 1;
    //next_x_vals[0] = car.x();
    //next_y_vals[0] = car.y();
    next_x_vals.push_back(car.x());
    next_y_vals.push_back(car.y());
  }
  else {
    temp_x.reserve(pts_to_copy+3);
    temp_y.reserve(pts_to_copy+3);
    temp_x.resize(pts_to_copy);
    temp_y.resize(pts_to_copy);
    std::copy(prev_path_x.begin(), prev_path_x.begin()+pts_to_copy, temp_x.begin());
    std::copy(prev_path_y.begin(), prev_path_y.begin()+pts_to_copy, temp_y.begin());
    std::copy(prev_path_x.begin(), prev_path_x.begin()+pts_to_copy, std::back_inserter(next_x_vals));
    std::copy(prev_path_y.begin(), prev_path_y.begin()+pts_to_copy, std::back_inserter(next_y_vals));
  }

  double last_x = temp_x.back();
  double last_y = temp_y.back();
  auto sd = map.getFrenet(last_x, last_y, car.yaw());
  for (int i = 0; i < 3; ++i) {
    auto xy = map.getXY(sd.first+(i+1)*15, 6.0);
    temp_x.push_back(xy.first);
    temp_y.push_back(xy.second);
  }

  for (int i = 0, n = temp_x.size(); i < n; ++i) {
    std::tie(temp_x[i], temp_y[i]) = car.from_map(temp_x[i], temp_y[i]);
  }
  
  tk::spline path;
  path.set_points(temp_x, temp_y);

  auto last_xy = car.from_map(last_x, last_y);
  last_x = last_xy.first;
  last_y = last_xy.second;
  double speed = car.speed();
  int num_prev = next_x_vals.size();
  if (num_prev > 1) {
    speed = (next_x_vals[num_prev-1] - next_x_vals[num_prev-2])/dt;
    auto rprev = prev_path_x.rbegin();
    std::cout << (*rprev-*(rprev+1))/dt << std::endl;
  }
  std::cout << num_prev << " " <<  car.speed() << " " << speed << std::endl;
  //double dx = speed; //convert to 
  for (int i = pts_to_copy; i < path_pts_num; ++i) {
    if (speed < 22) speed += 0.18;
    //if (speed > 22) speed = 22;
    double dx = speed*dt; //convert to 
    //double x = last_x+0.44*(i+1-pts_to_copy);
    //double x = last_x+dx*(i+1-pts_to_copy);
    double x = last_x+dx;
    last_x = x;
    //double x = 0.44*i;
    next_x_vals.push_back(0);
    next_y_vals.push_back(0);
    std::tie(next_x_vals[i], next_y_vals[i]) = car.to_map(x, path(x));
  }
  std::cout << "Final speed: " << speed << std::endl;
  auto rbegin = next_x_vals.rbegin();
  std::cout << (*rbegin-*(rbegin+1))/dt << std::endl;
  return {next_x_vals, next_y_vals};
}

//std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::getPath() {
//  int path_pts_num = 50;
//  double target_v = 22; //meters per second
//  std::vector<double> next_x_vals;
//  next_x_vals.reserve(path_pts_num);
//  std::vector<double> next_y_vals;
//  next_y_vals.reserve(path_pts_num);
//  int pts_to_copy = std::min(30, int(prev_path_x.size()));
//  std::vector<double> temp_x;
//  std::vector<double> temp_y;
//  if (prev_path_x.size() == 0) {
//    temp_x.reserve(4);
//    temp_y.reserve(4);
//    temp_x.push_back(car.x());
//    temp_y.push_back(car.y());
//    pts_to_copy = 1;
//    //next_x_vals[0] = car.x();
//    //next_y_vals[0] = car.y();
//    next_x_vals.push_back(car.x());
//    next_y_vals.push_back(car.y());
//  }
//  else {
//    temp_x.reserve(pts_to_copy+3);
//    temp_y.reserve(pts_to_copy+3);
//    temp_x.resize(pts_to_copy);
//    temp_y.resize(pts_to_copy);
//    std::copy(prev_path_x.begin(), prev_path_x.begin()+pts_to_copy, temp_x.begin());
//    std::copy(prev_path_y.begin(), prev_path_y.begin()+pts_to_copy, temp_y.begin());
//    std::copy(prev_path_x.begin(), prev_path_x.begin()+pts_to_copy, std::back_inserter(next_x_vals));
//    std::copy(prev_path_y.begin(), prev_path_y.begin()+pts_to_copy, std::back_inserter(next_y_vals));
//  }
//
//  double last_x = temp_x.back();
//  double last_y = temp_y.back();
//  auto sd = map.getFrenet(last_x, last_y, car.yaw());
//  for (int i = 0; i < 3; ++i) {
//    auto xy = map.getXY(sd.first+(i+1)*15, 6.0);
//    temp_x.push_back(xy.first);
//    temp_y.push_back(xy.second);
//  }
//
//  for (int i = 0, n = temp_x.size(); i < n; ++i) {
//    std::tie(temp_x[i], temp_y[i]) = car.from_map(temp_x[i], temp_y[i]);
//  }
//  
//  tk::spline path;
//  path.set_points(temp_x, temp_y);
//
//  auto last_xy = car.from_map(last_x, last_y);
//  last_x = last_xy.first;
//  last_y = last_xy.second;
//  double speed = car.speed();
//  int num_prev = next_x_vals.size();
//  if (num_prev > 1) {
//    speed = (next_x_vals[num_prev-1] - next_x_vals[num_prev-2])/dt;
//  }
//  std::cout << num_prev << " " <<  car.speed() << " " << speed << std::endl;
//  //double dx = speed; //convert to 
//  for (int i = pts_to_copy; i < path_pts_num; ++i) {
//    if (speed < 22) speed += 0.2;
//    if (speed > 22) speed = 22;
//    double dx = speed*dt; //convert to 
//    //double x = last_x+0.44*(i+1-pts_to_copy);
//    //double x = last_x+dx*(i+1-pts_to_copy);
//    double x = last_x+dx;
//    last_x = x;
//    //double x = 0.44*i;
//    next_x_vals.push_back(0);
//    next_y_vals.push_back(0);
//    std::tie(next_x_vals[i], next_y_vals[i]) = car.to_map(x, path(x));
//  }
//  std::cout << "Final speed: " << speed << std::endl;
//  return {next_x_vals, next_y_vals};
//}
