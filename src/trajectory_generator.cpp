#include "trajectory_generator.h"
#include <algorithm>
#include "json.hpp"

// compute s1-s2 for s1,s2 in [0, s_max] 
double subtract_s(double s1, double s2, double s_max = 6945.554) {
  double diff = s1 - s2;
  if (std::fabs(diff) < 0.5*s_max) return diff;
  return s_max - diff;
}

bool collision(double car_s, double car_d, double agent_s, double agent_d, double margin_s = 5., double margin_d = 2.5) {
  double distance = subtract_s(car_s, agent_s);
  //return (distance > -margin_s && distance < margin_s && car_d > agent_d - margin_d && car_d < agent_d + margin_d);
  return (std::fabs(distance) < margin_s && std::fabs(car_d-agent_d) < margin_d);
}

int getLane(double d) {
  int result = int(d) / 4;
  result = result > 2 ? 2 : result;
  result = result < 0 ? 0 : result;
  return result;
}

int fastestLane(const std::array<double, 3>& lane_speed) {
  return std::distance(std::begin(lane_speed), std::max_element(std::begin(lane_speed), std::end(lane_speed)));
}

void TrajectoryGenerator::FSM::update_state() {
  std::array<double, 3> lane_speeds = gen.getLaneSpeeds(50, 3);
  auto front_vehicle = gen.findClosestCarInLane();
  double front_dist = front_vehicle.first;
  double front_speed = front_vehicle.second;
  int fastest_lane = fastestLane(lane_speeds);
  if (state == KL) {
    steps_in_lane += steps_in_lane < 100 ? 1 : 0;
    if (steps_in_lane == 100 && lane_speeds[fastest_lane] > lane_speeds[origin_lane] + 1.0) {
      if (std::abs(fastest_lane - origin_lane) == 2) target_lane = 1;
      else target_lane = fastest_lane;
      final_lane = fastest_lane;
      state = PLC;
      std::cout << "Changing state to PLC.\n";
      std::cout << "Origin lane is " << origin_lane << " with speed " << lane_speeds[origin_lane] << "mps\n";
      std::cout << "Final lane is " << fastest_lane << " with speed " << lane_speeds[fastest_lane] << "mps\n";
    } else {
      if (front_dist < 10) {
        target_speed = front_speed - 2;
      }
      else if (front_dist < 20) {
        target_speed = front_speed;
      }
      else {
        target_speed = gen.car.max_speed;
      }
    }
  }
  // next lane front and back cars
  auto nl_front = gen.findNextCarInLane(target_lane, true);
  auto nl_back = gen.findNextCarInLane(target_lane, false);
  if (state == PLC) {
    if (front_dist < 10) {
      std::cout << "Too close during PLC at: " << front_dist << "\n";
      target_speed = front_speed - 2;
      return;
    }
    if (lane_speeds[final_lane] < lane_speeds[origin_lane] + 1.0 || final_lane != fastest_lane) {
      target_lane = origin_lane;
      final_lane = origin_lane;
      state = KL;
      std::cout << "Reverting state back to KL.\n";
      //steps_in_lane = 0;
      return;
    }
    if (nl_back.first > 10 && -nl_back.first + 3*(nl_back.second - gen.car.speed()) < -10) {
      if (nl_front.first > 10 && nl_front.first + 3*(nl_front.second - gen.car.speed()) > 10) {
        std::cout << "Changing state to LC.\n";
        state = LC;
      }
      else {
        if (front_dist < 20) {
          if (front_speed-nl_front.second < 1.) { 
            std::cout << "Hit front condition for " << front_speed << " and " << nl_front.second << "\n";
            target_speed = std::min(front_speed, nl_front.second - 1.);
          } else { 
            target_speed = front_speed;
          }
        } else {
          target_speed = gen.car.max_speed;
        }
      }
    }
    else {
      if (front_dist < 15) {
        if (front_dist + nl_back.first < 20 && front_speed-nl_back.second < 1.) { 
          std::cout << "Hit back condition for " << front_speed << " and " << nl_back.second << "\n";
          target_speed = std::min(front_speed, nl_back.second - 1.);
        } else { 
          target_speed = front_speed;
        }
      } else {
        target_speed = gen.car.max_speed;
      }
//      if (front_dist+nl_back_first < 15 && front_speed-nl_back.second < 1.) { 
//        std::cout << "Hit back condition for " << front_speed << " and " << nl_back.second << "\n";
//        target_speed = std::min(front_speed, nl_back.second - 1.);
//      } else if (front_dist < 20) {
//        target_speed = front_speed;
//      } else {
//        target_speed = gen.car.max_speed;
//      }
    }
  }
  if (state == LC) {
    //int car_lane = getLane(gen.car.d());
    if (std::fabs(gen.car.d()-gen.lanes[target_lane]) > 1.) {
      if (front_dist < 10) {
        target_speed = std::min(front_speed, nl_front.second);
      }
      else {
        target_speed = nl_front.first < 30 ? nl_front.second : gen.car.max_speed;
      }
    } else {
      target_speed = nl_front.first < 30 ? nl_front.second : gen.car.max_speed;
      origin_lane = target_lane;
      if (target_lane == final_lane) {
        steps_in_lane = 0;
        state = KL;
        std::cout << "Changing state to KL.\n";
      } else {
        target_lane = final_lane;
        state = PLC;
        std::cout << "Changing state to PLC.\n";
      }
    }
  }
}

std::pair<int,double> TrajectoryGenerator::FSM::get_lane_speed() {
  update_state();
  if (state == KL || state == PLC) return {origin_lane, target_speed};
  else return {target_lane, target_speed};
}

double TrajectoryGenerator::check_collision(const tk::spline & path, double last_x, double speed) {
  std::vector<double> s_traj;
  std::vector<double> d_traj;
  for (double dist = 0; dist < 60; dist += 4) {
    //auto map_xy = car.to_map(last_x + dist, path(last_x + dist));
    auto map_xy = car.to_map(dist, path(dist));
    auto sd = map.getFrenet(map_xy.first, map_xy.second, car.yaw());
    s_traj.push_back(sd.first);
    d_traj.push_back(sd.second);
  }
  double time_per_node = 4. / speed;
  double min_time = 1000;
  for (auto&& agent : agents) {
    if (agent.d - 1.5 < d_traj[0] && agent.d + 1.5 > d_traj[0]) {
      double diff_s = agent.s - s_traj[0];
      if (diff_s > 0 && diff_s < 100) {
        double curr_time = 1. + diff_s/(speed - agent.speed);
        std::cout << "Distance: " << diff_s << " car velocity: " << speed << " agent velocity: " << agent.speed << std::endl;
        if (curr_time < min_time) {
          min_time = curr_time;
        }
      }
    }
    /*double current_s = agent.s + agent.speed;
    int size_traj = s_traj.size();
    for (int i = 0; i < size_traj; ++i) {
      if (collision(s_traj[i], d_traj[i], current_s, agent.d)) return i*time_per_node + 1.0;
      current_s += time_per_node * agent.speed;
    }*/
  }
  if (min_time < 500) return min_time;
  return 0.0;
}

std::tuple<bool, double, double> TrajectoryGenerator::check_collision(const std::vector<double>& path_x, const std::vector<double>& path_y) {
  int num_pts = path_x.size();
  std::vector<double> s_traj;
  std::vector<double> d_traj;
  s_traj.reserve(num_pts);
  d_traj.reserve(num_pts);
  for (int i = 0; i < num_pts; ++i) {
    auto sd = map.getFrenet(path_x[i], path_y[i], car.yaw());
    s_traj.push_back(sd.first);
    d_traj.push_back(sd.second);
  }
  double min_time = 1000;
  double speed = 50.;
  bool collision_detected = false;
  for (auto&& agent : agents) {
    double current_s = agent.s;
    int size_traj = s_traj.size();
    for (int i = 0; i < size_traj; ++i) {
      double time = i*dt;
      if (collision(s_traj[i], d_traj[i], agent.s + agent.speed*time, agent.d, 4., 2.) && time < min_time) {
        //return {true, agent.speed};
        min_time = time;
        speed = agent.speed;
        collision_detected = true;
        break;
      }
    }
  }
  return std::tuple<bool, double, double>(collision_detected, speed, min_time);
}

tk::spline TrajectoryGenerator::getPath(int target_lane, int num_pts_prev) {
  std::vector<double> temp_x;
  std::vector<double> temp_y;

  double final_s = 90.;
  double step_distance = 30.;
  //double step_distance = target_lane == getLane(car.d()) ? 10. : 30.;
  int steps = final_s / step_distance;
  double final_d = lanes[target_lane];

  if (prev_path_x.size() == 0) {
    temp_x.reserve(4);
    temp_y.reserve(4);
    temp_x.push_back(car.x());
    temp_y.push_back(car.y());
  }
  else {
    temp_x.reserve(num_pts_prev + steps);
    temp_y.reserve(num_pts_prev + steps);
    std::copy(prev_path_x.begin(), prev_path_x.begin() + num_pts_prev, std::back_inserter(temp_x));
    std::copy(prev_path_y.begin(), prev_path_y.begin() + num_pts_prev, std::back_inserter(temp_y));
  }

  double last_x = temp_x.back();
  double last_y = temp_y.back();
  auto sd = map.getFrenet(last_x, last_y, car.yaw());
  double current_d = sd.second;
  //double step_d = (current_d + final_d)*0.2;
  //double step_d = (final_d - current_d)*0.5;
  double step_d = (final_d - current_d);
  for (int i = 0; i < steps; ++i) {
    current_d += step_d;
    current_d = std::fabs(current_d - final_d) < 0.5 ? final_d : current_d;
    auto xy = map.getXY(sd.first + (i + 1)*step_distance, current_d);
    temp_x.push_back(xy.first);
    temp_y.push_back(xy.second);
  }

  for (int i = 0, n = temp_x.size(); i < n; ++i) {
    std::tie(temp_x[i], temp_y[i]) = car.from_map(temp_x[i], temp_y[i]);
  }

  tk::spline path;
  path.set_points(temp_x, temp_y);
  return path;
}

std::pair<std::vector<double>, std::vector<double>> 
TrajectoryGenerator::generateTrajectory(int target_lane, double target_velocity, int pts_to_copy)
{
  //int pts_to_copy = prev_path_x.size();
  int path_pts_num = 100; // 50 points for 1 second of driving
//  if (target_lane != getLane(car.d()) && pts_to_copy <= path_pts_num) { //if target lane is different, but also if we are not in the middle of another lane change
//    path_pts_num = 100; // increase to 2 seconds to also get the change of lanes
//  }
  double target_v = target_velocity; //meters per second
  std::vector<double> next_x_vals;
  next_x_vals.reserve(path_pts_num);
  std::vector<double> next_y_vals;
  next_y_vals.reserve(path_pts_num);
  //int pts_to_copy = std::min(30, int(prev_path_x.size()));

  if (prev_path_x.size() == 0) {
    pts_to_copy = 1;
    next_x_vals.push_back(car.x());
    next_y_vals.push_back(car.y());
  }
  else {
    std::copy(prev_path_x.begin(), prev_path_x.begin() + pts_to_copy, std::back_inserter(next_x_vals));
    std::copy(prev_path_y.begin(), prev_path_y.begin() + pts_to_copy, std::back_inserter(next_y_vals));
  }

  //if (pts_to_copy >= 50 && pts_to_copy < 100) {
  //  return { next_x_vals, next_y_vals };
  //}

  tk::spline path = getPath(target_lane, pts_to_copy);
  double last_x = next_x_vals.back();
  double last_y = next_y_vals.back();
  std::tie(last_x, last_y) = car.from_map(last_x, last_y);
  double speed = car.speed();
  int num_prev = next_x_vals.size();
  if (num_prev > 1) {
    speed = distance(next_x_vals[num_prev - 1], next_y_vals[num_prev - 1], next_x_vals[num_prev - 2], next_y_vals[num_prev - 2]) / dt;
  }
  //double dx = speed; //convert to 
  double x_diff = 1.;
  double x_target = last_x + x_diff;
  double y_target = path(x_target);
  //double y_diff = y_target - last_y;
  double len = distance(last_x, last_y, x_target, y_target);
  double l = 0;
  double acc_coef = getLane(car.d()) == target_lane ? 0.8 : 0.1;
  double vel_step = car.max_acc*acc_coef*dt;
  auto same_lane = [target_d = lanes[target_lane]](double d) { return d > target_d - 0.5 && d < target_d + 0.5; };
  for (int i = pts_to_copy; i < path_pts_num; ++i) {
  /*std::pair<double, double> sd = map.getFrenet(next_x_vals.back(), next_y_vals.back(), car.yaw());
  for (int i = pts_to_copy; 
    i < path_pts_num || !same_lane(sd.second); ++i, sd = map.getFrenet(next_x_vals.back(), next_y_vals.back(), car.yaw())) {*/
    if (speed < target_v - vel_step) speed += vel_step;// 0.18;
    if (speed > target_v + vel_step) speed -= vel_step;// 0.18;
    if (speed > target_v - vel_step && speed < target_v + vel_step) speed = target_v;
    double dl = speed * dt; //convert to 
    l += dl;
    double x = last_x + l / len * x_diff;
    if (l / len > 1.0) {
      last_x = x;
      last_y = path(last_x);
      x_target = last_x + x_diff;
      y_target = path(x_target);
      len = distance(last_x, last_y, x_target, y_target);
      l = 0;
    }
    //double x = last_x+0.44*(i+1-pts_to_copy);
    //double x = last_x+dx*(i+1-pts_to_copy);
    //double x = last_x+dx;
    //last_x = x;
    //double x = 0.44*i;
    next_x_vals.push_back(0);
    next_y_vals.push_back(0);
    std::tie(next_x_vals[i], next_y_vals[i]) = car.to_map(x, path(x));
  }
  return { next_x_vals, next_y_vals };
}

// @param front_distance  meters, distance to agents in front of the car
// @param back_distance  meters, distance to agents behind the car
std::array<double, 3> TrajectoryGenerator::getLaneSpeeds(double front_distance, double back_distance) {
  std::array<double, 3> lane_speed{ car.max_speed, car.max_speed, car.max_speed };  
  for (auto&& agent : agents) {
    double agent_to_car = subtract_s(agent.s, car.s()); //compute agent.s - car.s
    if ((agent_to_car >= 0 && agent_to_car < front_distance) || (agent_to_car < 0 && -agent_to_car < back_distance)) {
      int agent_lane = getLane(agent.d);
      if (agent.speed < lane_speed[agent_lane]) {
        lane_speed[agent_lane] = agent.speed;
      }
    }
  }
  return lane_speed;
}

// find the nearest agent in front of the car and return its distance and speed
std::pair<double, double> TrajectoryGenerator::findClosestCarInLane() {
  int car_lane = getLane(car.d());
  double distance = map.max_s;
  double speed = car.max_speed;
  for (auto&& agent : agents) {
    if (getLane(agent.d) == car_lane) {
      double agent_to_car = subtract_s(agent.s, car.s()); //compute agent.s - car.s
      if (agent_to_car >= 0 && agent_to_car < distance) {
        distance = agent_to_car;
        speed = agent.speed;
      }
    }
  }
  return { distance, speed };
}

// find the nearest agent in front of the car and return its distance and speed
std::pair<double, double> TrajectoryGenerator::findNextCarInLane(int lane, bool in_front) {
  int car_lane = getLane(car.d());
  double distance = map.max_s;
  double speed = car.max_speed;
  double coef = in_front ? 1. : -1;
  for (auto&& agent : agents) {
    if (getLane(agent.d) == lane) {
      double agent_to_car = coef*subtract_s(agent.s, car.s()); //compute agent.s - car.s
      if (agent_to_car >= 0 && agent_to_car < distance) {
        distance = agent_to_car;
        speed = agent.speed;
      }
    }
  }
  return { distance, speed };
}

bool TrajectoryGenerator::isLaneClear(int target_lane, double margin_s) {
  double distance = map.max_s;
  double speed = car.max_speed;
  for (auto&& agent : agents) {
    if (getLane(agent.d) == target_lane) {
      double agent_to_car = subtract_s(agent.s, car.s()); //compute agent.s - car.s
      if (std::fabs(agent_to_car) < margin_s) {
        return false;
      }
    }
  }
  return true;
}

void TrajectoryGenerator::update(const nlohmann::json& j) {
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
  agents.clear();
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

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::getOptimalTrajectory() {
  std::pair<int, double> lane_speed = fsm.get_lane_speed();
  //return generateTrajectory(lane_speed.first, lane_speed.second, prev_path_x.size());
  auto trajectory = generateTrajectory(lane_speed.first, lane_speed.second, 30);
  auto collision = check_collision(trajectory.first, trajectory.second);
  bool collision_detected = std::get<0>(collision);
  double time_to_collision = std::get<2>(collision);
  if (collision_detected && time_to_collision < 2.) {
    double agent_speed = std::get<1>(collision);
    std::cout << "Collision detected!\n";
    double target_speed = std::min(agent_speed, lane_speed.second);
    return generateTrajectory(lane_speed.first, target_speed, 5);
  }
  return trajectory;
}
//
//std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::getOptimalTrajectory() {
//  std::vector<double> next_x_vals;
//  std::vector<double> next_y_vals;
//  int car_lane = getLane(car.d());
//  auto nearestCar = findClosestCarInLane();
//  int target_lane = car_lane;
//  if (nearestCar.first < 4) {
//    std::cout << nearestCar.first << " " << nearestCar.first << std::endl;
//    return generateTrajectory(target_lane, nearestCar.second - 5., prev_path_x.size());
//  }
//  else {
//    auto lane_speeds = getLaneSpeeds(40., 3.);
//    int fastest_lane = fastestLane(lane_speeds);
//    if (lane_speeds[fastest_lane] > lane_speeds[car_lane] + 2.0) {
//      if (std::abs(fastest_lane - car_lane) == 2) target_lane = 1;
//      else target_lane = fastest_lane;
//    }
//    double target_speed = lane_speeds[target_lane];
//    std::tie(next_x_vals, next_y_vals) = generateTrajectory(target_lane, target_speed, prev_path_x.size());
//    //if (target_lane != car_lane && !check_collision(next_x_vals, next_y_vals)) {
//    if (target_lane != car_lane && isLaneClear(target_lane, 8)) {
//      std::cout << "Change lane!" << std::endl;
//      return { next_x_vals, next_y_vals };
//    }
//    else {
//      if (target_lane != car_lane && check_collision(next_x_vals, next_y_vals)) std::cout << "Collision!" << std::endl;
//      return generateTrajectory(car_lane, lane_speeds[car_lane], prev_path_x.size());
//    }
//  }
//  int num_pts = next_x_vals.size();
//  std::vector<double> s(num_pts, 0.);
//  for (int i = 1; i < num_pts; ++i) {
//    s[i] = s[i - 1] + distance(next_x_vals[i], next_y_vals[i], next_x_vals[i-1], next_y_vals[i-1]);
//  }
//  double avg_acc = 0.;
//  for (int i = 1; i < num_pts - 1; ++i) {
//    double acc = (-s[i - 1] + 2 * s[i] - s[i + 1]) / (dt*dt);
//    acc = std::fabs(acc);
//    avg_acc += acc;
//    if (acc > 9) {
//      //std::cout << acc << '\n';
//    }
//  }
//  //std::cout << "Total amount travelled: " << s.back() << '\n';
//  std::cout << car.d() << '\n';
//  avg_acc /= num_pts - 2;
//  if (avg_acc > 9.5) {
//    std::cout << avg_acc << '\n';
//  }
//  //std::cout << std::endl;
//  //double time_to_collision = check_collision(path, last_x, speed);
//  //if (time_to_collision > 0.1) {
//  //  std::cout << "Estimated time to collision at current speeds: " << time_to_collision << "s\n";
//  //}
//  //std::cout << "Final speed: " << speed << std::endl;
//  //num_prev = next_x_vals.size();
//  //if (num_prev > 1)
//  //  std::cout << "Calculated speed: " << distance(next_x_vals[num_prev - 1], next_y_vals[num_prev - 1], next_x_vals[num_prev - 2], next_y_vals[num_prev - 2]) / dt << std::endl << std::endl;
//  return {next_x_vals, next_y_vals};
//}
