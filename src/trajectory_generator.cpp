#include "trajectory_generator.h"
#include <algorithm>
#include "json.hpp"

// compute s1-s2 for s1,s2 in [0, s_max] 
double subtract_s(double s1, double s2, double s_max = 6945.554) {
  double diff = s1 - s2;
  if (std::fabs(diff) < 0.5*s_max) return diff;
  return s_max - diff;
}

// Check if two cars collide based on their Frenet coordinates and safety margins
bool collision(double car_s, double car_d, double agent_s, double agent_d, double margin_s = 5., double margin_d = 2.5) {
  double distance = subtract_s(car_s, agent_s);
  return (std::fabs(distance) < margin_s && std::fabs(car_d-agent_d) < margin_d);
}

// Returns the lane of a car based on its d-coordinate
int getLane(double d) {
  int result = int(d) / 4;
  result = result > 2 ? 2 : result;
  result = result < 0 ? 0 : result;
  return result;
}

// Returns the fastest lane based on 'lane_speed'
int fastestLane(const std::array<double, 3>& lane_speed) {
  return std::distance(std::begin(lane_speed), std::max_element(std::begin(lane_speed), std::end(lane_speed)));
}

void TrajectoryGenerator::FSM::updateState() {
  std::array<double, 3> lane_speeds = gen.getLaneSpeeds(50, 3);
  int fastest_lane = fastestLane(lane_speeds);

  auto front_vehicle = gen.findClosestCarInLane();
  double front_dist  = front_vehicle.first;
  double front_speed = front_vehicle.second;
  // Keep lane
  if (state == KL) {
    steps_in_lane += steps_in_lane < 100 ? 1 : 0;
    // Change lane only if the car was already 100 timesteps (2 seconds) in this state
    // and also there is at least 1mps difference between current lane and fastest lane
    if (steps_in_lane == 100 && lane_speeds[fastest_lane] > lane_speeds[origin_lane] + 1.0) {
      // If the current lane and fastest lane are not adjacent select the lane between them as target
      if (std::abs(fastest_lane - origin_lane) == 2) target_lane = 1;
      else target_lane = fastest_lane;
      final_lane = fastest_lane;
      state = PLC;
    } else {
      // Keep at least 10 meter distance to the front car
      if (front_dist < 10) {
        target_speed = front_speed - 2;
      }
      // If the car is [10, 20) meters from front car, keep front's car speed
      else if (front_dist < 20) {
        target_speed = front_speed;
      }
      // Otherwise go full speed
      else {
        target_speed = gen.car.max_speed;
      }
    }
  }
  // Get target lane nearest front car
  auto tl_fc = gen.findNextCarInLane(target_lane, true);
  double tl_front_dist = tl_fc.first;
  double tl_front_speed = tl_fc.second;

  // Get target lane nearest behind car
  auto tl_bc = gen.findNextCarInLane(target_lane, false);
  double tl_back_dist = tl_bc.first;
  double tl_back_speed = tl_bc.second;
  // Prepare lane change
  if (state == PLC) {
    // Keep at least 10 meter distance to the front car
    if (front_dist < 10) {
      target_speed = front_speed - 2;
      return;
    }
    // If the fastest lane changed or the current lane's speed is within 1mps
    // cancel the overtaking manoeuvre and go back to keep lane state
    if (lane_speeds[final_lane] < lane_speeds[origin_lane] + 1.0 || final_lane != fastest_lane) {
      target_lane = origin_lane;
      final_lane = origin_lane;
      state = KL;
      return;
    }
    // If the car has enough distance in the target lane in front and in back
    // and also no agent will come within 10 meters in the next 3 seconds
    // given current speeds, then go to LC state
    if (tl_back_dist > 10 && -tl_back_dist + 3*(tl_back_speed - gen.car.speed()) < -10) {
      if (tl_front_dist > 10 && tl_front_dist + 3*(tl_front_speed - gen.car.speed()) > 10) {
        state = LC;
      }
      else {
        if (front_dist < 20) {
          if (front_speed-tl_front_speed < 1.) { 
            target_speed = std::min(front_speed, tl_front_speed - 1.);
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
        if (front_dist + tl_back_dist < 20 && front_speed-tl_back_speed < 1.) { 
          target_speed = std::min(front_speed, tl_back_speed - 1.);
        } else { 
          target_speed = front_speed;
        }
      } else {
        target_speed = gen.car.max_speed;
      }
    }
  }
  // Lane change
  if (state == LC) {
    // If the car is still not fully in its target lane, it need to mind its distance
    // to the agents from both the origin lane and the target lane
    if (std::fabs(gen.car.d()-gen.lanes[target_lane]) > 1.) {
      if (front_dist < 10) {
        target_speed = std::min(front_speed, tl_front_speed);
      }
      else {
        target_speed = tl_front_dist < 30 ? tl_front_speed : gen.car.max_speed;
      }
    } // Otherwise either reset into Keep Lane state or prepare to change into final lane
    else {
      target_speed = tl_front_dist < 30 ? tl_front_speed : gen.car.max_speed;
      origin_lane = target_lane;
      if (target_lane == final_lane) {
        steps_in_lane = 0;
        state = KL;
      } else {
        target_lane = final_lane;
        state = PLC;
      }
    }
  }
}

std::pair<int,double> TrajectoryGenerator::FSM::getLaneSpeed() {
  updateState();
  if (state == KL || state == PLC) return {origin_lane, target_speed};
  else return {target_lane, target_speed};
}

std::tuple<bool, double, double> TrajectoryGenerator::checkCollision(const std::vector<double>& path_x, const std::vector<double>& path_y) {
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

  double final_s = 100.; // The distance between the last point taken from the previous 
                        // trajectory and the final point used for the spline construction
  double step_distance = 33.; // The distance between the new points
  int steps = final_s / step_distance;
  double final_d = lanes[target_lane];

  if (prev_path_x.size() == 0) {
    // If the previous path is empty, initialize only with the car coordinates
    temp_x.reserve(4);
    temp_y.reserve(4);
    temp_x.push_back(car.x());
    temp_y.push_back(car.y());
  }
  else {
    // Otherwise copy the 'num_pts_prev' number of points from the previous path
    temp_x.reserve(num_pts_prev + steps);
    temp_y.reserve(num_pts_prev + steps);
    std::copy(prev_path_x.begin(), prev_path_x.begin() + num_pts_prev, std::back_inserter(temp_x));
    std::copy(prev_path_y.begin(), prev_path_y.begin() + num_pts_prev, std::back_inserter(temp_y));
  }

  // Take the last point copied from the previous path as a reference for the new points
  double last_x = temp_x.back();
  double last_y = temp_y.back();
  auto sd = map.getFrenet(last_x, last_y, car.yaw());
  double current_d = sd.second;
  double step_d = (final_d - current_d);
  for (int i = 0; i < steps; ++i) {
    current_d += step_d;
    current_d = std::fabs(current_d - final_d) < 0.5 ? final_d : current_d;
    //auto xy = map.getXY(sd.first + (i + 1)*step_distance, current_d);
    auto xy = map.getXY(sd.first + (i + 1)*step_distance, final_d);
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
  int path_pts_num = 100; // 50 points for 1 second of driving
  double target_v = target_velocity; //meters per second
  std::vector<double> next_x_vals;
  next_x_vals.reserve(path_pts_num);
  std::vector<double> next_y_vals;
  next_y_vals.reserve(path_pts_num);

  if (prev_path_x.size() == 0) {
    pts_to_copy = 1;
    next_x_vals.push_back(car.x());
    next_y_vals.push_back(car.y());
  }
  else {
    std::copy(prev_path_x.begin(), prev_path_x.begin() + pts_to_copy, std::back_inserter(next_x_vals));
    std::copy(prev_path_y.begin(), prev_path_y.begin() + pts_to_copy, std::back_inserter(next_y_vals));
  }

  tk::spline path = getPath(target_lane, pts_to_copy);
  double last_x = next_x_vals.back();
  double last_y = next_y_vals.back();
  std::tie(last_x, last_y) = car.from_map(last_x, last_y);
  double speed = car.speed();
  int num_prev = next_x_vals.size();
  if (num_prev > 1) {
    speed = distance(next_x_vals[num_prev - 1], next_y_vals[num_prev - 1], next_x_vals[num_prev - 2], next_y_vals[num_prev - 2]) / dt;
  }
  double x_diff = 1.;
  double x_target = last_x + x_diff;
  double y_target = path(x_target);
  double len = distance(last_x, last_y, x_target, y_target);
  double l = 0;
  double acc_coef = getLane(car.d()) == target_lane ? 0.8 : 0.1;
  double vel_step = car.max_acc*acc_coef*dt;
  auto same_lane = [target_d = lanes[target_lane]](double d) { return d > target_d - 0.5 && d < target_d + 0.5; };
  for (int i = pts_to_copy; i < path_pts_num; ++i) {
    if (speed < target_v - vel_step) speed += vel_step;
    if (speed > target_v + vel_step) speed -= vel_step;
    if (speed > target_v - vel_step && speed < target_v + vel_step) speed = target_v;
    double dl = speed * dt; 
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

// find the nearest agent in front of or behind the car in 'lane' and return its distance and speed
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

// Update the data for the generator
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
  std::pair<int, double> lane_speed = fsm.getLaneSpeed();
  auto trajectory = generateTrajectory(lane_speed.first, lane_speed.second, 30);
  auto collision = checkCollision(trajectory.first, trajectory.second);
  bool collision_detected = std::get<0>(collision);
  double time_to_collision = std::get<2>(collision);
  if (collision_detected && time_to_collision < 2.) {
    double agent_speed = std::get<1>(collision);
    double target_speed = std::min(agent_speed, lane_speed.second);
    return generateTrajectory(lane_speed.first, target_speed, 5);
  }
  return trajectory;
}
