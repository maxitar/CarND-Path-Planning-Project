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
  std::vector<Agent> agents;
  const double dt = 0.02; // The time step of the simulator is 0.02 seconds
  const int num_lanes = 3;
  const double lanes[3] = { 2.0, 6.0, 10.0 }; // The center of each lane
  class FSM {
    enum states {KL, PLC, LC}; // Valid states are Keep Lane (KL), Prepare Lane Change (PLC), and Lane Change (LC)
    states state = KL;
    int origin_lane = 1; // The lane the car starts a manoeuvre from 
    int target_lane = 1; // The target lane for the current manoeuvre
    int final_lane  = 1; // The ultimate target lane
    int steps_in_lane = 0; // Number of time steps that the car was in KL state
    double target_speed = 22.;
    TrajectoryGenerator& gen;
    // Updates the finite state machine
    void updateState();
  public:
    FSM(TrajectoryGenerator& generator) : gen(generator) {}
    // Returns the target lane and target speed
    std::pair<int,double> getLaneSpeed();
  } fsm{*this};

  // Check for collisions along a trajectory
  // Returns {collision_detected, time_to_collision, speed_of_agent_in_collision}
  std::tuple<bool,double,double> checkCollision(const std::vector<double>& path_x, const std::vector<double>& path_y);
  // Return a spline with the trajectory into 'target_lane' 
  // using 'num_pts_prev' from the previous trajectory 
  tk::spline getPath(int target_lane, int num_pts_prev);
  std::pair<std::vector<double>, std::vector<double>> generateTrajectory
    (int target_lane, double target_velocity, int pts_to_copy);
  std::array<double, 3> getLaneSpeeds(double front_distance = 50., double back_distance = 20.);
  std::pair<double, double> findClosestCarInLane();
  std::pair<double, double> findNextCarInLane(int lane, bool in_front);
public:
  TrajectoryGenerator(const Map& map_) : map(map_) {};
  // Update the data for the generator
  void update(const nlohmann::json& telemetry);
  // Compute and return the optimal trajectory
  std::pair<std::vector<double>, std::vector<double>> getOptimalTrajectory();
};

#endif
