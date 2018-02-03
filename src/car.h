#ifndef CAR_H
#define CAR_H
#include <cmath>
#include <utility>
#include "helper_functions.h"

// Main car
struct Car {
  // Car properties
  const double max_speed = 22; // m/s
  const double max_acc   = 10; // m/s^2
  const double max_jerk  = 10; // m/s^3
  Car() = default;
  Car(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) :
    x_(car_x), y_(car_y), s_(car_s), d_(car_d), yaw_(deg2rad(car_yaw)), speed_(car_speed/2.24), sin_t(std::sin(yaw_)), cos_t(std::cos(yaw_)) { }
  void set_values(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) {
    x_ = car_x;
    y_ = car_y;
    s_ = car_s;
    d_ = car_d;
    yaw_ = deg2rad(car_yaw);
    speed_ = car_speed/2.24;
    sin_t = std::sin(yaw_);
    cos_t = std::cos(yaw_);
  }
  // Transform from map coordinates to car coordinates
  std::pair<double, double> from_map(double map_x, double map_y) const {
    return {cos_t*(map_x-x_) + sin_t*(map_y-y_), -sin_t*(map_x-x_) + cos_t*(map_y-y_)};
  }
  // Transform from car coordinates to map coordinates
  std::pair<double, double> to_map(double car_x, double car_y) const {
    return {cos_t*car_x - sin_t*car_y + x_, sin_t*car_x + cos_t*car_y + y_};
  }
  const double& x() const { return x_; }
  const double& y() const { return y_; }
  const double& s() const { return s_; }
  const double& d() const { return d_; }
  const double& yaw() const { return yaw_; }
  const double& speed() const { return speed_; }
private:
  // Main car's localization Data
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;
  double sin_t;
  double cos_t;
  enum states {KL, PLCL, PLCR, LCL, LCR};
  states state = KL;
};

// Other cars on the road
struct Agent {
  const int id;
  const double x;
  const double y;
  const double vx;
  const double vy;
  const double s;
  const double d;
  const double speed;
  Agent(int id_, double x_, double y_, double vx_, double vy_, double s_, double d_) : 
    id(id_), x(x_), y(y_), vx(vx_), vy(vy_), s(s_), d(d_), speed(distance(vx, vy, 0, 0)) { }
};

#endif
