#include "helper_functions.h"
#include <cmath>

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


std::pair<std::vector<double>, std::vector<double>> map2car(double car_x, double car_y, double car_theta, const std::vector<double>& x, const std::vector<double>& y) {
  double cos_t = std::cos(-car_theta);
  double sin_t = std::sin(-car_theta);

  int n = x.size();
  std::vector<double> new_x(n);
  std::vector<double> new_y(n);

  for (int i = 0; i < n; ++i) {
    double x_0 = x[i]-car_x;
    double y_0 = y[i]-car_y;
    new_x[i] = x_0*cos_t - y_0*sin_t;// - car_x;
    new_y[i] = x_0*sin_t + y_0*cos_t;// - car_y;
  }

  return {std::move(new_x), std::move(new_y)};
}

std::pair<std::vector<double>, std::vector<double>> car2map(double car_x, double car_y, double car_theta, const std::vector<double>& x, const std::vector<double>& y) {
  double cos_t = std::cos(car_theta);
  double sin_t = std::sin(car_theta);

  int n = x.size();
  std::vector<double> map_x(n);
  std::vector<double> map_y(n);

  for (int i = 0; i < n; ++i) {
    map_x[i] = x[i]*cos_t - y[i]*sin_t + car_x;
    map_y[i] = x[i]*sin_t + y[i]*cos_t + car_y;
  }

  return {std::move(map_x), std::move(map_y)};
}

