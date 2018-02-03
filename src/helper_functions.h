#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <vector>
#include <string>
#include <utility>
#include <cmath>

// For converting back and forth between radians and degrees.
constexpr double pi() { return 3.1415926535897932; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

std::string hasData(std::string s);
std::pair<std::vector<double>, std::vector<double>> map2car(double car_x, double car_y, double car_theta, const std::vector<double>& x, const std::vector<double>& y);
std::pair<std::vector<double>, std::vector<double>> car2map(double car_x, double car_y, double car_theta, const std::vector<double>& x, const std::vector<double>& y);
#endif
