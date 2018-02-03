#ifndef MAP_WAYPOINTS_H
#define MAP_WAYPOINTS_H
#include <vector>
#include <utility>

struct Map {
  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  int num_wp; // number of waypoints
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;
  Map(); 
  int ClosestWaypoint(double x, double y) const;
  int NextWaypoint(double x, double y, double theta) const;
  std::pair<double,double> getFrenet(double x, double y, double theta) const;
  std::pair<double,double> getXY(double s, double d) const;
};

#endif
