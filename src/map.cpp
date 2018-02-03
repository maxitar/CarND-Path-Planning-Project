#include "map.h"
#include "helper_functions.h"
#include <vector>
#include <string>
#include <utility>
#include <fstream>
#include <sstream>
#include <cmath>

Map::Map() {
  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  map_waypoints_x.shrink_to_fit();
  map_waypoints_y.shrink_to_fit();
  map_waypoints_s.shrink_to_fit();
  map_waypoints_dx.shrink_to_fit();
  map_waypoints_dy.shrink_to_fit();
  num_wp = map_waypoints_x.size();
}

int Map::ClosestWaypoint(double x, double y) const {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < num_wp; i++) {
    double dist = distance(x,y,map_waypoints_x[i],map_waypoints_y[i]);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta) const {

  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = map_waypoints_x[closestWaypoint];
  double map_y = map_waypoints_y[closestWaypoint];
  double heading = std::atan2((map_y-y),(map_x-x));
  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == num_wp) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::pair<double,double> Map::getFrenet(double x, double y, double theta) const {
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = num_wp-1;
  }

  double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-map_waypoints_x[prev_wp];
  double center_y = 2000-map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::pair<double,double> Map::getXY(double s, double d) const {
  int prev_wp = -1;

  int num_wp_s = map_waypoints_s.size();
  while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(num_wp_s - 1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%num_wp;

  double heading = std::atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}
