#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <utility>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = std::atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::pair<double,double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::pair<double,double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          std::vector<double> previous_path_x = j[1]["previous_path_x"];
          std::vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          int path_pts_num = 50;
          std::vector<double> next_x_vals(path_pts_num);
          std::vector<double> next_y_vals(path_pts_num);

          int pts_to_copy = std::min(10, int(previous_path_x.size()));
          std::vector<double> temp_x;
          std::vector<double> temp_y;
          if (previous_path_x.size() == 0) {
            temp_x.reserve(4);
            temp_y.reserve(4);
            temp_x.push_back(car_x);
            temp_y.push_back(car_y);
            pts_to_copy = 1;
            next_x_vals[0] = car_x;
            next_y_vals[0] = car_y;
          }
          else {
            temp_x.reserve(pts_to_copy+3);
            temp_y.reserve(pts_to_copy+3);
            temp_x.resize(pts_to_copy);
            temp_y.resize(pts_to_copy);
            std::copy(previous_path_x.begin(), previous_path_x.begin()+pts_to_copy, temp_x.begin());
            std::copy(previous_path_y.begin(), previous_path_y.begin()+pts_to_copy, temp_y.begin());
            std::copy(previous_path_x.begin(), previous_path_x.begin()+pts_to_copy, next_x_vals.begin());
            std::copy(previous_path_y.begin(), previous_path_y.begin()+pts_to_copy, next_y_vals.begin());
          }

          std::copy(previous_path_x.begin(), previous_path_x.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          std::copy(previous_path_y.begin(), previous_path_y.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          double last_x = temp_x.back();
          double last_y = temp_y.back();
          car_yaw = deg2rad(car_yaw);
          auto sd = getFrenet(last_x, last_y, car_yaw, map_waypoints_x, map_waypoints_y);
          for (int i = 0; i < 3; ++i) {
            auto xy = getXY(sd.first+(i+1)*15, 6.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            temp_x.push_back(xy.first);
            temp_y.push_back(xy.second);
          }

          std::copy(temp_x.begin(), temp_x.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          auto car_coords = map2car(car_x, car_y, car_yaw, temp_x, temp_y);
          temp_x = std::move(car_coords.first);
          temp_y = std::move(car_coords.second);
          std::copy(temp_x.begin(), temp_x.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          std::cout << car_x << " " << car_y << " " << car_yaw << std::endl;
          tk::spline path;
          path.set_points(temp_x, temp_y);
          auto map2car = [sin_t = std::sin(car_yaw), cos_t = std::cos(car_yaw), car_x, car_y](double x, double y) -> std::pair<double, double> {
            return {cos_t*(x-car_x) + sin_t*(y-car_y), -sin_t*(x-car_x) + cos_t*(y-car_y)};
          };
          auto car2map = [sin_t = std::sin(car_yaw), cos_t = std::cos(car_yaw), car_x, car_y](double x, double y) -> std::pair<double, double> {
            return {cos_t*x - sin_t*y + car_x, sin_t*x + cos_t*y + car_y};
          };

          auto last_xy = map2car(last_x, last_y);
          last_x = last_xy.first;
          last_y = last_xy.second;
          for (int i = pts_to_copy; i < path_pts_num; ++i) {
            double x = last_x+0.44*(i+1-pts_to_copy);
            //double x = 0.44*i;
            auto xy = car2map(x, path(x));
            next_x_vals[i] = xy.first;
            next_y_vals[i] = xy.second;
          }
          std::copy(next_x_vals.begin(), next_x_vals.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          std::copy(next_y_vals.begin(), next_y_vals.end(), std::ostream_iterator<double>(std::cout, " "));
          std::cout << '\n';
          std::cout << previous_path_x.size() << std::endl;
          std::cout << '\n';
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
//          for (int i = 0; i < 50; ++i) {
//            double s = car_s + 0.3*(i+1);
//            double d = car_d;
//            auto xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            double x = xy.first;
//            double y = xy.second;
//            next_x_vals.push_back(x);
//            next_y_vals.push_back(y);
//          }
          // END
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
