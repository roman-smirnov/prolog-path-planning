/*
================================================================================================================================

 map_gateway.h

 World map model and various related functionality

================================================================================================================================
*/

#ifndef PROLOG_PATH_PLANNING_MAP_GATEWAY_H
#define PROLOG_PATH_PLANNING_MAP_GATEWAY_H

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// holds the world map model and encapsulates various related functionality
class MapGateway {
 public:
  // loads and sets up the world map waypoint model
  MapGateway();

  // convert degrees to radians
  double deg2rad(double x);

  // convert radians to degrees
  double rad2deg(double x);

  // calculate distance between coordinates
  double distance(double x1, double y1, double x2, double y2);

  std::vector<double> velocity(double speed, double cur_d, double cur_s, double nxt_d, double nxt_s);

  // convert map x,y coordinates to Frenet d,s coordinates
  std::vector<double> xy_to_frenet(double x, double y, double theta);

  // convert Frenet d,s coordinates to map x,y coordinates
  std::vector<double> frenet_to_xy(double s, double d);

 private:
  // path to file from which to read waypoint map
  static constexpr auto MAP_FILE_PATH = "../data/highway_map.csv";

  // max s value before wrapping around the track back to 0
  static constexpr double MAX_S_COORD = 6945.554;

  // cpp doesn't have a built in PI constant!
  static constexpr double PI = 3.14159265358979323846;

  // map values for coordinates x,y,s,d as normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // load spare waypoints from .csv file into waypoint coordinate vectors
  void load_map_from_file();

  // calculate the closest waypoint to given coordinates
  int closest_waypoint(double x, double y);

  // calculate the next waypoint to given current coordinates
  int next_waypoint(double x, double y, double theta);

  void interpolate_waypoints();
};

#endif //PROLOG_PATH_PLANNING_MAP_GATEWAY_H
