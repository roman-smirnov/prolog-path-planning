/*
================================================================================================================================

 map.cpp

 Implementation of class MapGateway declared in map.h

================================================================================================================================
*/

#include "map.h"
#include "spline.h"
using std::vector;
using std::string;
using std::ifstream;
using std::istringstream;


/*--------------------------------------------------------------
 Loads and sets up the world map waypoint model
--------------------------------------------------------------*/
MapGateway::MapGateway() {
  load_map_from_file();
  interpolate_waypoints();
}


/*-----------------
 Load waypoints from map .csv file into waypoint coordinate vectors
--------------------------------------------------------------*/
void MapGateway::load_map_from_file() {
  ifstream in_map_(MAP_FILE_PATH, ifstream::in);
  string line;
  while (getline(in_map_, line)) {
	istringstream iss(line);
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
}


void MapGateway::interpolate_waypoints() {
  tk::spline spline_s_to_x;
  tk::spline spline_s_to_y;
  spline_s_to_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_s_to_y.set_points(map_waypoints_s, map_waypoints_y);
  map_waypoints_x.clear();
  map_waypoints_y.clear();
  map_waypoints_s.clear();

  for (int s = 0; s < MAX_S_COORD; ++s) {
	map_waypoints_s.push_back((double) s);
	map_waypoints_x.push_back(spline_s_to_x(s));
	map_waypoints_y.push_back(spline_s_to_y(s));
  }
}


/*--------------------------------------------------------------
 Convert degrees to radians
--------------------------------------------------------------*/
double MapGateway::deg2rad(double x) {
  return x*PI/180.0;
}


/*--------------------------------------------------------------
 Convert radians to degrees
--------------------------------------------------------------*/
double MapGateway::rad2deg(double x) {
  return x*180.0/PI;
}


/*--------------------------------------------------------------
 Calculate euclidean distance
--------------------------------------------------------------*/
double MapGateway::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}


/*--------------------------------------------------------------
 Calculate velocity x,y componenets
--------------------------------------------------------------*/
vector<double> MapGateway::velocity(double speed, double cur_d, double cur_s, double nxt_d, double nxt_s) {
  double dd = nxt_d - cur_d;
  double ds = nxt_s - cur_s;
  double vel_d = dd*speed*0.44704/(abs(dd) + abs(ds));
  double vel_s = ds*speed*0.44704/(abs(dd) + abs(ds));
  return {vel_d, vel_s};
}


/*--------------------------------------------------------------
 Calculate the closest map waypoint to given coordinates
--------------------------------------------------------------*/
int MapGateway::closest_waypoint(double x, double y) {
  double closestLen = HUGE_VAL; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_waypoints_x.size(); i++) {
	double map_x = map_waypoints_x[i];
	double map_y = map_waypoints_y[i];
	double dist = distance(x, y, map_x, map_y);
	if (dist < closestLen) {
	  closestLen = dist;
	  closestWaypoint = i;
	}
  }
  return closestWaypoint;
}


/*--------------------------------------------------------------
 Calculate the next map waypoint given current coordinates
--------------------------------------------------------------*/
int MapGateway::next_waypoint(double x, double y, double theta) {

  int closest_wp = closest_waypoint(x, y);

  double map_x = map_waypoints_x[closest_wp];
  double map_y = map_waypoints_y[closest_wp];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = fmin(2*PI - angle, angle);

  if (angle > PI/4) {
	closest_wp++;
	if (closest_wp==map_waypoints_x.size()) {
	  closest_wp = 0;
	}
  }
  return closest_wp;
}


/*--------------------------------------------------------------
 Convert map x,y coordinates to Frenet d,s coordinates
--------------------------------------------------------------*/
vector<double> MapGateway::xy_to_frenet(double x, double y, double theta) {
  int next_wp = next_waypoint(x, y, theta);

  int prev_wp = next_wp - 1;

  if (next_wp==0) {
	prev_wp = static_cast<int>(map_waypoints_x.size() - 1);
  }

  double n_x = map_waypoints_x[next_wp] - map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp] - map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - map_waypoints_x[prev_wp];
  double center_y = 2000 - map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
	frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
	frenet_s += distance(map_waypoints_y[i], map_waypoints_y[i], map_waypoints_x[i + 1], map_waypoints_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}


/*--------------------------------------------------------------
 Convert Frenet d,s coordinates to map x,y coordinates
--------------------------------------------------------------*/
vector<double> MapGateway::frenet_to_xy(double s, double d) {
  int prev_wp = -1;

  while (s > map_waypoints_s[prev_wp + 1] && (prev_wp < (int) (map_waypoints_s.size() - 1))) {
	prev_wp++;
  }

  auto wp2 = (prev_wp + 1)%map_waypoints_x.size();

  double heading = atan2((map_waypoints_y[wp2] - map_waypoints_y[prev_wp]), (map_waypoints_x[wp2] - map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp] + seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp] + seg_s*sin(heading);

  double perp_heading = heading - PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x, y};
}