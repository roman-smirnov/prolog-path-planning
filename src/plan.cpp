/*
================================================================================================================================

 plan_controller.cpp

 Implementation of class PlanController declared in plan_controller.h

================================================================================================================================
*/

#include "plan.h"
#include "spline.h"
#include "trajectory.h"

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;


void PlanController::set_gateways(Network *network_gateway, MapGateway *map_gateway) {
  PlanController::network_gateway = network_gateway;
  PlanController::map_gateway = map_gateway;
}


void PlanController::handle_simulator_message(char *data, size_t length) {
  if (map_gateway==nullptr || network_gateway==nullptr) {
	return;
  }

  if (length <= 2 || data[0]!='4' || data[1]!='2') {
	return;
  }

  auto msg_str = has_data(data);
  if (msg_str.empty()) {
	string msg = "42[\"manual\",{}]"; // Manual driving
	network_gateway->send_message_to_simulator(msg);
	return;
  }
  auto j = nlohmann::json::parse(msg_str);
  auto event = j[0].get<string>();
  if (event!="telemetry") {
	return;
  }

  // ego vehicle localization Data
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_speed = j[1]["speed"];
  double end_path_d = j[1]["end_path_d"];
  double end_path_s = j[1]["end_path_s"];

  // a ist of all other cars on the same side of the road.
  vector<vector<int>> vehicles = j[1]["sensor_fusion"];

  auto velocity = map_gateway->velocity(car_speed, car_d, car_s, end_path_d, end_path_s);

  trajectory.set_telemetry(car_d, car_s);

  nlohmann::json msgJson;
  msgJson["pos_d"] = car_d;
  msgJson["pos_s"] = car_s;
  msgJson["vel_d"] = velocity[0];
  msgJson["vel_s"] = 22;
  msgJson["vehicles"] = vehicles;

  auto msg = msgJson.dump();
  network_gateway->send_message_to_prolog(msg);
}


void PlanController::handle_prolog_message(char *data, size_t length) {
  if (map_gateway==nullptr || network_gateway==nullptr) {
	return;
  }

  string in_msg(data, length);

  auto in_json = nlohmann::json::parse(in_msg);
  vector<double> next_d = in_json["next_d"];
  vector<double> next_s = in_json["next_s"];
  double time_inc = in_json["time_inc"];

  vector<double> next_vals_t;
  vector<double> next_vals_x;
  vector<double> next_vals_y;
  tk::spline spline_t_to_s;
  tk::spline spline_t_to_x;
  tk::spline spline_t_to_y;

  auto prev_d = trajectory.get_traj_d();
  auto prev_s = trajectory.get_traj_s();

  vector<double> traj_d;
  vector<double> traj_s;

  for (int i = 0; i < prev_s.size(); ++i) {
    if(prev_s.at(i) < trajectory.get_pos_s()) {
      traj_s.push_back(prev_s.at(i));
      traj_d.push_back(prev_d.at(i));
    }
  }

  traj_s.push_back(trajectory.get_pos_s());
  traj_d.push_back(trajectory.get_pos_d());

  for (int i = 0; i < next_s.size(); ++i) {
    if(next_s.at(i) > trajectory.get_pos_s()) {
      traj_s.push_back(next_s.at(i));
      traj_d.push_back(next_d.at(i));
    }
  }

  // define a path made up of (x,y) points that the car will visit sequentially every 0.02 seconds
  for (int t = 0; t < traj_s.size(); ++t) {
	vector<double> xy = map_gateway->frenet_to_xy(traj_s[t], traj_d[t]);
	next_vals_x.push_back(xy[0]);
	next_vals_y.push_back(xy[1]);
	next_vals_t.push_back(t*time_inc);
  }

  spline_t_to_s.set_points(next_vals_t, traj_s);
  spline_t_to_x.set_points(next_vals_t, next_vals_x);
  spline_t_to_y.set_points(next_vals_t, next_vals_y);
  next_vals_x.clear();
  next_vals_y.clear();

  trajectory.set_traj_d(next_d);
  trajectory.set_traj_s(next_s);

  auto num_values = traj_s.size()*time_inc/SIM_SAMPLE_TIME_INC;

  for (int t = 0; t < num_values; ++t) {
    if(spline_t_to_s(t*SIM_SAMPLE_TIME_INC)>trajectory.get_pos_s()){
      next_vals_x.push_back(spline_t_to_x(t*SIM_SAMPLE_TIME_INC));
      next_vals_y.push_back(spline_t_to_y(t*SIM_SAMPLE_TIME_INC));
    }
  }

  nlohmann::json out_json;
  out_json["next_x"] = next_vals_x;
  out_json["next_y"] = next_vals_y;
  auto out_msg = "42[\"control\"," + out_json.dump() + "]";
  network_gateway->send_message_to_simulator(out_msg);
}

string PlanController::has_data(string msg_str) {
  auto found_null = msg_str.find("null");
  auto b1 = msg_str.find_first_of('[');
  auto b2 = msg_str.find_first_of('}');
  if (found_null!=string::npos) {
	return "";
  } else if (b1!=std::string::npos && b2!=std::string::npos) {
	return msg_str.substr(b1, b2 - b1 + 2);
  }
  return "";
}