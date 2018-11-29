/*
================================================================================================================================

 plan_controller.cpp

 Implementation of class PlanController declared in plan_controller.h

================================================================================================================================
*/

#include "plan.h"
#include "spline.h"

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;


void PlanController::set_gateways(NetworkGateway *network_gateway, MapGateway *map_gateway) {
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
	// Manual driving
	string msg = "42[\"manual\",{}]";
	network_gateway->send_message_to_simulator(msg);
	return;
  }

  auto j = nlohmann::json::parse(msg_str);
  auto event = j[0].get<string>();

  if (event!="telemetry") {
	return;
  }


  // Main car'msg_str localization Data
  double car_x = j[1]["x"];
  double car_y = j[1]["y"];
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_yaw = j[1]["yaw"];
  double car_speed = j[1]["speed"];



  // Previous path data given to the Planner
  auto previous_path_x = j[1]["previous_path_x"];
  auto previous_path_y = j[1]["previous_path_y"];
  // Previous path'msg_str end msg_str and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  // vector<vector<int>> vehicles = j[1]["sensor_fusion"];
  // for (int i = 0; i < vehicles.size(); ++i) {
  // for (int k = 0; k < vehicles[0].size() ; ++k) {
  //   vehicles[i][k] = round(vehicles[i][k]);
  // }
  // }

  cout << "car_d: " << car_d << " car_s: " << car_s << " car_speed: " << car_speed << endl;
  cout << "end_path_d: " << end_path_d << " end_path_s: " << end_path_s << endl;

  car_speed = car_speed < 49.0 ? car_speed : 49.0;
  vector<double> velocity = map_gateway->velocity(car_speed, car_d, car_s, end_path_d, end_path_s);

  nlohmann::json msgJson;
  msgJson["pos_d"] = car_d;//end_path_d > 0 ? end_path_d : car_d;
  msgJson["pos_s"] = car_s; //end_path_s > 0 ? end_path_s : car_s;
  msgJson["vel_d"] = velocity[0];
  msgJson["vel_s"] = velocity[1];
  // msgJson["vehicles"] = vehicles;
  auto msg = msgJson.dump();

  network_gateway->send_message_to_prolog(msg);

}


void PlanController::handle_prolog_message(char *data, size_t length) {
  if (map_gateway==nullptr || network_gateway==nullptr) {
	return;
  }

  string in_msg(data, length);

  auto in_json = nlohmann::json::parse(in_msg);
  double path_len = in_json["path_len"];
  double time_inc = in_json["time_inc"];
  vector<double> next_vals_d = in_json["next_d"];
  vector<double> next_vals_s = in_json["next_s"];
  vector<double> next_vals_t;
  vector<double> next_vals_x;
  vector<double> next_vals_y;
  tk::spline spline_t_to_x;
  tk::spline spline_t_to_y;

  // define a path made up of (x,y) points that the car will visit sequentially every 0.02 seconds
  for (int t = 0; t < path_len; ++t) {
	vector<double> xy = map_gateway->frenet_to_xy(next_vals_s[t], next_vals_d[t]);
	next_vals_x.push_back(xy[0]);
	next_vals_y.push_back(xy[1]);
	next_vals_t.push_back((double) t*time_inc);
  }

  spline_t_to_x.set_points(next_vals_t, next_vals_x);
  spline_t_to_y.set_points(next_vals_t, next_vals_y);
  next_vals_x.clear();
  next_vals_y.clear();

  auto num_values = path_len*time_inc/SIMULATOR_SAMPLE_TIME_INC;

  for (int t = 0; t < num_values; ++t) {
	next_vals_x.push_back(spline_t_to_x((double) t*SIMULATOR_SAMPLE_TIME_INC));
	next_vals_y.push_back(spline_t_to_y((double) t*SIMULATOR_SAMPLE_TIME_INC));
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
