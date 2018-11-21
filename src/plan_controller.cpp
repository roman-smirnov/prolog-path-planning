/*
================================================================================================================================

 plan_controller.cpp

 Implementation of class PlanController declared in plan_controller.h

================================================================================================================================
*/

#include "plan_controller.h"

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

using json = nlohmann::json;


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

  auto j = json::parse(msg_str);
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
  vector<vector<int>> vehicles = j[1]["sensor_fusion"];


  // for (int i = 0; i < vehicles.size(); ++i) {
  // for (int k = 0; k < vehicles[0].size() ; ++k) {
  //   vehicles[i][k] = round(vehicles[i][k]);
  // }
  // }

  json msgJson;
  msgJson["vehicles"] = vehicles;
  msgJson["car_s"] = car_s;
  msgJson["car_d"] = car_d;
  msgJson["car_speed"] = car_speed;


  auto msg = msgJson.dump();
  network_gateway->send_message_to_prolog(msg);

}


void PlanController::handle_prolog_message(char *data, size_t length) {
  if (map_gateway==nullptr || network_gateway==nullptr) {
	return;
  }

  string in_msg(data, length);
  auto in_json = json::parse(in_msg);

  vector<double> next_x_vals = in_json["next_d"];
  vector<double> next_y_vals = in_json["next_s"];

  // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  for (int i = 0; i < 2; ++i) {
	vector<double> xy = map_gateway->frenet_to_xy(next_y_vals[i], next_x_vals[i]);
	next_x_vals[i] = xy[0];
	next_y_vals[i] = xy[1];
  }

  json out_json;
  out_json["next_x"] = next_x_vals;
  out_json["next_y"] = next_y_vals;

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
