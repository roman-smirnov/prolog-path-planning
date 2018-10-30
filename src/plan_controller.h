/*
================================================================================================================================

 plan_controller.h

 Management, logic, message processing and output.

================================================================================================================================
*/

#ifndef PROLOG_PATH_PLANNING_PLANNING_CONTROLLER_H
#define PROLOG_PATH_PLANNING_PLANNING_CONTROLLER_H

#include "json.hpp"
#include "network_gateway.h"
#include "map_gateway.h"

class NetworkGateway;
class MapGateway;

class PlanController {
 public:
  PlanController() = default;

  // parse and handle messages received from simulator
  void handle_simulator_message(char *data, size_t length);

  // parse and handle messages received from prolog
  void handle_prolog_message(char *data, size_t length);

  // set the gateways to handle world map and networking
  void set_gateways(NetworkGateway *network_gateway, MapGateway *map_gateway);

 private:
  // distance between waypoints
  static constexpr double DISTANCE_INCREMENT = 0.5;

  // receive requests and send messages
  NetworkGateway *network_gateway = nullptr;

  // handle world representation related logic
  MapGateway *map_gateway = nullptr;

  // Checks if event has JSON data. return the message in strin format, or return an empty string if there's no message
  std::string has_data(std::string msg_str);

};

#endif //PROLOG_PATH_PLANNING_PLANNING_CONTROLLER_H
