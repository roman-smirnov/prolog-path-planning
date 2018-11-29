/*
================================================================================================================================

 network_gateway.h

 Simulator and Prolog networking

================================================================================================================================
*/


#ifndef PROLOG_PATH_PLANNING_NETWORK_GATEWAY_H
#define PROLOG_PATH_PLANNING_NETWORK_GATEWAY_H

#include "plan.h"

#include <uWS/Hub.h>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "json.hpp"

class PlanController;

// handles simulator and prolog networking via websockets
class NetworkGateway {
 public:
  // sets up the networking component
  NetworkGateway();

  // start the server, connect to client, begin handling requests and responses, etc
  void start();

  // set the controller to handle request callbacks
  void set_controller(PlanController *plan_controller);

  // send a message to the simulator
  void send_message_to_simulator(std::string &msg);

  // send a message to the prolog server
  void send_message_to_prolog(std::string &msg);

 private:
  // prolog server localhost ip address and port
  static constexpr auto PROLOG_SERVER_ADDRESS = "ws://127.0.0.1:8083/ws";

  // simulator client localhost port
  static constexpr int SIMULATOR_CLIENT_PORT = 4567;

  // management and logic object, processes incoming messages and issues output commands.
  PlanController *plan_controller = nullptr;

  // websocket handling and managment
  uWS::Hub websocket_hub;

  // sets up simulator networing
  void init_simulator_networking();

  // sets up prolog networking
  void init_prolog_networking();

};

#endif //PROLOG_PATH_PLANNING_NETWORK_GATEWAY_H
