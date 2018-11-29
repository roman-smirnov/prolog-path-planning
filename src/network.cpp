/*
================================================================================================================================

 network_gateway.cpp

 implementation of class NetworkGateway declared in network_gateway.h

================================================================================================================================
*/

#include "network.h"

using std::vector;
using std::cout;
using std::cerr;
using std::string;
using std::endl;

/*--------------------------------------------------------------
 Sets up the networking component
--------------------------------------------------------------*/
NetworkGateway::NetworkGateway() {
  init_simulator_networking();
  init_prolog_networking();
}


void NetworkGateway::set_controller(PlanController *plan_controller) {
  NetworkGateway::plan_controller = plan_controller;
}

/*--------------------------------------------------------------
 Setup simulator port listening, request callbacks, response handling, etc
 The simulator service is a websocket client, the function sets up a server for it to connect and send data to.
--------------------------------------------------------------*/
void NetworkGateway::init_simulator_networking() {
  websocket_hub.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
	// cout << "MESSAGE RECEIVED FROM SIMULATOR CLIENT" << endl;
	if (plan_controller!=nullptr) {
	  plan_controller->handle_simulator_message(data, length);
	}
  });

  websocket_hub.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	cout << "CONNECTED TO SIMULATOR CLIENT" << endl;
  });

  websocket_hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
	ws.close();
	cout << "DISCONNECTED FROM SIMULATOR CLIENT" << endl;
  });

  if (websocket_hub.listen(SIMULATOR_CLIENT_PORT)) {
	cout << "Listening to simulator on port " << SIMULATOR_CLIENT_PORT << endl;
  } else {
	cerr << "Failed to listen to simulator on port " << SIMULATOR_CLIENT_PORT << endl;
	exit(EXIT_FAILURE);
  }
}

/*--------------------------------------------------------------
 Setup Prolog connection, request callbacks, response handling, etc
 The Prolog service is a websocket server, the function sets up a client to open a websocket connectin and send data.
--------------------------------------------------------------*/
void NetworkGateway::init_prolog_networking() {
  websocket_hub.onError([](void *user) {
	cerr << "ERROR ON CONNECTION ATTEMPT TO LOGIC SERVER" << endl;
	// exit(EXIT_FAILURE);
  });

  websocket_hub.onConnection([](uWS::WebSocket<uWS::CLIENT> ws, uWS::HttpRequest req) {
	cout << "CONNECTED TO LOGIC SERVER " << endl;
  });

  websocket_hub.onMessage([this](uWS::WebSocket<uWS::CLIENT> ws, char *data, size_t length, uWS::OpCode opCode) {
	// cout << "MESSAGE RECEIVED FROM LOGIC SERVER" << endl;
	if (plan_controller!=nullptr) {
	  plan_controller->handle_prolog_message(data, length);
	}
  });

  websocket_hub.onDisconnection([](uWS::WebSocket<uWS::CLIENT> ws, int code, char *message, size_t length) {
	cerr << "DISCONNECTED FROM LOGIC SERVER " << endl;
	cerr << message << endl;
	exit(EXIT_FAILURE);
  });

  websocket_hub.connect(PROLOG_SERVER_ADDRESS, (void *) 8);
}

/*--------------------------------------------------------------
 Start simulator and Prolog networking
--------------------------------------------------------------*/
void NetworkGateway::start() {
  websocket_hub.run();
}


/*--------------------------------------------------------------
 Send a message to the simulator
--------------------------------------------------------------*/
void NetworkGateway::send_message_to_simulator(string &msg) {
  websocket_hub.getDefaultGroup<uWS::SERVER>().broadcast(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


/*--------------------------------------------------------------
 Send a message to the prolog server
--------------------------------------------------------------*/
void NetworkGateway::send_message_to_prolog(string &msg) {
  websocket_hub.getDefaultGroup<uWS::CLIENT>().broadcast(msg.data(), msg.length(), uWS::OpCode::TEXT);
}



