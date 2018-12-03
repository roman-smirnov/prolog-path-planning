#include "network.h"

using namespace std;


int main() {
  MapGateway map_gateway;
  PlanController plan_controller;
  Network network_gateway;
  plan_controller.set_gateways(&network_gateway, &map_gateway);
  network_gateway.set_controller(&plan_controller);
  network_gateway.start();
}
