//
// Created by Roman Smirnov on 29/11/2018.
//

#include "trajectory.h"


void Trajectory::set_telemetry(double car_d, double car_s) {
  car_pos_d = car_d;
  car_pos_s = car_s;
}


double Trajectory::get_pos_d() const {
  return car_pos_d;
}


double Trajectory::get_pos_s() const {
  return car_pos_s;
}


double Trajectory::get_vel_d() const {
  return car_vel_d;
}


double Trajectory::get_vel_s() const {
  return car_vel_s;
}


double Trajectory::get_acl_d() const {
  return car_acl_d;
}


double Trajectory::get_acl_s() const {
  return car_acl_s;
}


void Trajectory::set_vel_d(double car_vel_d) {
  Trajectory::car_vel_d = car_vel_s;
}


void Trajectory::set_vel_s(double car_vel_s) {
  Trajectory::car_vel_s = car_vel_s;
}


void Trajectory::set_acl_d(double car_acl_d) {
  Trajectory::car_acl_d = car_acl_d;
}


void Trajectory::set_acl_s(double car_acl_s) {
  Trajectory::car_acl_s = car_acl_s;


}

void Trajectory::set_traj_d(const vector<double> &traj_d) {
  Trajectory::traj_d = traj_d;
}

void Trajectory::set_traj_s(const vector<double> &traj_s) {
  Trajectory::traj_s = traj_s;
}


const vector<double> &Trajectory::get_traj_d() const {
  return traj_d;
}


const vector<double> &Trajectory::get_traj_s() const {
  return traj_s;
}





