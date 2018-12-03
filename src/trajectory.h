//
// Created by Roman Smirnov on 29/11/2018.
//

#ifndef PROLOG_PATH_PLANNING_TRAJECTORY_H
#define PROLOG_PATH_PLANNING_TRAJECTORY_H

#include <vector>

using std::vector;

class Trajectory {
 public:

  void set_telemetry(double car_d, double car_s);
  void set_traj_d(const vector<double> &traj_d);
  void set_traj_s(const vector<double> &traj_s);
  const vector<double> &get_traj_d() const;
  const vector<double> &get_traj_s() const;

  double get_pos_d() const;
  double get_pos_s() const;
  double get_vel_d() const;
  double get_vel_s() const;
  double get_acl_d() const;
  double get_acl_s() const;
  void set_vel_d(double car_vel_d);
  void set_vel_s(double car_vel_s);
  void set_acl_d(double car_acl_d);
  void set_acl_s(double car_acl_s);



 private:
  double car_pos_d = 0;
  double car_pos_s = 0;
  double car_vel_d = 0;
  double car_vel_s = 0;
  double car_acl_d = 0;
  double car_acl_s = 0;

  vector<double> traj_d;
  vector<double> traj_s;
};

#endif //PROLOG_PATH_PLANNING_TRAJECTORY_H
