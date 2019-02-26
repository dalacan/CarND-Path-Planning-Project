#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <string>
#include <vector>
#include "json.hpp"
#include "vehicle.h"
#include "highway.h"

using nlohmann::json;
using std::string;
using std::vector;

class Planner {
public:
  explicit Planner(Highway &highway);

  void SetParameters(int number_lanes, double lane_size, double speed_limit, double max_acceleration, double max_jerk);


  // Get the ego vehicle
  Vehicle GetEgo();
  void SetEgo(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, json &previous_path_x, json &previous_path_y);

  // Add detected vehicles
  void UpdateDetectedVehicles(json sensor_fusion);

  void GeneratePredictions(int horizon = 5);
  void PlanBehavior();

  vector<vector<double>> GenerateTrajectory();

  vector<double> lane_speeds;

  Vehicle ego;
  Vehicle ego_next_state;
  vector<Vehicle> detected_vehicles;
  map<int ,vector<Vehicle>> predictions;

  double ref_vel = 0;


  // Highway variables
  int num_lanes = 3;
  double speed_limit = 49.5/2.24; // mph
  double lane_size = 4.0f;

  // Limits
  double max_acceleration = 10.0f; // m/s^2
  double max_jerk = 10.0f; // m/s^3

  // Vehicle variables
  constexpr static double time_per_point = 0.02f; // 0.02 seconds per point
  constexpr static int points_per_path = 50; // Number of points per path

  Highway highway;

  json previous_path_x;
  json previous_path_y;
  int previous_path_size;

};

#endif //PATH_PLANNING_HIGHWAY_DRIVING_H
