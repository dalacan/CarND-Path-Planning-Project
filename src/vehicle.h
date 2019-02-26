#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "json.hpp"

using std::map;
using std::string;
using std::vector;
using nlohmann::json;

class Vehicle {
public:
  // Constructors
  Vehicle();

  // For ego vehicle
  Vehicle(double x, double y, double s, double d, double yaw, double v, int lane, string state="CS");
  void Configure(double num_lanes, double target_speed, double max_acceleration);

  // For sensor fusion vehicle
  Vehicle(int id, double x, double y, double s, double d, double v, int lane, string state="CS");

  // For prediction and trajectory
  Vehicle(int lane, double s, double v, double a, string state);


  vector<Vehicle> GeneratePredictions(int horizon = 5);
  double PositionAt(double t);
  double VelocityAt(double t);

  vector<Vehicle> ChooseNextState(map<int, vector<Vehicle>> &predictions);
  vector<string> SuccessorStates();
  vector<Vehicle> GenerateTrajectory(string state, map<int, vector<Vehicle>> &predictions);

  // Trajectories
  vector<Vehicle> ConstantSpeedTrajectory();
  vector<Vehicle> KeepLaneTrajectory(map<int, vector<Vehicle>> &predictions);
  vector<Vehicle> LaneChangeTrajectory(string state, map<int, vector<Vehicle>> &predictions);
  vector<Vehicle> PrepLaneChangeTrajectory(string state, map<int, vector<Vehicle>> &predictions);

  vector<double> GetKinematics(map<int, vector<Vehicle>> &predictions, int lane);
  bool GetVehicleBehind(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle);
  bool GetVehicleBehind(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle, Vehicle &ref_vehicle);
  bool GetVehicleAhead(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle);

  string FindGap(map<int, vector<Vehicle>> &predictions, int lane, double &gap_s);

  void RealizeNextState(vector<Vehicle> &trajectory);

  // Destructor
  virtual ~Vehicle();

  int id, lane;

  double x, y, s, d, v, a, yaw, target_speed, max_acceleration, num_lanes;

  int preferred_buffer = 30; // impacts "keep lane" behavior.

  string state;

  double max_s = 6945.554;

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1},
                                     {"LCR", 1}, {"PLCR", 1}};

  vector<string> gap_locations = {"FRONT", "BESIDE", "BACK"};
};

#endif  // VEHICLE_H