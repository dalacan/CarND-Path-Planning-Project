#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <iostream>

using std::string;
using std::vector;

const float EFFICIENCY = pow(10, 5);
const float LANE_CHANGE = pow(10, 5);
const float OUT_OF_LANE = pow(10, 7);
const float CHANGE_LANE_SAFE = pow(10,5);

float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  // This function is very similar to what you have already implemented in
  //   the "Implement a Second Cost Function in C++" quiz.
  Vehicle vehicle_ahead;
  float proposed_speed_intended = -1;
  if(get_vehicle_ahead(predictions, data["intended_lane"], vehicle_ahead)) {
    proposed_speed_intended = vehicle_ahead.v;
  }

  std::cout << "lane: " << data["intended_lane"] << " proposed_speed_intended: " << proposed_speed_intended << std::endl;
  if (proposed_speed_intended < 0 || proposed_speed_intended > vehicle.target_speed) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = -1;
  if(get_vehicle_ahead(predictions, data["final_lane"], vehicle_ahead)) {
    proposed_speed_final = vehicle_ahead.v;
  }
  std::cout << "lane: " << data["final_lane"] << " proposed_speed_final: " << proposed_speed_final << std::endl;
  if (proposed_speed_final < 0 || proposed_speed_final > vehicle.target_speed) {
    proposed_speed_final = vehicle.target_speed;
  }

  float cost = (2.0*vehicle.target_speed - proposed_speed_intended
                - proposed_speed_final)/vehicle.target_speed;

  std::cout << "vehicle.target_speed: " << vehicle.target_speed << std::endl;

  std::cout << "inefficiency_cost: " << cost << std::endl;

  return cost;
}

float lane_change_cost(const Vehicle &vehicle,
                       const vector<Vehicle> &trajectory,
                       const map<int, vector<Vehicle>> &predictions,
                       map<string, float> &data) {

  Vehicle vehicle_ahead;
  double vehicle_ahead_distance = 9999;
  if(get_vehicle_ahead(predictions, vehicle.lane, vehicle_ahead)) {
    if(vehicle_ahead.s < 100 && vehicle.s > vehicle.max_s - 100) {
      vehicle_ahead_distance = vehicle_ahead.s + vehicle.max_s - vehicle.s;
    } else {
      vehicle_ahead_distance = vehicle_ahead.s - vehicle.s;
    }
  }
  std::cout << "vehicle_ahead_distance: " << vehicle_ahead_distance << std::endl;
  std::cout << "Vehicle lane: " << vehicle.lane << " intended lane: " << data["intended_lane"]  << " final lane: " << data["final_lane"] << std::endl;

  double intended_lane_vehicle_ahead_distance = 9999;
  if(get_vehicle_ahead(predictions, (int)data["intended_lane"], vehicle_ahead)) {
    intended_lane_vehicle_ahead_distance = vehicle_ahead.s - vehicle.s;
  }

  float cost = (double)abs(vehicle.lane - data["intended_lane"]) * 0.5 * ((1 - exp(-vehicle_ahead_distance/300.0)) + exp(-intended_lane_vehicle_ahead_distance/30.0));

  std::cout << "lane_change_cost: " << cost << std::endl;

  return cost;

}

float out_of_lane_cost(const Vehicle &vehicle,
                       const vector<Vehicle> &trajectory,
                       const map<int, vector<Vehicle>> &predictions,
                       map<string, float> &data) {

  float cost = 0;
  if(data["intended_lane"] >= vehicle.num_lanes || data["intended_lane"] < 0) {
    cost = 1;
  }

  std::cout << "out_of_lane_cost: " << cost << std::endl;

  return cost;
}

float change_lane_safe_cost(const Vehicle &vehicle,
                            const vector<Vehicle> &trajectory,
                            const map<int, vector<Vehicle>> &predictions,
                            map<string, float> &data) {

  float cost = 0;
  if(vehicle.lane != data["intended_lane"]  ||  vehicle.lane != data["final_lane"]) {
    cost = 1;
    float min_gap_size = 15;
    double gap_dist = 1;
    // Attempt to find a next to vehicle
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    bool vehicle_ahead_bool = get_vehicle_ahead(predictions, (int)data["intended_lane"], vehicle_ahead);
    bool vehicle_behind_bool = get_vehicle_behind(predictions, (int)data["intended_lane"], vehicle_behind);

    // Only calculate cost for lane change
    if(!vehicle_ahead_bool && !vehicle_behind_bool) {
      // No vehicle in intended lane
      cost = 0;
      std::cout << "gap found - No vehicle ahead of behind" << std::endl;
    }
    else if(vehicle_ahead_bool && !vehicle_behind_bool) {
      // Vehicle ahead but no vehicle behind, check if distance to closest vehicle in front has sufficient gap relative to ego
      if(vehicle_ahead.s < 300 && vehicle.s > vehicle.max_s - 300) {
        // Handle wrap aground
        gap_dist = vehicle_ahead.s + vehicle.max_s - vehicle.s;
      } else {
        gap_dist = vehicle_ahead.s - vehicle.s;
      }

      if(gap_dist > min_gap_size/2.0) {
        cost = 1/gap_dist;
        std::cout << "gap found - Vehicle ahead but no vehicle behind" << std::endl;
      }
    } else if(vehicle_behind_bool && !vehicle_ahead_bool) {
      // Vehicle behind but no vehicle ahead, check if distance to closest vehicle behind has sufficient gap to ego relative to ego
      if(vehicle.s < 300 && vehicle_behind.s > vehicle.max_s - 300) {
        // Handle wrap aground
        gap_dist = vehicle.s + vehicle.max_s - vehicle_behind.s;
      } else {
        gap_dist = vehicle.s - vehicle_behind.s;
      }

      if(gap_dist > min_gap_size/2.0) {
        cost = 1/gap_dist;
        std::cout << "gap found - Vehicle behind but no vehicle ahead" << std::endl;
      }
    } else if(vehicle_ahead_bool && vehicle_behind_bool && (vehicle_ahead.s - vehicle_behind.s)> min_gap_size) {
      // Vehicle ahead and behind, check if gap between both vehicle have a sufficient gap
      if(vehicle_ahead.s < 300 && vehicle_behind.s > vehicle_behind.max_s - 300) {
        // Handle wrap aground
        gap_dist = vehicle_ahead.s + vehicle.max_s - vehicle_behind.s;
      } else {
        gap_dist = vehicle_ahead.s - vehicle_behind.s;
      }

      if(gap_dist > min_gap_size) {
        cost = 1 / gap_dist;
        std::cout << "gap found - Vehicle ahead and vehicle behind" << std::endl;
      }
    }
  }

  std::cout << "change_lane_safe_cost: " << cost << std::endl;

  return cost;
}

bool get_vehicle_ahead(const map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle) {
  //   Get the speed for the closest car in front
  bool found_vehicle = false;
  Vehicle temp_vehicle;

  Vehicle ego = predictions.at(-1)[0];
  double min_s = ego.max_s*2;

  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    int key = it->first;
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && key != -1) {
      // Handle wrap around
      if(ego.s > ego.max_s - 300 && temp_vehicle.s < 300
         && (temp_vehicle.s+ego.max_s) < min_s) {
        min_s = temp_vehicle.s+ego.max_s; // for wraparound
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
      else if(temp_vehicle.s > ego.s && temp_vehicle.s < min_s) {
        min_s = temp_vehicle.s;
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
    }
  }

  // Found no vehicle in the lane
  return found_vehicle;
}

bool get_vehicle_behind(const map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle) {
  Vehicle ego = predictions.at(-1)[0];
  double max_s = -ego.max_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;

  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == ego.lane) {
      // Handle wrap around
      if(temp_vehicle.s > max_s - 300 && ego.s < 300 && (temp_vehicle.s-ego.s) > max_s){
        max_s = (temp_vehicle.s-ego.max_s);
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
      if(temp_vehicle.s < ego.s
         && temp_vehicle.s > max_s) {
        max_s = temp_vehicle.s;
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
    }
  }

  if(found_vehicle) { std::cout << "Found vehicle behind" << std::endl; }

  return found_vehicle;
}

float CalculateCost(const Vehicle &vehicle,
                    const map<int, vector<Vehicle>> &predictions,
                    const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory,
                                                       predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &,
                             const map<int, vector<Vehicle>> &,
                             map<string, float> &)
  >> cf_list = {inefficiency_cost, lane_change_cost, out_of_lane_cost, change_lane_safe_cost};
  vector<float> weight_list = {EFFICIENCY, LANE_CHANGE, OUT_OF_LANE, CHANGE_LANE_SAFE};

  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions,
                                               trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const map<int, vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.

  // Note that intended_lane and final_lane are both included to help
  //   differentiate between planning and executing a lane change in the
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;

  return trajectory_data;
}