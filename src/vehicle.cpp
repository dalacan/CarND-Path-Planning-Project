#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"

#include <iostream>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

// Initialize an instance of the vehicle class for the ego vehicle
Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double v, int lane, string state) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->v = v;
    this->yaw = yaw;
    this->lane = lane;

    this->state = state;
//    max_acceleration = -1;

}

// Initialize an instance of the vehicle class for detected vehicles
Vehicle::Vehicle(int id, double x, double y, double s, double d, double v, int lane, string state) {
    this->id = id;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;

    // Assume vehicle at constant speed
    this->a = 0.0f;

    this->v = v;
    this->lane = lane;

    this->state = state;
}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::GeneratePredictions(int horizon) {
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    vector<Vehicle> predictions;
    for(int i = 0; i < horizon; ++i) {
        double next_s = PositionAt(i);
        double next_v = VelocityAt(i);
        predictions.push_back(Vehicle(this->lane, next_s, next_v, this->a, this->state));
    }

    return predictions;
}

double Vehicle::PositionAt(double t) {
    // Calculate position
    return this->s + this->v*t + this->a*t*t/2.0;
}

double Vehicle::VelocityAt(double t) {
    double v = this->v + this->a * t;

    if (v < 0.0) {
        return 0.0;
    }

    return v;
}

vector<Vehicle> Vehicle::ChooseNextState(map<int, vector<Vehicle>> &predictions) {
    // Get successor states
    vector<string> success_states = SuccessorStates();

    vector<Vehicle> trajectory_for_state;
    vector<float> cost_for_state;
    vector<vector<Vehicle>> final_trajectories;

    // Generate trajectory for each succorstates
    for(int i=0; i<success_states.size(); i++) {
        std::cout << "Calculating trajectory for: " << success_states[i] << std::endl;
        trajectory_for_state = GenerateTrajectory(success_states[i], predictions);
        if (trajectory_for_state.size() != 0) {
//            std::cout << "Trajectory: " << trajectory_for_state.size() << std::endl;
            // Calculate cost for each successor state
            cost_for_state.push_back(CalculateCost(*this, predictions, trajectory_for_state));
            final_trajectories.push_back(trajectory_for_state);
        }
    }

    float min_cost = 9999999;
    float cost;
    int best_state_idx = 0;
    // Choose lowest cost state
    for(int i=0; i<cost_for_state.size(); i++) {
        cost = cost_for_state[i];
        std::cout << "State: " << success_states[i] << " Cost: " << cost << std::endl;
        if(cost < min_cost) {
            min_cost = cost;
            best_state_idx = i;
        }
    }


    std::cout << "Choosen state: " << success_states[best_state_idx] << std::endl;
    return  final_trajectories[best_state_idx];
}

// Provides the possible next states given the current state for the FSM
vector<string> Vehicle::SuccessorStates() {
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) { // KL - PLCL, PLCR
      if (lane != 0) {
        states.push_back("PLCL");
      }

      if (lane != this->num_lanes - 1) {
        states.push_back("PLCR");
      }
    } else if (state.compare("PLCL") == 0) { // PLCL - KL, PLCL, LCL
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) { // PLCR - KL, PLCR, LCR
        if (lane != this->num_lanes - 1) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    } else if (state.compare("LCL") == 0) { // LCL - KL & LCL
        states.push_back("LCL");
    } else if (state.compare("LCR") == 0) { // LCR - KL & LCR
        states.push_back("LCR");
    }

    return states;
}

vector<Vehicle> Vehicle::GenerateTrajectory(string state, map<int, vector<Vehicle>> &predictions) {
// Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = ConstantSpeedTrajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = KeepLaneTrajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = LaneChangeTrajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = PrepLaneChangeTrajectory(state, predictions);
    }

    return trajectory;
}

vector<Vehicle> Vehicle::ConstantSpeedTrajectory() {
// Generate a constant speed trajectory.
    double next_pos = PositionAt(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state),
                                  Vehicle(this->lane,next_pos,this->v,0,this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::KeepLaneTrajectory(map<int, vector<Vehicle>> &predictions) {
    // Generate a keep lane trajectory.
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<double> kinematics = GetKinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];

    std::cout << "Keep Lane Traj (s, v, a): " << new_s << ", " << new_v << ", " << new_a << std::endl;
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));

    return trajectory;
}

vector<Vehicle> Vehicle::LaneChangeTrajectory(string state, map<int, vector<Vehicle>> &predictions) {
    double min_gap_size = 15.0;
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s < this->s+min_gap_size/2 && next_lane_vehicle.s > this->s-min_gap_size/2 && next_lane_vehicle.lane == new_lane) {
            // If lane change is not possible, return empty trajectory.
            std::cout << "Lane change not possible" << std::endl;
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a,
                                 this->state));
    vector<double> kinematics = GetKinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], state));
    return trajectory;
}
vector<Vehicle> Vehicle::PrepLaneChangeTrajectory(string state, map<int, vector<Vehicle>> &predictions) {
// Generate a trajectory preparing for a lane change.
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a,
                                          this->state)};
    vector<double> curr_lane_new_kinematics = GetKinematics(predictions, this->lane);

    if (GetVehicleBehind(predictions, this->lane, vehicle_behind)) {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = GetKinematics(predictions, new_lane);
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));

    return trajectory;
}

vector<double> Vehicle::GetKinematics(map<int, vector<Vehicle>> &predictions, int lane) {
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (GetVehicleAhead(predictions, lane, vehicle_ahead)) {
//        if (GetVehicleBehind(predictions, lane, vehicle_behind)) {
//            // must travel at the speed of traffic, regardless of preferred buffer
//            new_velocity = vehicle_ahead.v;
//        } else {
            std::cout << "Front vehicle velocity: " << vehicle_ahead.v << std::endl;
        double max_velocity_in_front;
            if(vehicle_ahead.s < 300 && this->s > max_s -300) {
                // Handle wrap around
                max_velocity_in_front = (vehicle_ahead.s + max_s - this->s
                                               - this->preferred_buffer) + vehicle_ahead.v
                                              - 0.5 * (this->a);

            } else {
                max_velocity_in_front = (vehicle_ahead.s - this->s
                                               - this->preferred_buffer) + vehicle_ahead.v
                                              - 0.5 * (this->a);

            }
            std::cout << "Max velocity in front: " << max_velocity_in_front << std::endl;
            std::cout << "vehicle_ahead.s: " << vehicle_ahead.s << " this->s: " << this->s << " vehicle_ahead.v: " << vehicle_ahead.v << " this->a: " << this->a << std::endl;
            new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
            std::cout << "Proposed new new_velocity: " << new_velocity << std::endl;
//        }
    } else {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;

    return{new_position, new_velocity, new_accel};
}

string Vehicle::FindGap(map<int, vector<Vehicle>> &predictions, int lane, double &gap_s) {
  bool gap_found = false;
  double min_gap_size = 15.0;
  string gap_location;

  // Attempt to find a next to vehicle
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  bool vehicle_ahead_bool = GetVehicleAhead(predictions, lane, vehicle_ahead);
  bool vehicle_behind_bool = GetVehicleBehind(predictions, lane, vehicle_behind);

  if(!vehicle_ahead_bool && !vehicle_behind_bool) {
    // No vehicle in intended lane
    gap_found = true;
    gap_location = gap_locations[1];
      gap_s = this->s;
  }
  else if(vehicle_ahead_bool && !vehicle_behind_bool && (vehicle_ahead.s - this->s) > min_gap_size/2.0) {
    // Vehicle ahead but no vehicle behind, check if distance to closest vehicle in front has sufficient gap relative to ego
    gap_found = true;
    gap_location = gap_locations[1];
    gap_s = this->s;
  } else if(vehicle_behind_bool && !vehicle_ahead_bool && (this->s - vehicle_behind.s) > min_gap_size/2.0) {
    // Vehicle behind but no vehicle ahead, check if distance to closest vehicle behind has sufficient gap to ego relative to ego
    gap_found = true;
    gap_location = gap_locations[1];
    gap_s = this->s;
  } else if(vehicle_ahead_bool && vehicle_behind_bool && (vehicle_ahead.s - vehicle_behind.s)> min_gap_size) {
    // Vehicle ahead and behind, check if gap between both vehicle have a sufficient gap
    gap_found = true;
    gap_location = gap_locations[1];
    gap_s = this->s;
  }

  // Attempt to find a gap behind
  if(!gap_found) {
      Vehicle vehicle_behind2; // Track the second vehicle behind

      while(!gap_found) {
          if(GetVehicleBehind(predictions, lane, vehicle_behind2, vehicle_behind)) {
              if(vehicle_behind.s - vehicle_behind2.s > min_gap_size) {
                  gap_found = true;
                  gap_location = gap_locations[2];
                  gap_s = (vehicle_behind.s - vehicle_behind2.s) / 2;
              }

              vehicle_behind = vehicle_behind2;
          } else {
              gap_found = true;
              gap_location = gap_locations[2];
              gap_s = vehicle_behind.s + min_gap_size / 2.0;
          }
      }
  }

  return gap_location;
}

bool Vehicle::GetVehicleBehind(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.

    double max_s = -this->max_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane) {
            // Handle wrap around
            if(temp_vehicle.s > max_s - 300 && this->s < 300 && (temp_vehicle.s-this->s) > max_s){
                max_s = (temp_vehicle.s-this->max_s);
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
            if(temp_vehicle.s < this->s
            && temp_vehicle.s > max_s) {
                max_s = temp_vehicle.s;
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
        }
    }

//    if(found_vehicle) { std::cout << "Found vehicle behind" << std::endl; }

    return found_vehicle;
}

bool Vehicle::GetVehicleBehind(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle, Vehicle &ref_vehicle) {
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.

    double max_s = -this->max_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == ref_vehicle.lane) {
            // Handle wrap around
            if(temp_vehicle.s > max_s - 300 && ref_vehicle.s < 300 && (temp_vehicle.s-ref_vehicle.s) > max_s){
                max_s = (temp_vehicle.s-ref_vehicle.max_s);
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
            if(temp_vehicle.s < ref_vehicle.s
               && temp_vehicle.s > max_s) {
                max_s = temp_vehicle.s;
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
        }
    }

//    if(found_vehicle) { std::cout << "Found vehicle behind" << std::endl; }

    return found_vehicle;
}

bool Vehicle::GetVehicleAhead(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    double min_s = max_s*2;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane) {
//            std::cout << "ego s: " << this->s << std::endl;
//            std::cout << "vehicle id: " << it->first << " s: " << temp_vehicle.s << std::endl;

            // Handle wrap around
            if(this->s > max_s - 300 && temp_vehicle.s < 300
            && (temp_vehicle.s+max_s) < min_s) {
                min_s = temp_vehicle.s+max_s; // for wraparound
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
            else if(temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
                min_s = temp_vehicle.s;
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
        }
    }

//    if(found_vehicle) { std::cout << "Found vehicle ahead" << std::endl; }

    return found_vehicle;
}

void Vehicle::RealizeNextState(vector<Vehicle> &trajectory) {
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::Configure(double num_lanes, double target_speed, double max_acceleration) {
    // Called by simulator before simulation begins. Sets various parameters which
    //   will impact the ego vehicle.
    this->num_lanes = num_lanes;
    this->target_speed = target_speed;
    this->max_acceleration = max_acceleration;
}