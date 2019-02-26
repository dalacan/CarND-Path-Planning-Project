#include "planner.h"
#include "spline.h"

#include <iostream>

Planner::Planner(Highway &highway) {
  this->highway = highway;
}

void Planner::SetParameters(int number_lanes, double lane_size, double speed_limit, double max_acceleration, double max_jerk) {
  this->num_lanes = number_lanes;
  this->speed_limit = speed_limit;
  this->lane_size = lane_size;
  this->max_acceleration = max_acceleration;
  this->max_jerk = max_jerk;
}

// Update the ego vehicle localization data
void Planner::SetEgo(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, json &previous_path_x, json &previous_path_y) {
  string state = "CS";
  if(this->ego.state.compare("CS") != 0) {
    // Set new state
    state = this->ego.state;
  }

  // Set previous path info
  auto prev_path_x = previous_path_x;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
//  std::cout << "set prev path size: " << prev_path_x.size() << std::endl;
  this->previous_path_size = prev_path_x.size();

  if(previous_path_size>0) {
    // Use the previous path to set vehicle location
    double ref_x = previous_path_x[this->previous_path_size-1];
    double ref_y = previous_path_y[this->previous_path_size-1];

    double ref_prev_x = previous_path_x[this->previous_path_size-2];
    double ref_prev_y = previous_path_y[this->previous_path_size-2];
    double ref_yaw = atan2(ref_y -ref_prev_y, ref_x - ref_prev_x);

    car_x = ref_x;
    car_y = ref_y;
    car_yaw = ref_yaw;

    vector<double> car_sd = highway.getFrenet(car_x, car_y, car_yaw);

    car_s = car_sd[0];
    car_d = car_sd[1];

    double vel_x = (ref_x - ref_prev_x) / 0.02;
    double vel_y = (ref_y - ref_prev_y) / 0.02;

    car_speed = sqrt((vel_x*vel_x + vel_y*vel_y));
  }

  int lane = highway.GetLaneNumber(car_d);

  Vehicle ego = Vehicle(car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, state);
//  std::cout << "|   x   |   y   |   s   |   d   |  yaw   | speed | lane | state |" << std::endl;
//  std::cout << " " << car_x << " " << car_y << "  " << car_s << " " << car_d << " " << car_yaw << " " << car_speed << "    " << lane << "     " << state << std::endl;

  ego.Configure(highway.num_lanes, this->speed_limit, this->max_acceleration);

  this->ego = ego;
}

// Update the detected vehicles to keep track of vehicles detected from sensor fusion.
void Planner::UpdateDetectedVehicles(json sensor_fusion) {

  auto sensed_vehicles = sensor_fusion;

  vector<Vehicle> detected_vehicles;

  for(int i=0; i< sensed_vehicles.size(); i++) {
    int detected_car_id = sensor_fusion[i][0];
    double detected_car_x = sensor_fusion[i][1];
    double detected_car_y = sensor_fusion[i][2];
    double detected_car_vx = sensor_fusion[i][3];
    double detected_car_vy = sensor_fusion[i][4];
    double detected_car_s = sensor_fusion[i][5];
    double detected_car_d = sensor_fusion[i][6];

    double detected_car_v = sqrt((detected_car_vx*detected_car_vx + detected_car_vy*detected_car_vy));


    /*
     * Using previous points, to need to project car out for a more accurate position of the car now.
     * Note: Assuming car velocity is the same for the time period between the previous points and the
     * estimated projected position.
    */
    if(previous_path_size>0) {
      detected_car_s += ((double)this->previous_path_size*.02*detected_car_v);
    }

    int lane = highway.GetLaneNumber(detected_car_d);
//    std::cout << "Detected car @ " << detected_car_id << ", lane: " << lane << ", velocity: " << detected_car_v << " d: " << detected_car_d <<  std::endl;

    Vehicle vehicle = Vehicle(detected_car_id, detected_car_x, detected_car_y, detected_car_s, detected_car_d, detected_car_v, lane);
    vehicle.state = "CS";
    detected_vehicles.push_back(vehicle);
  }

  this->detected_vehicles = detected_vehicles;
}

// Generate prediction for the ego and detected vehicles
void Planner::GeneratePredictions(int horizon) {
  // Generate predictions for ego vehicle
  int ego_id = -1;
  this->predictions[ego_id] = this->ego.GeneratePredictions(horizon);

  // Generate predictions for detected vehicles
  for(int i=0; i<this->detected_vehicles.size(); i++) {
    int v_id = this->detected_vehicles[i].id;
    this->predictions[v_id] = this->detected_vehicles[i].GeneratePredictions(horizon);
  }

  int temp_v_id;
  Vehicle temp_vehicle;

  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_v_id = it->first;
    temp_vehicle = it->second[0];
//    std::cout << "1: Vehicle id, lane, s, v: " << temp_v_id << ", " << temp_vehicle.lane << ", " << temp_vehicle.s << ", " << temp_vehicle.v << std::endl;
  }
}

// Plan the next behavior for the ego vehicles for the current state and predictions
void Planner::PlanBehavior() {
  vector<Vehicle> next_state = this->ego.ChooseNextState(this->predictions);

//  std::cout << "current state: " << ego.state << std::endl;
//  std::cout << "current  state lane: " << ego.lane << std::endl;
//  std::cout << "current  state s: " << ego.s << std::endl;
//  std::cout << "current  state v: " << ego.v << std::endl;
//  std::cout << "current  state a: " << ego.a << std::endl;

  Vehicle nxt_state = next_state[1];
//  std::cout << "next state: " << nxt_state.state << std::endl;
//  std::cout << "next state lane: " << nxt_state.lane << std::endl;
//  std::cout << "next state s: " << nxt_state.s << std::endl;
//  std::cout << "next state v: " << nxt_state.v << std::endl;
//  std::cout << "next state a: " << nxt_state.a << std::endl;

  // Set next state
  this->ego_next_state = this->ego;
  ego_next_state.RealizeNextState(next_state);
}

// Generate trajectory for the specified planned state
vector<vector<double>> Planner::GenerateTrajectory() {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Generate using spline
  vector<double> pts_x;
  vector<double> pts_y;

  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = this->highway.deg2rad(ego.yaw);

  double ref_prev_x;
  double ref_prev_y;

  // Set the starting trajectory base on either a previous point estimate or previous point
  if(this->previous_path_size < 2) {
    // If there is not previous path,estimate a previous point base on the reference points and yaw
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(ego.x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(ego.y);

    ref_vel = 0;

  } else {
    // Use the previous path
    ref_x = previous_path_x[this->previous_path_size-1];
    ref_y = previous_path_y[this->previous_path_size-1];

    ref_prev_x = previous_path_x[this->previous_path_size-2];
    ref_prev_y = previous_path_y[this->previous_path_size-2];
    ref_yaw = atan2(ref_y -ref_prev_y, ref_x - ref_prev_x);

    pts_x.push_back(ref_prev_x);
    pts_x.push_back(ref_x);

    pts_y.push_back(ref_prev_y);
    pts_y.push_back(ref_y);
  }

  /*
   * Add X number of waypoints 30m apart in FreNet relative to the target velocity.
   * The greater the velocity, the more waypoints are required.
   */
  double waypoint_interval = 30;
  int required_waypoints = 3 + (int)this->ego_next_state.v*2.24/50;
  for(int i=0; i<required_waypoints; i++) {
    vector<double> next_waypt = highway.getXY(ego.s+waypoint_interval*(i+1), (2+4*ego_next_state.lane));


    pts_x.push_back(next_waypt[0]);
    pts_y.push_back(next_waypt[1]);
  }

  // Add previous points
  for (int i = 0; i <+ this->previous_path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Use this for constant velocity (cruising)
  for (int i = 0; i < pts_x.size(); i++) {
    // Shift car reference angle to 0
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;

    pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    pts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }

  // Create a spline
  tk::spline spl;

  // Set x, y points to splint
  spl.set_points(pts_x, pts_y);

  // Using 30 metres as an arbitrary number
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  /*
   * Calculate the required number of n intervals over the target distance for the
   * specified target reference velocity over 0.02 seconds (time at which the simulator
   * process each point).
  */
  double x_add_on = 0;
//  std::cout << "Initial ref velocity: " << ref_vel << std::endl;
//  std::cout << "Target velocity: " << this->ego_next_state.v*2.24 << std::endl;
  for (int i = 0; i < 50 - this->previous_path_size; i++) {
    // Calculate the next point relative to the car
    if(ref_vel <= this->ego_next_state.v*2.24) {
      ref_vel += 0.224;
//        ref_vel += 0.4032; // 0.448 = 10m/s2, 0.4256 = 9.5m/s2
    } else if(ref_vel > this->ego_next_state.v*2.24) {
      ref_vel -= 0.224;
//        ref_vel -= 0.4032; // 0.448 = 10m/s2, 0.4256 = 9.5m/s2
    }
//    std::cout << "Next Ref velocity: " << ref_vel << std::endl;

    double n = target_dist / (0.02 * ref_vel / 2.24);

    double next_x = x_add_on + (target_x / n);
    double next_y = spl(next_x);

    x_add_on = next_x;

    double car_ref_x = next_x;
    double car_ref_y = next_y;

    // Convert back to cartesian coordinates
    next_x = car_ref_x * cos(ref_yaw) - car_ref_y * sin(ref_yaw);
    next_y = car_ref_x * sin(ref_yaw) + car_ref_y * cos(ref_yaw);

    // Add the next waypoint projection to the car's actual current reference
    next_x += ref_x;
    next_y += ref_y;

    // Set ego next state
    ego.state = ego_next_state.state;

    next_x_vals.push_back(next_x);
    next_y_vals.push_back(next_y);
  }
  return {next_x_vals, next_y_vals};

}