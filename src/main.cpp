#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "planner.h"
#include "highway.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int num_lanes = 3;
double speed_limit = 49.5;
double lane_size = 4.0;

// Limits
double max_acceleration = 10.0f; // m/s^2
double max_jerk = 10.0f; // m/s^3

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // Load highway
  Highway highway;
  highway.LoadWaypoints("../data/highway_map.csv");

  // Initialize highway driving class
  Planner planner = Planner(highway);

  // starting lane
  int lane_number = 1;

  // Reference velocity
  double ref_vel = 0; // 49.5

  h.onMessage([&highway,
               &planner,
               &lane_number,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          json previous_path_x = j[1]["previous_path_x"];
          json previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          json sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Set ego vehicle
          string prev_state = "CS";
          Vehicle previous_ego;

          planner.SetEgo(car_x, car_y, car_s, car_d, car_yaw, car_speed/2.24, previous_path_x, previous_path_y);

          // Update sensor fusion
          planner.UpdateDetectedVehicles(sensor_fusion);

          // Generate predictions
          planner.GeneratePredictions();

          // Choose next state behavior
          planner.PlanBehavior();

          // Generate optimal trajectory for selected state & prepare path plan
          vector<vector<double>> path_xy = planner.GenerateTrajectory();

          msgJson["next_x"] = path_xy[0];
          msgJson["next_y"] = path_xy[1];


          // Debug output
//          for (int i = 0; i < next_x_vals.size(); i++) {
//            if(i < prev_size) {
//              std::cout<< "next x,y: " << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
//            } else {
//              std::cout<< "|next x,y: " << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
//            }
//          }

//          msgJson["next_x"] = next_x_vals;
//          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}