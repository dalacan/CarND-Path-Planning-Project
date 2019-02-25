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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // starting lane
  int lane_number = 1;

  // Reference velocity
  double ref_vel = 0; // 49.5

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
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
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();

          if(prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          double target_velocity = 49.5;

          vector<double> lane_and_cars(3, 60.0);
          vector<double> lane_velocity(3, 49.5);

          // find ref_v to use
          for(int i=0; i< sensor_fusion.size(); i++) {
            // Look for cars in same lane
            double detected_car_d = sensor_fusion[i][6];
            double detected_car_s = sensor_fusion[i][5];

            // Set to 2.5 to consider other cars that cross over to the same lane (i.e. changing lane)
            if(detected_car_d < (2+4*lane_number+2) && detected_car_d > (2+4*lane_number-2)) {
              // Get the car's velocity
              double detected_car_vx = sensor_fusion[i][3];
              double detected_car_vy = sensor_fusion[i][4];


              double detected_car_v = sqrt((detected_car_vx*detected_car_vx + detected_car_vy*detected_car_vy));

              /*
               * USing previous points, to need to project car out for a more accurate position of the car.
               * Note: Assuming car velocity is the same for the time period between the previous points and the
               * estimated projected position.
              */
              detected_car_s += ((double)prev_size*.02*detected_car_v);

              // Check the car gap
              if(detected_car_s > car_s && (detected_car_s - car_s ) < 30) {
                too_close = true;

                // Set velocity to match detected car speed
                std::cout << "Found car " << sensor_fusion[i][0] <<  " in front @ velocity: " << detected_car_v << std::endl;
                target_velocity = detected_car_v*2.24;

                // TO DO: Add logic to decide whether to change lane logic or lower velocity

              }
            }

            // track for car within range of 30m (will be used for safe lane changing)
            if(detected_car_s > (car_s-20) && (detected_car_s - car_s ) < 60.0) {
              for(int j=0; j< lane_and_cars.size(); j++) {
                if(detected_car_d < (2+4*j+2) && detected_car_d > (2+4*j-2)) {
                  std::cout << "Detect car " << sensor_fusion[i][0] <<  " @ " << (detected_car_s - car_s) << "m in front @ lane "  << j <<  std::endl;
                  if((detected_car_s - car_s) < lane_and_cars[j]) {
                    lane_and_cars[j] = (detected_car_s - car_s);
                  }

                  // Calculate lane velocity
                  double detected_car_vx = sensor_fusion[i][3];
                  double detected_car_vy = sensor_fusion[i][4];


                  double detected_car_v = sqrt((detected_car_vx*detected_car_vx + detected_car_vy*detected_car_vy));
                  if((detected_car_v*2.24) < lane_velocity[j]) {
                    lane_velocity[j] = detected_car_v*2.24;
                  }
                }
              }
            }

          }

          int choosen_lane = lane_number;
          double empty_lane_distance = 5;


          if(too_close == true) {
            for(int s=0; s<lane_velocity.size(); s++) {
              std::cout << "lane " << s << " velocity: " << lane_velocity[s] << std::endl;
              std::cout << "lane " << s << " empty lane d: " << lane_and_cars[s] << std::endl;
            }

            // Find empty lanes within 10m
            for(int i=0; i<lane_and_cars.size(); i++) {
              if(lane_and_cars[i] > 5 && lane_velocity[i] > target_velocity && lane_and_cars[i] > empty_lane_distance) {
                if(abs(i-lane_number) <= 1) {
                  choosen_lane = i;
                  empty_lane_distance = lane_and_cars[i];
//                  target_velocity = lane_velocity[i];
                } else if(abs(i-lane_number) == 2 && lane_and_cars[1] > 5) {
                  // Switch to middle lane if further lane is faster and middle lane is clear (to eventually switch to further faster lane)
                  choosen_lane =1;
                  empty_lane_distance = lane_and_cars[1];
                  std::cout << "performing double lane change" << std::endl;
                }
              }
            }
            std::cout << "changing lane from " << lane_number << " to " << choosen_lane << " with eld: " << empty_lane_distance << std::endl;
            lane_number = choosen_lane;
          }

          // Output debugger
//          std::cout << "car x: " << car_x
//                    << " y: " << car_y
//                    << " s: " << car_s
//                    << " d: " << car_d << std::endl;
//
//          int closestwaypoint_idx = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
//          int nextwaypoint_idx = NextWaypoint(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
//          std::cout << "Closest waypoint: " << map_waypoints_x[closestwaypoint_idx] << ", " << map_waypoints_y[closestwaypoint_idx] << std::endl;
//          std::cout << "Next waypoint: " << map_waypoints_x[nextwaypoint_idx] << ", " << map_waypoints_y[nextwaypoint_idx] << std::endl;
          // End output debugger

          // Next
//          nextwaypoint_idx = (nextwaypoint_idx + 1)%map_waypoints_x.size();
//          vector<double> next_sd;
//          next_sd = getFrenet(map_waypoints_x[nextwaypoint_idx], map_waypoints_y[nextwaypoint_idx] , deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
//          next_sd[1] = 6; // Set to middle lane

//          vector<double> next_xy = getXY(next_sd[0], 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          double ref_prev_x;
          double ref_prev_y;

          double ref_v;

          // Set the starting trajectory base on either a previous point estimate or previous point
          if(prev_size < 2) {
            // If there is not previous path,estimate a previous point base on the reference points and yaw
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);

            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);

            ref_vel = 0;
          } else {
            // Use the previous path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            ref_prev_x = previous_path_x[prev_size-2];
            ref_prev_y = previous_path_y[prev_size-2];
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
          double waypoint_interval = 40;
          int required_waypoints = (int)target_velocity/15;
          for(int i=0; i<required_waypoints; i++) {
            vector<double> next_waypt = getXY(car_s+waypoint_interval*(i+1), (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);


            pts_x.push_back(next_waypt[0]);
            pts_y.push_back(next_waypt[1]);
          }
          // Add 3 waypoints 30m apart in FreNet
//            vector<double> next_waypt0 = getXY(car_s+30, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            vector<double> next_waypt1 = getXY(car_s+60, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            vector<double> next_waypt2 = getXY(car_s+90, (2+4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);

//          pts_x.push_back(next_waypt0[0]);
//          pts_x.push_back(next_waypt1[0]);
//          pts_x.push_back(next_waypt2[0]);
//
//          pts_y.push_back(next_waypt0[1]);
//          pts_y.push_back(next_waypt1[1]);
//          pts_y.push_back(next_waypt2[1]);

          // Testing
          // Convert to s and d
          vector<vector<double>> pts_sd;
          for(int i=0; i<pts_x.size(); i++) {
            pts_sd.push_back(getFrenet(pts_x[i], pts_y[i], ref_yaw, map_waypoints_x, map_waypoints_y));
          }

          // Add previous points
          for (int i = 0; i <+ prev_size; i++) {
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
          double target_x = 40.0;
          double target_y = spl(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          /*
           * Calculate the required number of n intervals over the target distance for the
           * specified target reference velocity over 0.02 seconds (time at which the simulator
           * process each point).
           * Note: The velocity in (mph) is divide by 2.24 to convert to metre per second
          */
          double x_add_on = 0;
//          std::cout << "Initial ref velocity: " << ref_vel << std::endl;
//          std::cout << "Target velocity: " << target_velocity << std::endl;
          for (int i = 0; i < 50 - prev_size; i++) {
            // Calculate the next point relative to the car
            if(ref_vel <= target_velocity) {
              ref_vel += 0.224;
//              ref_vel += 0.4032; // 0.448 = 10m/s2, 0.4256 = 9.5m/s2
            } else if(ref_vel > target_velocity) {
              ref_vel -= 0.224;
//              ref_vel -= 0.4032; // 0.448 = 10m/s2, 0.4256 = 9.5m/s2
            }
//            std::cout << "Ref velocity: " << ref_vel << std::endl;

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

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          // Debug output
//          for (int i = 0; i < next_x_vals.size(); i++) {
//            if(i < prev_size) {
//              std::cout<< "next x,y: " << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
//            } else {
//              std::cout<< "|next x,y: " << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
//            }
//          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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