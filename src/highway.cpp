#include "highway.h"
#include <fstream>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

using std::string;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Highway::Highway() {}

// Load the highway waypoints from the specified map file
void Highway::LoadWaypoints(string map_file) {
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

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
    this->map_waypoints_x.push_back(x);
    this->map_waypoints_y.push_back(y);
    this->map_waypoints_s.push_back(s);
    this->map_waypoints_dx.push_back(d_x);
    this->map_waypoints_dy.push_back(d_y);
  }
}

// Set the lane configuration for the highway
void Highway::SetLane(double lane_width, int num_lanes) {
  this->lane_width = lane_width;
  this->num_lanes = num_lanes;
}

int Highway::GetLaneNumber(double d) {
  double half_lane_width = this->lane_width/2.0;
  for(int lane_number=0; lane_number<this->num_lanes; lane_number++) {
    if(d < (half_lane_width+this->lane_width*lane_number+half_lane_width) && d > (half_lane_width+this->lane_width*lane_number-half_lane_width)) {
      return lane_number;
    }
  }
}

int Highway::GetLaneNumber(double x, double y, double theta) {
  vector<double> s_d = getFrenet(x, y, theta);

  return GetLaneNumber(s_d[1]);
}

// For converting back and forth between radians and degrees.
double Highway::deg2rad(double x) { return x * pi / 180; }
double Highway::rad2deg(double x) { return x * 180 / pi; }

// Calculate distance between two points
double Highway::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int Highway::ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int Highway::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi - angle, angle);

  if (angle > pi/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Highway::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x,y, theta, map_waypoints_x,map_waypoints_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = map_waypoints_x.size()-1;
  }

  double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-map_waypoints_x[prev_wp];
  double center_y = 2000-map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Highway::getXY(double s, double d) {
  int prev_wp = -1;

  while (s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%map_waypoints_x.size();

  double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),
                         (map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> Highway::JMT(vector<double> &start, vector<double> &end, double T) {
  MatrixXd t(3, 3);
  t << pow(T, 3), pow(T, 4), pow(T, 5),
          3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
          6*T, 12*pow(T, 2), 20*pow(T, 3);

  MatrixXd s(3, 1);
  s << end[0] - (start[0] + start[1]*T + 0.5*start[2]*pow(T, 2)),
          end[1] - (start[1] + start[2]*T),
          end[2] - start[2];

  MatrixXd c(3, 1);
  c = t.inverse() * s;

  return {start[0],start[1],0.5*start[2],c(0), c(1), c(2)};
}
