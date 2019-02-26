#ifndef HIGHWAY_H
#define HIGHWAY_H

#include <vector>
#include <string>
#include <math.h>

using std::string;
using std::vector;

class Highway {
public:
  Highway();
  void LoadWaypoints(string map_file);

  void SetLane(double lane_width, int num_lanes);

  int GetLaneNumber(double d);
  int GetLaneNumber(double x, double y, double theta);

  // For converting back and forth between radians and degrees.

  double deg2rad(double x);
  double rad2deg(double x);

// Calculate distance between two points
  double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                      const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                   const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta);

// Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d);

  vector<double> JMT(vector<double> &start, vector<double> &end, double T);


  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  double lane_width = 4.0f;
  int num_lanes = 3;
  constexpr static double pi = M_PI;
};

#endif // HIGHWAY_H
