#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float CalculateCost(const Vehicle &vehicle,
                    const map<int, vector<Vehicle>> &predictions,
                    const vector<Vehicle> &trajectory);


float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data);

float lane_change_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data);

float out_of_lane_cost(const Vehicle &vehicle,
                       const vector<Vehicle> &trajectory,
                       const map<int, vector<Vehicle>> &predictions,
                       map<string, float> &data);

float change_lane_safe_cost(const Vehicle &vehicle,
                            const vector<Vehicle> &trajectory,
                            const map<int, vector<Vehicle>> &predictions,
                            map<string, float> &data);

float lane_cost(const Vehicle &vehicle,
                const vector<Vehicle> &trajectory,
                const map<int, vector<Vehicle>> &predictions,
                map<string, float> &data);

bool get_vehicle_ahead(const map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle);
bool get_vehicle_behind(const map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle);

map<string, float> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const map<int, vector<Vehicle>> &predictions);

#endif  // COST_H