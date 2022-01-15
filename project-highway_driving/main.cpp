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

/** ********************************************************************
  GLOBAL VARIABLES, fixed and tunable
  ******************************************************************** */      
// STATIC
// - Fixed
const double MPH_MPS = 1609.34/(60.0*60.0);
const double MPS_MP20MS = 20.0/1000.0;
const double MPH_MP20MS = MPH_MPS*MPS_MP20MS;
enum EgoVehicleState{Ready=0, LaneKeep=1, LaneChangeLeft=2, LaneChangeRight=3};


// - Tunables
const int nb_points_planner = 50; // Points for planner (more = reduced ability to react to impromptu events)
const double spline_distance = 30.0; // Meters 

const double limit_speed_margin = 0.5; // Margin for max speed in miles per hour
const double limit_speed_mph = 50.0 - limit_speed_margin; // Max speed in miles per hour
const double limit_speed_mp20ms = limit_speed_mph*MPH_MP20MS; // Max speed in meters per 20ms
const double limit_acc_mps2 = 10.0; // Max acceleration in meters per seconds^2
const double limit_acc_mp20ms2 = limit_acc_mps2*MPS_MP20MS*MPS_MP20MS; // Max acceleration in meters per 20ms^2
const double collision_avoid_safe_distance = 30.0; // Safe distance in meters

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
  /*****************************************************************************
     VARIABLES TO PERSIST BETWEEN CALLS
   ****************************************************************************/
  int target_lane = 1;
  double target_speed_mp20ms = 0;
  EgoVehicleState target_state = Ready;
  /****************************************************************************/

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               // Pass variables to persist between calls
               &target_lane, &target_speed_mp20ms, &target_state] 
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

          // Non-consumed points of the previous path given to the Planner          
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
        
          /*********************************************************************
            OBJECTIVE:
            Determine a path made up of (x,y) points that the car will visit 
            sequentially every .02 seconds (20 milliseconds).
            
            LOGIC:
            - Avoid imminent collision by adapting speed
            - Avoid slowing down by looking far horizon
            - Determine 
              - Front/rear left/right lane change possibility
                - Closest front/rear left/right vehicles and speed
                - Closest vehicles collision if lane change
              - Best lane for change based on closest vehicles presence & speed
              - Ego vehicle state (speed, lane keep or change)
            
            NOTE:
            Logic is executed around every ~.1 seconds (100 milliseconds)
          *********************************************************************/
          // Retrieve previous path points that were not consumed yet
          int previous_path_size = previous_path_x.size();
          
          // Starting point (either car location or previous path end point)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Initial flags
          bool flag_too_close_front = false;
          bool flag_lane_change_required = false;
          
          // TODO: Add cost function to determine whether it is better to go left or right or stay in lane or slow down to pass vehicles
          
          /** ********************************************************************
              COLLISION AVOIDANCE: SPEED CHANGE REQUIRED? & LANE CHANGE REQUIRED?
              ******************************************************************** */
          
          if(previous_path_size > 0) {
              car_s = end_path_s;
          }
          
          for(int i =0; i < sensor_fusion.size(); i++) {
            // Check if car is in ego vehicle lane
            double d = sensor_fusion[i][6];
            // Check if car is in +2 or -2 d from our car (1 lane is 4 large)
            if((2+4*target_lane-2) < d && d < (2+4*target_lane+2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                // Determine the other car S future value to compare to the ego car end of path S value
                check_car_s += ((double)previous_path_size*0.02*check_speed);
                
                // Check if car is in front and if distance is inferior to 30 m
                if((check_car_s > car_s) && ((check_car_s-car_s) < collision_avoid_safe_distance) ) {                                        
                  // TODO: Check if can take over
                  // Assign limited speed
                  flag_too_close_front = true;
                  flag_lane_change_required = true;
                  
                  // TODO: Put lane change after LR collision avoidance and cost function
                  // Change lane
                  if (0 < d && d <= 4) {
                    target_lane = 1;
                  } else if (4 < d && d <= 8) {
                    target_lane = 0;
                  } else if (8 < d && d <= 12) {
                    target_lane = 1;
                  } 
                  // Break loop
                  continue;
                  // TODO: Memorize speed of car in front to adapt ego vehicle speed to it
                }
            }
          }
          
          car_s = j[1]["s"];
          
          /** ********************************************************************
              DETERMINE ACCELERATION
              ******************************************************************** */
          // TODO: Raise/Lower speed in path planner
          if (flag_too_close_front) {
              target_speed_mp20ms -= limit_acc_mp20ms2;
          } else {
              if (target_speed_mp20ms < limit_speed_mp20ms) {
                target_speed_mp20ms += limit_acc_mp20ms2;
                
                if (target_speed_mp20ms > limit_speed_mp20ms) {
                  target_speed_mp20ms = limit_speed_mp20ms;
                }
              }
          }
          
          /** ********************************************************************
              FAR HORIZON: LANE CHANGE REQUIRED?
              ******************************************************************** */
          
          /** ********************************************************************
              DETERMINE PATH
              ******************************************************************** */          
          // Widely spaced waypoints for spline determination
          vector<double> spline_xs;
          vector<double> spline_ys;
          
          // Determine car reference location using either tangent (requiring 2 points) or ego vehicle information
          if (previous_path_size < 2) {
            // Reference is vehicle information
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            spline_xs.push_back(prev_car_x);
            spline_xs.push_back(car_x);
            
            spline_ys.push_back(prev_car_y);
            spline_ys.push_back(car_y);
            
          } else {
            // Reference is previous path end point
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            
            double prev_ref_x = previous_path_x[previous_path_size-2];
            double prev_ref_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
            
            spline_xs.push_back(prev_ref_x);
            spline_xs.push_back(ref_x);
            
            spline_ys.push_back(prev_ref_y);
            spline_ys.push_back(ref_y);            
          }
          
          // Calculate XY position of spline points, separated by spline_distance meters
          vector<double> next_wp0 = getXY(car_s+1.0*spline_distance, (2.0+4.0*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+2.0*spline_distance, (2.0+4.0*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+3.0*spline_distance, (2.0+4.0*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          spline_xs.push_back(next_wp0[0]);
          spline_xs.push_back(next_wp1[0]);
          spline_xs.push_back(next_wp2[0]);
          
          spline_ys.push_back(next_wp0[1]);
          spline_ys.push_back(next_wp1[1]);
          spline_ys.push_back(next_wp2[1]);
          
          // Shift car reference angle to 0 degress
          for (int i = 0; i< spline_xs.size(); i++) {
            double shift_x = spline_xs[i]-ref_x;
            double shift_y = spline_ys[i]-ref_y;
            
            spline_xs[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            spline_ys[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          // Create spline
          tk::spline path_spline;
          path_spline.set_points(spline_xs, spline_ys);
          
          // Create point vector for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          // - Add previous non-consumed points
          for (int i = 0; i< previous_path_size; i++) { // TODO: REVERT IF DOES NOT CHANGE
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // - Generate points up to max size (to compensate for points that were consumed between iterations)
          double target_x = spline_distance;
          double target_y = path_spline(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on = 0.0;
          
          for (int i = 0; i<= nb_points_planner-previous_path_size; i++) {            
            double N = (target_dist/target_speed_mp20ms);
            double x_point = x_add_on + target_x/N;
            double y_point = path_spline(x_point);

            x_add_on = x_point;
            
            // 
            double x_veh_coord = x_point;
            double y_veh_coord = y_point;
            
            // Rotate back to map coordinates
            x_point = (x_veh_coord * cos(ref_yaw) - y_veh_coord * sin(ref_yaw));
            y_point = (x_veh_coord * sin(ref_yaw) + y_veh_coord * cos(ref_yaw));
            
            // Translate from car current position
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
         
          /** ********************************************************************
              TODO: END
              ******************************************************************** */

          // Console path dump
          //for (int i = 0; i< next_x_vals.size(); i++) {            
          //  std::cout << "[" << next_x_vals[i] << "; "<< next_y_vals[i] << "]    ";
          //}
          //std::cout << std::endl;
          
          json msgJson;
          
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
