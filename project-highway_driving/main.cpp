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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

         
          /** ********************************************************************
              TODO: define a path made up of (x,y) points that the car will visit
              sequentially every .02 seconds
              ******************************************************************** */
          // Parameters
          int nb_points_planner = 50;
          int lane = 1;
          double spline_distance = 30.0; // Meters
          
          // Speed and acceleration targets
          double limit_speed_margin = 0.5; // MPH
          double limit_speed_mph = 50.0 - limit_speed_margin; // MPH
          double limit_speed_mps = limit_speed_mph/(0.00062137*60.0*60.0); // MPH
          double limit_acc = 10.0; // 10 M/S2
          double limit_dist_inc = limit_acc / (50.0); // /1000 M/MS2 >> *20 M/20MS2

          // Dynbamic variables initialization
          int previous_path_size = previous_path_x.size();
          
          // Widely spaced waypoints for spline determination
          vector<double> pts_x;
          vector<double> pts_y;
          
          // Starting point (either car location or previous path end point)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // Determine car reference location using either tangent (requiring 2 points) or ego vehicle information
          if (previous_path_size < 2) {
            std::cout << "No path!" << std::endl;
            // Reference is vehicle information
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
            
          } else {
            std::cout << "Path!" << std::endl;
            // Reference is previous path end point
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            
            double prev_ref_x = previous_path_x[previous_path_size-2];
            double prev_ref_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
            
            pts_x.push_back(prev_ref_x);
            pts_x.push_back(ref_x);
            
            pts_y.push_back(prev_ref_y);
            pts_y.push_back(ref_y);            
          }
          
          // Calculate XY position of spline points, separated by spline_distance meters
          vector<double> next_wp0 = getXY(car_s+1.0*spline_distance, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+2.0*spline_distance, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+3.0*spline_distance, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          
          // Shift car reference angle to 0 degress
          for (int i = 0; i< pts_x.size(); i++) {
            double shift_x = pts_x[i]-ref_x;
            double shift_y = pts_y[i]-ref_y;
            
            pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          // Create spline
          tk::spline s;
          s.set_points(pts_x, pts_y);
          
          // Create point vector for planner
          // - Add previous non-consumed points
          for (int i = 0; i< previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // - Generate points up to max size (to compensate for points that were consumed between iterations)
          double target_x = spline_distance;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on = 0;
          
          for (int i = 0; i< nb_points_planner-previous_path_size; i++) {            
            double N = (target_dist/(0.02*limit_speed_mph/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to map coordinates
            x_point = (x_ref * cos(0-ref_yaw) - y_ref * sin(0-ref_yaw));
            y_point = (x_ref * sin(0-ref_yaw) + y_ref * cos(0-ref_yaw));
            
            x_point += x_ref;
            y_point += y_ref;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
         
          /** ********************************************************************
              TODO: END
              ******************************************************************** */

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