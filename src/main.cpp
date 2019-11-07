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

  //start in lane 1
  int ego_lane = 1;
  
  //Have a reference velocity to start, mph
  double ref_vel = 0.0;
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_lane]
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
          
          //set one para to get the previous path size
          int prev_size = previous_path_x.size();
          
          //set the ego car's as the previous path end s
          if (prev_size > 0){
            car_s = end_path_s;
          }
          
          //flag to detect whether some other cars in surrounding area
          bool hascar_ahead = false;
          bool hascar_left = false;
          bool hascar_right = false;
          
          //check all surrounding cars' location
          for (int i = 0; i < sensor_fusion.size(); i++) {
            //get the surrounding car's d value
            float d = sensor_fusion[i][6];
            int Sur_car_lane = -1;
            
            //check which lane it belonging to
            if (d > 0 && d < 4) {
              Sur_car_lane = 0;
            }
            else if (d >= 4 && d < 8) {
              Sur_car_lane = 1;
            }
            else if (d >= 8 && d < 12) {
              Sur_car_lane = 2;
            }
            
            if (Sur_car_lane < 0) {
              continue;
            }
            
            //get surrounding car's speed and s
            double Sur_car_vx = sensor_fusion[i][3];
            double Sur_car_vy = sensor_fusion[i][4];
            double Sur_car_v = sqrt(Sur_car_vx*Sur_car_vx + Sur_car_vy*Sur_car_vy);
            double Sur_car_s = sensor_fusion[i][5];
            
            //predict surrounding cars new s,
            Sur_car_s  += ((double)prev_size*0.02*Sur_car_v);
            //TODO: to be precisely, need to predict surrounding car's d value also
            
            //to be safe, need to check whether a car within a distance before making change, assume 4 cars' length as 4*5m
            double safe_dis = 20;
            //TODO: need to make an ajustable safe_dis function, so that it can turn quickly under traffic jam situations, need or not?
            //check the relation of surrounding car, whether some car within the safe range
            //surrounding car is ahead of ego car
            if (Sur_car_lane == ego_lane){
              hascar_ahead = hascar_ahead || ((Sur_car_s > car_s) && (Sur_car_s - car_s < safe_dis)); //if any surround car is ahead, hascar_ahead will be 1, the || operation guarantee that if any car in the loop is ahead, the true value will be carried on.
              //TODO: need to check whether some car behind is accelarating to crash into ego
            }
            //surrounding car is left of ego car
            else if (Sur_car_lane - ego_lane == -1) {
              hascar_left = hascar_left || ((car_s - safe_dis < Sur_car_s) && (car_s + safe_dis > Sur_car_s)); //check if sur_car_s is in car_s safe_dis range
            }
            //surrounding car is right of ego car
            else if (Sur_car_lane - ego_lane == 1) {
              hascar_right = hascar_right || ((car_s - safe_dis < Sur_car_s) && (car_s + safe_dis > Sur_car_s));
            }
          }
          
          //behavior planning
          //define max_speed, below but close to 50mph
          const double max_speed = 48.5;
          //define max_acceleration 10 m/s**2, and change to miles per 20 ms, it is approx 0.45, to avoid collision happened during test run, lower this value to 0.30
          const double max_acc = 0.30;
          //TODO: need to define a accelaration function, so that the car can accelarate or decelarate at a continuous comfortable line.
          if (hascar_ahead) {
            //if no car on left and left lane exists, turn left
            if (!hascar_left && ego_lane > 0) {
              ego_lane --;
            }
            //turn right
            else if (!hascar_right && ego_lane < 2) {
              ego_lane++;
            }
            else {
              ref_vel -= max_acc;
            }
          }
          else {
            if (ref_vel < max_speed) {
              ref_vel += max_acc;
              if (ref_vel > max_speed) {
                ref_vel = max_speed;
              }
            }
          }
          
//            if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
//              double vx = sensor_fusion[i][3];
//              double vy = sensor_fusion[i][4];
//              double check_speed = sqrt(vx*vx + vy*vy);
//              double check_car_s = sensor_fusion[i][5];
//
//              check_car_s += ((double)prev_size*0.02*check_speed); //if using previous points can predict s value out
//              //check s values greater tahn mine and s gap
//              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
//                //do some logic here, lower reference velocity so we don't crash into the car in front of us, could also flag to try to change lanes
//                //ref_vel = 29.5; //mph
//                too_close = true;
//                if (lane > 0) {
//                  lane = 0;
//                }
//              }
//
//            }
//          }
//
//          if (too_close) {
//            ref_vel -= 0.224;
//          }
//          else if (ref_vel < 49.5) {
//            ref_vel += 0.224;
//          }
          
          //create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          //later we will interoplate these waypoints with a spline and fill it in with more points that control the spline
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          //reference x,y, yaw states
          //either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous path size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            //use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //use the previous path's end point as starting reference
          else {
            //redefine reference state as prevous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            //use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
          }
          
          //in Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++) {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          //create a spline
          tk::spline s;
          
          //set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          
          double x_add_on = 0;
          
          //fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            
            double N = (target_dist / (0.02 * ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on  = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
//TODO: here used the fixed length spline, need to upgrade this spline function so that it can adjust according to driving environment, according to cost functions, can take quick turn sometimes
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // start the run with straight forward line
//          double dist_inc = 0.5;
//          for (int i = 0; i < 50; ++i) {
//            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//          }
          
          //Try more complex paths
//          double pos_x;
//          double pos_y;
//          double angle;
//          int path_size = previous_path_x.size();
//
//          for (int i = 0; i < path_size; ++i) {
//            next_x_vals.push_back(previous_path_x[i]);
//            next_y_vals.push_back(previous_path_y[i]);
//          }
//
//          if (path_size == 0) {
//            pos_x = car_x;
//            pos_y = car_y;
//            angle = deg2rad(car_yaw);
//          } else {
//            pos_x = previous_path_x[path_size-1];
//            pos_y = previous_path_y[path_size-1];
//
//            double pos_x2 = previous_path_x[path_size-2];
//            double pos_y2 = previous_path_y[path_size-2];
//            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//          }
//
//          double dist_inc = 0.5;
//          for (int i = 0; i < 50-path_size; ++i) {
//            next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
//            next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
//            pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
//            pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
//          }
          
//           // keep the car in the lane, by using Frenet s, d coordinates
//            double dist_inc = 0.5;
//            for (int i = 0; i < 50; ++i) {
//              double next_s = car_s + (i+1)*dist_inc;
//              double next_d = 6;
//              vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//              next_x_vals.push_back(xy[0]);
//              next_y_vals.push_back(xy[1]);
//            }
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
