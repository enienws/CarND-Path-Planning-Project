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

//State related variables
//We are starting in middle
//Current lane
int current_lane = 1;
//Target velocity
double max_target_velocity = 49.5;
double target_velocity = 0;
double velocity_increment = 0.125;
bool velocity_override = false;
double velocity_override_time = 0;
bool lane_change_request = false;
bool lane_change = false;

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

  //Read the map
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


          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Previous path data given to the Planner
          std::vector<double> previous_path_x = j[1]["previous_path_x"];
          std::vector<double> previous_path_y = j[1]["previous_path_y"];
          int previous_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          std::vector<double> generated_x, generated_y;


          //Decision making

          //Ego vehicle's d
          float ego_d_min = current_lane * 4;
          float ego_d_max = (current_lane + 1) * 4;

          //Check for other cars
          for(int i=0; i<sensor_fusion.size(); i++)
          {
        	  //Get the s, d position of other car
        	  float d = sensor_fusion[i][6];
        	  float s = sensor_fusion[i][5];

        	  //Check whether the target car is in the same lane with ego vehicle
        	  if(d < ego_d_max && d > ego_d_min)
        	  {
        		  //Ego vehicle shares the same lane with target vehicle
        		  //Check the distance
        		  if (s > car_s)
        		  {
        			  if((s - car_s) * 2.0 < target_velocity * 1.609)
					  {
						  velocity_override = true;
						  velocity_override_time += 0.02;
						  std::cout << "VELOCITY_OVERRIDE: " << velocity_override_time << std::endl;
						  if(velocity_override_time > 0.8 && !lane_change_request)
						  {
							  lane_change_request = true;
							  velocity_override_time = 0;
							  std::cout << "LANE_CHANGE_REQUEST" << std::endl;
						  }
					  }
        		  }
        	  }
          }


          if(lane_change_request)
          {
        	  std::cout << "EVALUATING LANE CHANGE REQUEST..." << std::endl;
        	  //Determine the possible lanes
        	  std::vector<int> possible_lanes;
        	  if (current_lane == 1)
        	  {
        		  possible_lanes.push_back(0);
        		  possible_lanes.push_back(2);
        	  }
        	  else if(current_lane == 2)
        	  {
        		  possible_lanes.push_back(1);
        	  }
        	  else if(current_lane == 0)
        	  {
        		  possible_lanes.push_back(1);
        	  }

        	  int appropriate_lane = -1;

              double s_max = car_s + 15;
              double s_min = car_s - 30;

              for(int j=0; j<possible_lanes.size(); j++)
              {
            	  bool is_current_lane_appropriate = true;
                  for(int i=0; i<sensor_fusion.size(); i++)
                  {
                	  std::cout << "object id: " << sensor_fusion[i][0] << std::endl;
                	  //Get the s, d position of other car
            		  float d = sensor_fusion[i][6];
            		  float s = sensor_fusion[i][5];

            		  int other_car_lane;
            		  if(d > 0 && d < 4)
            			  other_car_lane = 0;
            		  else if(d > 4 && d < 8)
            			  other_car_lane = 1;
            		  else if(d > 8 && d < 12)
            			  other_car_lane = 2;

            		  //Check whether this car is in the lane we are controlling for..
            		  if (other_car_lane != possible_lanes[j])
            			  //If it is in a different lane simply do not perform more action on this car..
            			  continue;

            		  if (s < s_max && s > s_min)
                	  {
            			  //This car is avoiding a lane change to the current lane
            			  is_current_lane_appropriate = false;
            			  //Do not continue to control on this lane anymore..
            			  break;
                	  }
                  }

                  if(is_current_lane_appropriate)
                  {
                	  //This lane is appropriate
					  //Switch to this lane
					  current_lane = possible_lanes[j];
                	  //current_lane = 2;
					  lane_change = true;
					  lane_change_request = false;
					  std::cout << "LANE_CHANGE" << std::endl;
					  //Do not need to check other lanes, if there exists any..
					  break;
                  }
              }
          }


          //Adjust the speed
          if (velocity_override)
          {
			  target_velocity -= 5 * velocity_increment;
			  velocity_override = false;
          }
          else if(target_velocity < max_target_velocity)
        	  target_velocity += velocity_increment;
          else
        	  target_velocity = max_target_velocity;

          //Path Generation
          //Check simulator's current size of pts to consume
          if(previous_size < 2)
          {
        	  //We don't have many points to add points that are tangent to current trajectory
        	  //Generate a virtual previous point
        	  double prev_car_x = car_x - cos(car_yaw);
        	  double prev_car_y = car_y - sin(car_yaw);

        	  //Push back the virtual points
        	  generated_x.push_back(prev_car_x);
        	  generated_y.push_back(prev_car_y);

        	  //Push back the car's current position
        	  generated_x.push_back(car_x);
        	  generated_y.push_back(car_y);

          }
          else
          {
        	  //We have enough points..
        	  //Fetch the last two points from previous points in order to generate
        	  //tangent points to the previous path
              ref_x = previous_path_x[previous_size - 1];
              ref_y = previous_path_y[previous_size - 1];

              double ref_x_prev = previous_path_x[previous_size - 2];
              double ref_y_prev = previous_path_y[previous_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              generated_x.push_back(ref_x_prev);
              generated_x.push_back(ref_x);

              generated_y.push_back(ref_y_prev);
              generated_y.push_back(ref_y);

          }

          //Append 30 meters spaced points to the  generated points list
//          for(int i=1; i<4; i++)
//          {
//        	  std::vector<double> next_wp;
//
//			  next_wp = getXY(car_s + i * 30, 4*current_lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//        	  generated_x.push_back(next_wp[0]);
//        	  generated_y.push_back(next_wp[1]);
//          }
          {
				// Setting up target points in the future.
				vector<double> next_wp0 = getXY(car_s + 45, 2 + 4*current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp1 = getXY(car_s + 90, 2 + 4*current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp2 = getXY(car_s + 135, 2 + 4*current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

				generated_x.push_back(next_wp0[0]);
				generated_x.push_back(next_wp1[0]);
				generated_x.push_back(next_wp2[0]);

				generated_y.push_back(next_wp0[1]);
				generated_y.push_back(next_wp1[1]);
				generated_y.push_back(next_wp2[1]);
          }

          //Transform the points to the car's reference of frame
          for(int i=0; i<generated_x.size(); i++)
          {
        	  double shift_x = generated_x[i] - ref_x;
        	  double shift_y = generated_y[i] - ref_y;

        	  generated_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        	  generated_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          //generated_x and generated_y consists of some pivot points
          //Create a spline, a smoother path, in order for the car to follow.
          tk::spline s;
          s.set_points(generated_x, generated_y);

          //Append all the previous points as next points to the list
          for(int i=0; i<previous_path_x.size(); i++)
          {
        	  next_x_vals.push_back(previous_path_x[i]);
        	  next_y_vals.push_back(previous_path_y[i]);
          }
          //std::copy(generated_x.begin(), generated_x.end(), next_x_vals.end());
          //std::copy(generated_y.begin(), generated_y.end(), next_y_vals.end());

          //Get the target point from the spline
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double N = target_dist / (0.02f * target_velocity / 2.24f);
          double x_spacing = target_x / N;


          double x_add_on = 0;
          for(int i=1; i <= 50-previous_size; i++)
          {
				double x_point = x_add_on + x_spacing;
				double y_point = s(x_point);
				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
				y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
          }


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
