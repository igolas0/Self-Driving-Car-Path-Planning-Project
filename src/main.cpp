#include <math.h>

#include <fstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); ++i)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; ++i)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  //start in lane 1 (this variable represents our target lane)
  int lane = 1;

  //define reference velocity in MPH
  double vel_ref = 0.0;

  //default Menu = 1 (Keep Lane). This will be used later to switch between states (Keep lane, lane change,...)
  int menuItem = 1; 

  h.onMessage([&menuItem, &vel_ref,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

                //store previous path size
                int prev_size = previous_path_x.size();

                //set s coordinate of the car to end of previous path
                if(prev_size > 0)
                {
                   car_s = end_path_s;
                }


                //boolean variable will be set to true if we encounter cars in our lane
                bool too_close = false;
                //variable to track ID of front car (ID in sensor fusion data)
                int next_car_front_id;
                //keep track of front car speed if vehicle too close,
                // so that we can adapt our speed
                double front_speed;


                // Loop over sensor fusion. If we encounter a car driving 
                // our same lane in front of us then set too_close
                // variable to TRUE and track ID of that car. 
                // Later we will adapt our speed based on this
                for(int i = 0; i < sensor_fusion.size(); ++i)
                {
                  float d = sensor_fusion[i][6];
                  if(d <(2+4*lane+2) && d > (2+4*lane-2))  //if car is in my lane
                  {
                     double vx = sensor_fusion[i][3];
                     double vy = sensor_fusion[i][4];
                     double check_speed = sqrt(vx*vx+vy*vy);
                     double check_car_s= sensor_fusion[i][5];
                     //project s coordinate of front car in the future based on that car's speed
                     check_car_s += ((double)prev_size * 0.02 * check_speed);
                     //check for cars in front if future paths collide
                     if((check_car_s > car_s) && ((check_car_s - car_s) < 30) )
                     {
                        too_close = true;
                        next_car_front_id = i;
                        front_speed = check_speed;
                     }
                  }
                }
                //if front vehicle too close decrease speed until we are driving 
                // at roughly same speed. Else accelerate until just below speed limit
                if(too_close && (vel_ref > front_speed))
                {
                   vel_ref -= 0.2;
                }
                else if(vel_ref < 49.5)
                { 
                   vel_ref += 0.224; 
                } 

                //defining some variables to manage lane change decisions
                bool change_left = true; 
                bool change_right = true;
                int next_car_left_id = -1;
                int next_car_right_id = -1;
                double score_left = 0.0;
                double score_right = 0.0;
                double score_center = 1.0;
                double check_car_s;
                double car_s_pos;

                //implement switch as a state machine to manage lane changes
                switch(menuItem) { 
                   //case menuItem=1 equals KEEP LANE 
                   case(1): if(too_close) 
                   {
                      menuItem = 2;
                   } 
                   break;
                   //case menuItem=2 equals PREPARE LANE CHANGE
                   case(2):   

                   // Loop over all cars in sensor fusion and assert viability of 
                   // left and right lane changes
                   for(int i = 0; i < sensor_fusion.size(); ++i)
                   {
                      float d = sensor_fusion[i][6];

                      //check space in left lane (only if I am in lane 1 or 2)
                      if(lane == 0)
                      {
                         change_left = false;
                      }
                      if(change_left)
                      {
                         if(d <(2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) )
                         {
                               double vx = sensor_fusion[i][3];
                               double vy = sensor_fusion[i][4];
                               double check_speed = sqrt(vx*vx+vy*vy);
                               check_car_s= sensor_fusion[i][5];
                               //project s coordinate in the future based on that car's speed
                               double check_car_s_p = (check_car_s + (double)prev_size * 0.02 * check_speed);

                               //get our car s position (since car_s overwritten with endpath above if prev_size > 0)
                               car_s_pos = j[1]["s"];

                               // check for available space in target lane based on current and projected positions
                               // of our and other cars. (Differenciating front and rear cars). Safety margins of 10 meters
                               // for front cars and 20 meters for cars approximating from behind.
                               if(((check_car_s > car_s_pos) && (((check_car_s_p - car_s) < 5) || ((check_car_s - car_s_pos) < 5))) 
                                 || ((check_car_s < car_s_pos) && (((car_s - check_car_s_p) < 15) || ((car_s_pos - check_car_s) < 15)))) 
                               {
                                  change_left = false;
                               }
                               //track ID of nearest front car in LEFT lane
                               if(check_car_s > car_s_pos)
                               {
                                  if(next_car_left_id == -1)
                                  {
                                     next_car_left_id = i;
                                  }
                                  else if(check_car_s < sensor_fusion[next_car_left_id][5])
                                  {
                                     next_car_left_id = i;
                                  }
                               }
                         }
                      }
                      // now the same is done for the right lane.
                      // check space in Right lane for lange change (only if I am in lane 0 or 1)
                      if(lane == 2)
                      {
                         change_right = false;
                      }
                      if(change_right)
                      {
                            if(d <(2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) )
                            {
                               double vx = sensor_fusion[i][3];
                               double vy = sensor_fusion[i][4];
                               double check_speed = sqrt(vx*vx+vy*vy);
                               check_car_s= sensor_fusion[i][5];
                               //project s coordinate in the future based on that car's speed
                               double check_car_s_p = (check_car_s + (double)prev_size * 0.02 * check_speed);

                               //get our car s position (since car_s overwritten with endpath if prev_size > 0)
                               car_s_pos = j[1]["s"];
                               if(((check_car_s > car_s_pos) && (((check_car_s_p - car_s) < 5) || ((check_car_s - car_s_pos) < 5))) 
                                 || ((check_car_s < car_s_pos) && (((car_s - check_car_s_p) < 15) || ((car_s_pos - check_car_s) < 15)))) 
                               {
                                  change_right = false;
                               }
                            }
                            //track ID of nearest front car in RIGHT lane
                            if(check_car_s > car_s_pos)
                            {
                               if(next_car_right_id == -1)
                               {
                                  next_car_right_id = i;
                               }
                               else if(check_car_s < sensor_fusion[next_car_right_id][5])
                               {
                                  next_car_right_id = i;
                               }
                            }
                      }
                   }
                   // Next we set as scoring system to determine which lane change (or lane keeping) will
                   // allow us to advance more based on projected position of front cars in each lane after 10 sec.
                   // Higher scores are better.

                   if(change_left)
                   {
                      score_left = 99999.9;
                      if(next_car_left_id != -1)  
                      {
                         //score based on extrapolating the nearest front car's s position in left lane 10s into the future (based on its speed)
                         score_left = (double)sensor_fusion[next_car_left_id][5] 
                                      + 10 * sqrt((double)sensor_fusion[next_car_left_id][3] * (double)sensor_fusion[next_car_left_id][3]
                                      + (double)sensor_fusion[next_car_left_id][4] * (double)sensor_fusion[next_car_left_id][4]);
                      }
                   }
                   if(change_right)
                   {
                      score_right = 99998.8;
                      if(next_car_right_id != -1)
                      {
                         score_right = (double)sensor_fusion[next_car_right_id][5] 
                                     + 10 * sqrt((double)sensor_fusion[next_car_right_id][3] * (double)sensor_fusion[next_car_right_id][3]
                                     + (double)sensor_fusion[next_car_right_id][4] * (double)sensor_fusion[next_car_right_id][4]);
                      }
                   }
                   score_center = (double)sensor_fusion[next_car_front_id][5] 
                                + 10 * sqrt((double)sensor_fusion[next_car_front_id][3] * (double)sensor_fusion[next_car_front_id][3]
                                + (double)sensor_fusion[next_car_front_id][4] * (double)sensor_fusion[next_car_front_id][4]);

                   //if conditions for lane change not given or front car moving faster than traffic in side lanes --> do nothing
                   if((change_left == 0.0 && change_right == 0.0) || ((score_center > score_left) && (score_center > score_right))) {}
                   else if(score_left > score_right)
                   {
                      menuItem = 3;
                   }
                   else if(score_right > score_left)
                   {
                      menuItem = 4;
                   }
                   break; 
                   //case menuItem=3 equals LANE CHANGE LEFT
                   case(3):
                   lane -= 1;
                   menuItem = 1;
                   break;
                   //case menuItem=4 equals LANE CHANGE RIGHT
                   case(4):
                   lane += 1;  
                   menuItem = 1;
                   break;
                }

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);


                if(prev_size < 2)
                {
                  //use two points that make path tangent to previus path's end point
                  double prev_car_x = car_x - cos(car_yaw);
                  double prev_car_y = car_y - sin(car_yaw);

                  ptsx.push_back(prev_car_x);
                  ptsx.push_back(car_x);

                  ptsy.push_back(prev_car_y);
                  ptsy.push_back(car_y);

                }
                else
                {
                  ref_x = previous_path_x[prev_size-1];
                  ref_y = previous_path_y[prev_size-1];

                  double ref_x_prev = previous_path_x[prev_size-2];
                  double ref_y_prev = previous_path_y[prev_size-2];
                  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                  //use two points that make path tangent to previus path's end point
                  ptsx.push_back(ref_x_prev);
                  ptsx.push_back(ref_x);

                  ptsy.push_back(ref_y_prev);
                  ptsy.push_back(ref_y);
                }
                //In Frenet add evenly 30m spaced points ahead of the starting reference (in target lane)
                vector<double> next_mp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_mp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_mp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);


                ptsx.push_back(next_mp0[0]);
                ptsx.push_back(next_mp1[0]);
                ptsx.push_back(next_mp2[0]);

                ptsy.push_back(next_mp0[1]);
                ptsy.push_back(next_mp1[1]);
                ptsy.push_back(next_mp2[1]);


                for(int i = 0; i < ptsx.size(); ++i)
                {    
                   //shift car reference angle to 0 degrees
                   double shift_x = (ptsx[i]-ref_x);
                   double shift_y = (ptsy[i]-ref_y);

                   ptsx[i] = ((shift_x * cos(0-ref_yaw)) - (shift_y * sin(0-ref_yaw)));
                   ptsy[i] = ((shift_x * sin(0-ref_yaw)) + (shift_y * cos(0-ref_yaw)));
                }
                //create spline
                tk::spline s;

                //set (x,y) points to the spline
                s.set_points(ptsx,ptsy);

                for(int i = 0; i < previous_path_x.size(); ++i)
                {    
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
                }
                //Caculate how to break up spline points to travel at desired target velocity
                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

                double x_add_on = 0;

                for(int i = 1; i <= 50-previous_path_x.size(); ++i)
                {    
                  double N = (target_dist/(0.02*vel_ref/2.24));
                  double x_point = (x_add_on+(target_x/N));
                  double y_point = s(x_point);

                  x_add_on = x_point;

                  double x_ref = x_point;
                  double y_ref = y_point;

                  //rotate back to normal coordinates
                  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
                  y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

                  x_point += ref_x;
                  y_point += ref_y;

                  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
                }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
