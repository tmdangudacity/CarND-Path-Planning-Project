#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct sensor_fusion_data
{
    unsigned int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return (x * pi()  / 180.0); }
double rad2deg(double x) { return (x * 180.0 / pi() ); }

// For converting back and forth between Miles per hour and meters per second.
double Mph2mps(double x) { return (x * 1.60934 / 3.6); }
double mps2Mph(double x) { return (x * 3.6 / 1.60934); }

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

    for(int i = 0; i < maps_x.size(); i++)
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

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
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
    for(int i = 0; i < prev_wp; i++)
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

int find_lane(double d)
{
    //Lane 0 limits
    static const double d_min_lane_0 = 0.0;
    static const double d_max_lane_0 = 4.0;

    //Lane 1 limits
    static const double d_min_lane_1 = d_min_lane_0 + 4.0;
    static const double d_max_lane_1 = d_max_lane_0 + 4.0;

    //Lane 2 limits
    static const double d_min_lane_2 = d_min_lane_1 + 4.0;
    static const double d_max_lane_2 = d_max_lane_1 + 4.0;

    int ret = -1;

    if( (d > d_min_lane_0) && (d < d_max_lane_0) )
    {
        ret = 0;
    }
    else if( (d > d_min_lane_1) && (d < d_max_lane_1) )
    {
        ret = 1;
    }
    else if( (d > d_min_lane_2) && (d < d_max_lane_2) )
    {
        ret = 2;
    }
    //else
    //{
        //std::cout << "Error! Not within lanes!" << std::endl;
    //}

    return ret;
}

vector<vector<int>> lanes_to_check_for_change(int current_lane)
{
    vector<vector<int>> ret(2);
    ret[0].clear();
    ret[1].clear();

    if(0 == current_lane)
    {   //1 is immediate lane to change to the Right. 2 is lane to check if other cars could potentially change lane
        ret[1] = {1, 2};
    }
    else if(1 == current_lane)
    {   // 0 is immediate lane to change to the Left.
        ret[0] = {0};

        // 2 is immediate lane to change to the Right
        ret[1] = {2};
    }
    else if(2 == current_lane)
    {
        // 1 is immediate lane to change to the Right. 0 is lane to check if other cars could potentially change lane
        ret[0] = {1, 0};
    }

    return ret;
}

//@TODO: Update distance_check to only check within a radius of 100m, then group cars into front and behind by comparing heading from the ego car to the car
//The absolute ds calculation then will need to consider wrapping of s after max_s
void distance_check(const vector<sensor_fusion_data>& check_cars, double proj_time, double proj_s, double current_s, double& closest_d_front, double& closest_d_back)
{
    double vx    = 0.0;
    double vy    = 0.0;
    double speed = 0.0;
    double ds    = 0.0;

    bool front_started = false;
    bool back_started  = false;

    for(unsigned int i = 0; i < check_cars.size(); ++i)
    {
        vx    = check_cars[i].vx;
        vy    = check_cars[i].vy;
        speed = sqrt(vx * vx + vy * vy);
        ds    = check_cars[i].s + proj_time * speed - proj_s;

        //A positive distance means a car is in front of our car in the future
        //A negative distance means a car is behind of our car in the future
        if(check_cars[i].s >= current_s)
        {
            if( (!front_started) || (ds < closest_d_front) )
            {
                closest_d_front = ds;
                front_started = true;
            }
        }
        else
        {
            if( (!back_started)|| (ds > closest_d_back))
            {
                closest_d_back = ds;
                back_started = true;
            }
        }
    }

    if(!front_started) closest_d_front =  10000.0; //If no car on the front
    if(!back_started)  closest_d_back  = -10000.0; //If no car behind
}

int main()
{
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

  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &max_s, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

              //Splitting other car sensor fusion data on lanes
              vector< vector<sensor_fusion_data> > cars_on_lanes(3);
              sensor_fusion_data temp_data;
              int lane_id = 0;

              for(unsigned int i = 0; i < sensor_fusion.size(); ++i)
              {
                  temp_data.id = sensor_fusion[i][0];
                  temp_data.x  = sensor_fusion[i][1];
                  temp_data.y  = sensor_fusion[i][2];
                  temp_data.vx = sensor_fusion[i][3];
                  temp_data.vy = sensor_fusion[i][4];
                  temp_data.s  = sensor_fusion[i][5];
                  temp_data.d  = sensor_fusion[i][6];

                  lane_id = find_lane(temp_data.d);

                  if(lane_id >= 0)
                  {
                      cars_on_lanes[lane_id].push_back(temp_data);
                  }
              }

              unsigned int prev_size = previous_path_x.size();

              //Handling wrapping around for s
              std::cout << "Max S: " << max_s << ", Car S: " << car_s << ", End path S: " << end_path_s << std::endl;

              //End of previous path.
              double proj_s = car_s;

              //s of end of previous path
              if(prev_size > 0)
              {
                  proj_s = end_path_s;
              }

              //Current car lane
              int car_lane = find_lane(car_d);

              //Lane of end of previous path
              int proj_lane = car_lane;

              if(prev_size > 0)
              {
                  proj_lane = find_lane(end_path_d);
              }

              //Potential lane to change, initialised to the lane of the end of the previous path
              int lane_to_change = proj_lane;

              double d_front = 0.0;
              double d_back  = 0.0;

              //Distance check for the lane of the end of previous path
              distance_check(cars_on_lanes[proj_lane], (prev_size * 0.02), proj_s, car_s, d_front, d_back);
              std::cout << "Car lane: " << car_lane << ", End of prev. path lane: " << proj_lane << ", Min D Front: " << d_front << ", Min D Back: " << d_back << std::endl;

              //If the car is getting to close to one in front
              bool check_lane_change = (d_front > 0.0) && (d_front < 30.0);
              bool getting_too_close = false;

              if(check_lane_change)
              {
                  std::cout << "Start checking for lane change" << std::endl;

                  vector<vector<int>> lanes_for_change = lanes_to_check_for_change(proj_lane);

                  //Set cost to max of 1.0 that means impossible to change lane.
                  double cost_change_left  = 1.0;
                  double cost_change_right = 1.0;

                  //Left lane change
                  if(lanes_for_change[0].size())
                  {
                      //@TODO: There could be one lane next to the left that also needs consideration
                      int lane_on_left = lanes_for_change[0][0];
                      double d_front_left = 0.0;
                      double d_back_left  = 0.0;

                      distance_check(cars_on_lanes[lane_on_left], (prev_size * 0.02), proj_s, car_s, d_front_left, d_back_left);
                      std::cout << "Left lane: " << lane_on_left << ", Min D Front: " << d_front_left << ", Min D Back: " << d_back_left << std::endl;

                      //Calculate cost out from d_front_left and d_back_left
                      //@TODO: Use time instead of distance ?
                      if( (d_front_left > 30.0) && (d_back_left < -30.0) )
                      {
                          cost_change_left = 60.0 / (d_front_left - d_back_left);
                      }
                  }

                  //Right lane change
                  if(lanes_for_change[1].size())
                  {
                      //@TODO: There could be one lane next to the right that also needs consideration
                      int lane_on_right = lanes_for_change[1][0];
                      double d_front_right = 0.0;
                      double d_back_right  = 0.0;

                      distance_check(cars_on_lanes[lane_on_right], (prev_size * 0.02), proj_s, car_s, d_front_right, d_back_right);
                      std::cout << "Right lane: " << lane_on_right << ", Min D Front: " << d_front_right << ", Min D Back: " << d_back_right << std::endl;

                      //Calculate cost out from d_front_right and d_back_right
                      //@TODO: Use time instead of distance?
                      if( (d_front_right > 30.0) && (d_back_right < -30.0) )
                      {
                          cost_change_right = 60.0 / (d_front_right - d_back_right);
                      }
                  }

                  std::cout << "Cost to change Left: " << cost_change_left << ", Cost to change Right: " << cost_change_right << std::endl;

                  if( (cost_change_left < 1.0) && (cost_change_right < 1.0) )
                  {
                      if(cost_change_left > cost_change_right)
                      {
                          //Change to right lane
                          lane_to_change = proj_lane + 1;

                          std::cout << "CHANGE RIGHT" << std::endl;
                      }
                      else
                      {
                          //Change to left lane
                          lane_to_change = proj_lane - 1;

                          std::cout << "CHANGE LEFT" << std::endl;
                      }
                  }
                  else if(cost_change_left < 1.0)
                  {
                      //Can only change to left lane
                      lane_to_change = proj_lane - 1;

                      std::cout << "CHANGE LEFT" << std::endl;
                  }
                  else if(cost_change_right < 1.0)
                  {
                      //Can only change to right lane
                      lane_to_change = proj_lane + 1;

                      std::cout << "CHANGE RIGHT" << std::endl;
                  }
                  else
                  {
                       //Otherwise no change lane and slow down
                       getting_too_close = true;
                       std::cout << "GETTING TOO CLOSE AND CAN'T CHANGE LANE! SLOWING DOWN!" << std::endl;
                  }
              }

              //Slowing down or speeding up at 0.1 m/s in 0.02 seconds or 5 m/s2
              if(getting_too_close)
              {
                  ref_vel -= mps2Mph(0.16);
              }
              else if(ref_vel < 49.5)
              {
                  ref_vel += mps2Mph(0.16);
              }

              //---------------------------------------------------------------
              //Trajectory generation
              vector<double> ptsx;
              vector<double> ptsy;

              double ref_x = car_x;
              double ref_y = car_y;
              double ref_yaw = deg2rad(car_yaw);

              if(prev_size < 2)
              {
                  double prev_car_x = car_x - cos(car_yaw);
                  double prev_car_y = car_y - sin(car_yaw);

                  ptsx.push_back(prev_car_x);
                  ptsx.push_back(car_x);

                  ptsy.push_back(prev_car_y);
                  ptsy.push_back(car_y);
              }
              else
              {
                  ref_x = previous_path_x.back();
                  ref_y = previous_path_y.back();

                  double ref_x_prev = previous_path_x[prev_size - 2];
                  double ref_y_prev = previous_path_y[prev_size - 2];
                  ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));

                  ptsx.push_back(ref_x_prev);
                  ptsx.push_back(ref_x);

                  ptsy.push_back(ref_y_prev);
                  ptsy.push_back(ref_y);
              }

              //In Frenet frame add 3 points 30m spaced ahead of the staring frame
              vector<double> next_wp0 = getXY(proj_s + 30.0, (2.0 + 4.0 * lane_to_change), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp1 = getXY(proj_s + 60.0, (2.0 + 4.0 * lane_to_change), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp2 = getXY(proj_s + 90.0, (2.0 + 4.0 * lane_to_change), map_waypoints_s, map_waypoints_x, map_waypoints_y);

              ptsx.push_back(next_wp0[0]);
              ptsx.push_back(next_wp1[0]);
              ptsx.push_back(next_wp2[0]);

              ptsy.push_back(next_wp0[1]);
              ptsy.push_back(next_wp1[1]);
              ptsy.push_back(next_wp2[1]);


              for(unsigned int i = 0; i < ptsx.size(); ++i)
              {
                  //Transformation from global frame to local vehicle frame
                  double shift_x = ptsx[i] - ref_x;
                  double shift_y = ptsy[i] - ref_y;

                  ptsx[i] = (shift_x * cos(0.0 - ref_yaw) - shift_y * sin(0.0 - ref_yaw));
                  ptsy[i] = (shift_x * sin(0.0 - ref_yaw) + shift_y * cos(0.0 - ref_yaw));
              }

              //Spline interpolation in local frame to avoid "singularity" in X axis
              tk::spline s;

              s.set_points(ptsx, ptsy);

              vector<double> next_x_vals;
              vector<double> next_y_vals;

              for(unsigned int i = 0; i < prev_size; ++i)
              {
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
              }

              double target_x = 30.0;
              double target_y = s(target_x);

              double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
              double N           = target_dist / (0.02 * Mph2mps(ref_vel));
              double delta_x     = target_x / N;

              double x_local  = 0.0;
              double y_local  = 0.0;
              double x_global = 0.0;
              double y_global = 0.0;

              //Logging how many added to the previous path.
              unsigned int i = 0;
              for(i = 1; i <= (50 - prev_size); ++i)
              {
                  x_local = i * delta_x;
                  y_local = s(x_local);

                  //Coverting points from local frame back to global frame
                  x_global = (x_local * cos(ref_yaw) - y_local * sin(ref_yaw)) + ref_x;
                  y_global = (x_local * sin(ref_yaw) + y_local * cos(ref_yaw)) + ref_y;

                  next_x_vals.push_back(x_global);
                  next_y_vals.push_back(y_global);
              }

              std::cout << "Previous size: " << prev_size << ", Added: " << i << ", Total: " << next_x_vals.size() << std::endl;

              //---------------------------------------------------------------

              json msgJson;

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
