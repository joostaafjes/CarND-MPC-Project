#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2.67;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          /**
           * convert waypoint to car coordinate system
           */
          std::cout << "Before:" << std::endl;
          for (int index = 0; index << ptsy.size(); index++) {
            std::cout << ptsy.at(index) << " ";
          }
          for (int index = 0; index < ptsx.size(); index++) {
            double diff_ptsx = ptsx[index] - px;
            double diff_ptsy = ptsy[index] - py;
            ptsx[index] = cos(psi) * diff_ptsx + sin(psi) * diff_ptsy;
            ptsy[index] = cos(psi) * diff_ptsy - sin(psi) * diff_ptsx;
          }

          /*
           * convert vector to Eigen Vector
           */
          VectorXd wptsx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          VectorXd wptsy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());
          std::cout << "After:" << std::endl;
          std::cout << "x:" << wptsx << std::endl;
          std::cout << "y:" << wptsy << std::endl;

          /**
           * Fit a polynomial to the above x and y coordinates
           */
          auto coeffs = polyfit(wptsx, wptsy, 3);
          std::cout << "Coeffs:" << coeffs << std::endl;

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          /**
           * Determine state parameters in car coordinate system at dt
           * and calculate state after 100ms because of latency between actuation commands
           */
          double latency = 0.10;
          double py_car = 0.0;
          double px_car = latency * v;
          double psi_car = -v / Lf * steering_angle * latency;
          double v_car = v + throttle * latency;

          /**
           * Calculate the cross track error
           */
          double cte =  polyeval(coeffs, 0) + v * CppAD::sin(-atan(coeffs[1])) * latency;

          /**
           * Calculate the orientation error
           */
          double epsi = psi_car -atan(coeffs[1]);

          std::cout << "Before(car coordinates):" << std::endl;
          std::cout << "x = " << px_car << std::endl;
          std::cout << "y = " << py_car << std::endl;
          std::cout << "psi = " << psi_car << std::endl;
          std::cout << "v = " << v_car << std::endl;
          std::cout << "cte = " << cte << std::endl;
          std::cout << "epsi = " << epsi << std::endl;

          VectorXd state(6);
          state << px_car, py_car, psi_car, v_car, cte, epsi;
          std::vector<double> result = mpc.Solve(state, coeffs);

          double steer_value = result[0];
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = steer_value / deg2rad(25.0);

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
           for (int index = 2; index < result.size(); index += 2) {
             mpc_x_vals.push_back(result[index]);
             mpc_y_vals.push_back(result[index + 1]);
           }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */

          double poly_inc = 2.5;
          int num_points = 25;
          for ( int i = 0; i < num_points; i++ ) {
            double x = poly_inc * i;
            next_x_vals.push_back( x );
            next_y_vals.push_back( polyeval(coeffs, x) );
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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