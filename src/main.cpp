#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

namespace {
  double deg2rad(double x) { return x * M_PI / 180.; }
  constexpr double LATENCY           = 0.1;         // latency adjustment for MPC calculation, in seconds
  constexpr int    SIM_SLEEP_LATENCY = 100;         // in miliseconds
  constexpr double V_REF             = 60. / 2.237; //mph to mps
  constexpr double Lf                = 2.67;        // prevent declaring this twice!!
}

constexpr int VERBOSE_LEVEL = 1; // 2: full json packages, 1: everything except json, 0: nothing 

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial's derivative
double polyeval_derivative(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i-1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Eigen::VectorXd polyfit(std::vector<double> xvals, std::vector<double> yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  Eigen::VectorXd xvals_eigen(xvals.size());
  Eigen::VectorXd yvals_eigen(yvals.size());
  for (int i=0; i < xvals.size(); ++i) {
    xvals_eigen[i] = xvals[i];
    yvals_eigen[i] = yvals[i];
  }
  return polyfit(xvals_eigen, yvals_eigen, order);
}


int main() {
  uWS::Hub h;

  // user specified verbose level
  // std::cout << "Please type in VERBOSE_LEVEL (0, 1 or 2):" << std::endl;
  // std::cin >> VERBOSE_LEVEL;

  // MPC is initialized here!
  MPC mpc(VERBOSE_LEVEL, V_REF);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (VERBOSE_LEVEL == 2) {cout << sdata << endl;}
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // get the state from the simulator
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px    = j[1]["x"];
          double py    = j[1]["y"];
          double psi   = j[1]["psi"];
          double v     = double(j[1]["speed"]) / 2.237; // convert to m/s
          double delta = - double(j[1]["steering_angle"]); // minus because simulator has inversed angles
          double a     = j[1]["throttle"];


          /***************************************/
          /****** START OF MPC CALCULATIONS ******/

          // STEP 1 - update car position for latency

          double x_lat    = px  + v * std::cos(psi) * LATENCY;
          double y_lat    = py  + v * std::sin(psi) * LATENCY;
          double psi_lat  = psi + v / Lf * delta * LATENCY;
          double v_lat    = v   + a * LATENCY;

          // STEP 2 - map global ground truth path coordinates to local
          Eigen::VectorXd local_x(ptsx.size());
          Eigen::VectorXd local_y(ptsx.size());

          for (int i = 0; i < ptsx.size(); ++i) {
            double x_coord = ptsx[i] - x_lat;
            double y_coord = ptsy[i] - y_lat;
            local_x[i] =   std::cos(psi_lat) * x_coord + std::sin(psi_lat) * y_coord;
            local_y[i] = - std::sin(psi_lat) * x_coord + std::cos(psi_lat) * y_coord;
          }

          // STEP 3 - fit polynomial
          Eigen::VectorXd coeffs = polyfit(local_x, local_y, 3);

          // STEP 4 - create 6 dim state (calculate cte, epsi)
          // local state variables. The car is now at the origin so x, y, and psi are 0

          double x_loc = 0.; double y_loc = 0.; double psi_loc = 0.; double v_loc = v_lat;
          double cte_loc  = - polyeval(coeffs, x_loc);
          double epsi_loc = - std::atan(polyeval_derivative(coeffs, x_loc));
          
          Eigen::VectorXd state(6);
          state << x_loc, y_loc, psi_loc, v_loc, cte_loc, epsi_loc;

          // STEP 5 - Run solver
          vector<double> results = mpc.Solve(state, coeffs);

          /***************************************/
          /******* END OF MPC CALCULATIONS *******/

          
          

          // Send results back to simulator in correct format
          // WHY DO WE DIVIDE THIS BY deg2rad(25)??
          double steer_value = -results[0] / deg2rad(25);
          double throttle_value = results[1];

          if (VERBOSE_LEVEL >= 1) {std::cout << "steer_value: " << steer_value << std::endl;}
          if (VERBOSE_LEVEL >= 1) {std::cout << "throttle_value: " << throttle_value << std::endl;}

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;
          
          for (int i=2; i<results.size(); i += 2) {
            mpc_x_vals.push_back(results[i]);
            mpc_y_vals.push_back(results[i+1]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = {};
          vector<double> next_y_vals = {};

          for (int i = 0; i<50; i += 5) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (VERBOSE_LEVEL == 2) {std::cout << msg << std::endl;}

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(SIM_SLEEP_LATENCY));
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