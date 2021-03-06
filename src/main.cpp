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
    cout << sdata << endl;
    
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          //set up bins for x, and y in car coordinates
          vector<double> ptsx_local(6);
          vector<double> ptsy_local(6);

          //set up bins for latency projected x and y coordinates
          vector<double> ptsx_local_lat(6);
          vector<double> ptsy_local_lat(6);
          
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double delta  = j[1]["steering_angle"];
          delta = delta / deg2rad(25);
          
          double v = j[1]["speed"];
          //try adding latency to global coordinates instead of the local coordinates
          double latency = 0.250; //100ms is what is in the simulator but was easier to tune at higher speed with 250ms of latency compensation
          
          //calcualte the projected x and y assuming the same angle as before. tried to used commanded steer to predict new angle but too unstable
          double px_lat = px + v * cos(psi) * latency;
          double py_lat = py + v * sin(psi) * latency;
          
          //use this section to draw the yellow lines
          for(unsigned int i = 0; i < ptsx.size() ; ++i){
            //https://discussions.udacity.com/t/mpc-car-space-conversion-and-output-of-solve-intuition/249469/4
            ptsx_local[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
            ptsy_local[i] = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
          }

          //convert latency coordinates to local car coordinates
          for(unsigned int i = 0; i < ptsx.size() ; ++i){
            //https://discussions.udacity.com/t/mpc-car-space-conversion-and-output-of-solve-intuition/249469/4
            ptsx_local_lat[i] = (ptsx[i] - px_lat) * cos(psi) + (ptsy[i] - py_lat) * sin(psi);
            ptsy_local_lat[i] = (ptsy[i] - py_lat) * cos(psi) - (ptsx[i] - px_lat) * sin(psi);
          }
          
          //create the vectors for the polynomial fit.  Use the local latency predicted values
          Eigen::VectorXd ptsxE(6);
          ptsxE << ptsx_local_lat[0], ptsx_local_lat[1], ptsx_local_lat[2], ptsx_local_lat[3], ptsx_local_lat[4], ptsx_local_lat[5];

          Eigen::VectorXd ptsyE(6);
          ptsyE << ptsy_local_lat[0], ptsy_local_lat[1], ptsy_local_lat[2], ptsy_local_lat[3], ptsy_local_lat[4], ptsy_local_lat[5];
          
          // do the fit here. 3rd degree polynomial
          auto coeffs = polyfit(ptsxE, ptsyE, 3);
          
          double cte = polyeval(coeffs, 0);
          //std::cout <<"CTE: " <<cte << std::endl;
          double epsi = -atan(coeffs[1]);

          //assign 0s for the first elements.  x, y and psi are are the local coordiante system here so they should all be zero
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          double steer_value;
          double throttle_value;
          vector<double> outputs = mpc.Solve(state, coeffs);
          steer_value = -outputs[14]/deg2rad(25);  //minus is to the left
          throttle_value = outputs[15];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 0; i < 5; ++i){
            mpc_x_vals.push_back(outputs[i]);
            //std::cout <<"mpc x val: " <<outputs[i] << std::endl;
            mpc_y_vals.push_back(outputs[i+5]);
          }
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          next_x_vals = ptsx_local;
          next_y_vals = ptsy_local;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
           
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
