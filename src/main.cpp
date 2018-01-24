#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>


#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
// for convenience
using json = nlohmann::json;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals,int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A(xvals.size(), order + 1);

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

double polyeval(VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
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



int main(int argc, char *argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  size_t N=12;
  double dt = 0.1;
  if (argc>1)
  {
//      N = atoi(argv[1]);
//      dt = atof(argv[2]);
//      mpc.init(N,dt);
      double refV = atof(argv[1]);
      mpc.setRefV(refV);
       std::cout <<"Set parameters for MPC: "<< N <<","<<dt <<","<<refV<< std::endl;

  }



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

          // waypoints in global coords
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          // car positions in global coords
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];  // psi is in degrees, cos and sin work on radians, should be deg2rad(psi)!
		  //cout<<"psi from "<<j[1]["psi"]<<" to "<<psi<<endl;
          double v = j[1]["speed"];  // mph
          v = v*0.44704;  // m/s
          double delta = j[1]["steering_angle"];  // (-1,1)
          cout<<"delta received: "<<delta<<endl;
          //delta = -delta *deg2rad(25);   // Is this correct?
          delta = -delta;
          double a = j[1]["throttle"];


          // change from global coords to car coords
          VectorXd x_car = VectorXd( ptsx.size() );
          VectorXd y_car = VectorXd( ptsy.size() );
          for (size_t i=0;i<ptsx.size(); i++)
          {
              double dx = ptsx[i] - px;
              double dy = ptsy[i] - py;
              x_car[i] = dx*cos(psi) + dy*sin(psi);
              y_car[i] = -dx*sin(psi) + dy*cos(psi);
          }


          // fit the waypoints to polynomial of order 3
          auto coeffs = polyfit(x_car, y_car, 3);

          // define initial state
          // x,y,psi start at 0 for a car at the beginning
          double x0 = 0.0;
          double y0 = 0.0;
          double psi0 = 0.0;
          double v0 = v;  // m/s
          double cte0 = polyeval(coeffs, x0)-y0;
          double epsi0 =  -atan(coeffs[1]); //psi0 -atan(mpc.polyFirstDeriv(x0))

           // 100ms latency
          double latency = 0.1; // 100 ms
          double Lf = 2.67; // m
          double x1 = x0 + v0*cos(psi0)*latency;
          double y1 = y0 + v0*sin(psi0)*latency;
          double psi1 = psi0 + v0/Lf* delta * latency;
          double cte1 = cte0 + v0 * sin(epsi0)*latency;
          double epsi1 = epsi0 + v0/Lf* delta * latency ;
          double v1 = v0 + a * latency;

          VectorXd initState(6);
          initState << x1,y1,psi1,v1,cte1,epsi1;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto result = mpc.Solve(initState, coeffs);

          size_t N = mpc.getN();
          double delta_value =  result[2*N];
          double throttle_value = result[2*N+1];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -delta_value /(deg2rad(25)*Lf);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals(N);
          vector<double> mpc_y_vals(N);

          for (size_t i=0;i<N;i++)
          {
            mpc_x_vals[i]=result[i];
            mpc_y_vals[i] = result[i+N];
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals ;
          vector<double> next_y_vals ;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          double di = 2.5;
          int npoint = 25;
          for (int i = 0;  i < npoint;  i++)
          {
            next_x_vals.push_back(di*i) ;
            next_y_vals.push_back(polyeval(coeffs, di*i) ) ;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
