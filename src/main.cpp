#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <boost/algorithm/clamp.hpp>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// for PID and twiddle
double MAX_ANGLE = 100.0;
double MAX_SPD_MOD = 1.0;
double MIN_SPD_MOD = 0.25; 
double HI_ANGLE = 5.0;

int main() {
  uWS::Hub h;

  PID pid;
  PID spd_pid;
  /**
   * Initialize the pid variable.
   */ 

  double kd = 0.01;
  double kp = 0.1;
  double ki = 0.000;
  pid.Init(kp, ki, kd);
  spd_pid.Init(0.3, 0.00, 0.001);
  int iter = 0; 
  double tol = 0.0005  ;
  double best_cte = 100000;
  double throttle_set = 0.2;
  bool tune_steering = false;
  bool hs_tuning = false;
  bool tune_throttle = false;
  double target_speed = 20.0;
  double best_spd_err = 100000;
  bool spd_tuned = false;

  h.onMessage([&pid, &spd_pid, &iter, &tol, &best_cte, &best_spd_err, &hs_tuning,
                &throttle_set, &target_speed, &spd_tuned,
                &tune_steering, &tune_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * Calculate steering value--remember the steering value is
           *   [-1, 1]
           */

          double spd_err = speed - target_speed;     

          if (iter == 20) {
            best_cte = fabs(cte);
            tune_steering = true;
            //spd_tuned = true;
            }


          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          steer_value = boost::algorithm::clamp(steer_value, -1.0, 1.0);
          iter+=1;


          if (tune_steering == true) {
            std::cout << "cte/best: " << cte << " / " << best_cte << std::endl;
            pid.Twiddle(best_cte, cte);
            std::cout << "  --- tol: "<< pid.TolCheck() << std::endl;

            if (pid.TolCheck() < tol) {
              std::cout << "*************** steering tuned ***************" << std::endl;
              tune_steering = false;
              tune_throttle = true;
              best_spd_err = spd_err;
              hs_tuning = false;
            }
          }

          // if (tune_throttle) {
          //   std::cout << "speed err/best: " << spd_err << " / " << best_spd_err << std::endl;
          //   spd_pid.Twiddle(best_spd_err, spd_err);
          //   std::cout << "  --- tol: "<< spd_pid.TolCheck() << std::endl;

          //   if (spd_pid.TolCheck() < tol) {
          //     tune_throttle = false;
          //     spd_tuned = true;
          //   }
          // }

          if (iter % 3000 == 0 && hs_tuning == false) {
            tune_steering = true;
            hs_tuning = true;
            pid.RetrainPID();
          }

          if (tune_throttle) {
            spd_tuned = true;
            }

          
          if (spd_tuned) {
            spd_pid.UpdateError(spd_err);
            throttle_set = spd_pid.TotalError();
            throttle_set = boost::algorithm::clamp(throttle_set, 0.0, 0.5);

            double spd_mod = 1.0;
            if (fabs(angle) > HI_ANGLE) {
              spd_mod = 0.25;
            } else {
              double low_mod = 1.0 / (HI_ANGLE + 1);
              spd_mod = 1.0 / (fabs(angle) + 1);

              spd_mod =  (spd_mod - low_mod)/(1.0 - low_mod) * (MAX_SPD_MOD - MIN_SPD_MOD) + MIN_SPD_MOD;
              // spd_mod = boost::algorithm::clamp(spd_mod, 0.5, 1.0);            
            }
            //std::cout << spd_mod << std::endl;
            throttle_set *= spd_mod;

          }



          // DEBUG

          // std::cout << iter << " ->  CTE: " << cte << " Steering Value: " << steer_value  << "   Throttle: " << throttle_set
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_set;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    

        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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