#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <ctime>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Timer
std::clock_t current_time;
std::clock_t previous_time;
double delta_t;


// States
double previous_cte = 0; // Initialize cross track error
double previous_speed = 0; // Initialize car speed
double previous_theta = 0; // Initialize car heading 
double previous_x = 0; // Start point x
double previous_y = 0; // Start point y
double acceletation ; // Car acceleration
double travel ; // Distance travalled 
double theta; // car heading angle
int counter = 0; // frame counter
int N; // Num of frames rolling average, range from 1 to 8 tested
double steering_avg ; // Initialize steering average
bool isBrake = false;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/*
* Params setup in one place
* Kp, Ki, Kd, N_start, N_High_gear, Brake time, Brake threshold 
*/

//std::vector<double> p = {2.3, 0.0051, 0.55, 1, 1, 15, 1500}; // initial params works
//std::vector<double> p = {2.3, 0.0051, 2.55, 1, 1, 15, 1500}; // initial params works too
std::vector<double> p = {5.3, 0.0051, 4.55, 1, 5, 15, 1500}; // initial params survived
//std::vector<double> p = {7.3, 0.0051, 4.55, 1, 7, 15, 1500}; // initial params ok
//std::vector<double> p = {7.8, 0.0051, 4.55, 1, 8, 15, 1500}; // initial params ok



// http://stackoverflow.com/questions/10990618/calculate-rolling-moving-average-in-c
double approxRollingAverage (double avg, double input, int N) {
    avg -= avg/N;
    avg += input/N;
    return avg;
}




int main()
{


  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(p[0], p[1], p[2]);
  previous_time = clock();
  steering_avg = 0; 


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));

      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steering_value;
          double speed_value;
          // follow the udacity class material
          double err = cte*cte;  
	  double x_trajectory;
	  double y_trajectory;
          int current_time = clock();
          delta_t = (current_time-previous_time)*1.0/(CLOCKS_PER_SEC); // sec
          //std::cout <<"Delta_t = "<<delta_t<< std::endl;

          //Car travelled distance, follow the track direction, convert MPH to m/s, unit meter
          travel = 0.44704*speed*delta_t;  

          //CTE per delta time, assume in meter
          double diff_cte = cte - previous_cte;        
          //std::cout <<"Travel = "<< "\t" <<travel<< std::endl;
          //std::cout <<"CTE = "<< "\t" << diff_cte << std::endl;

          //calculate car heading angle
          if (travel !=0){
          theta = atan2(diff_cte,2.5);
          //std::cout <<"cte \t"<<cte<<"\t"<<";diff_cte \t"<<diff_cte<<"\t"<<" Steering Value:"  << steer_value << std::endl;
          }


          //update time and position
          previous_cte = cte;
          previous_time = current_time;
          previous_x = travel;
          previous_y = diff_cte;
          previous_theta = theta;

	  
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // Update P, I, D error
          pid.UpdateError(cte, delta_t, diff_cte, speed);
          
          // Up shift, increase smoothness as speed increase
          if (N<p[4]){
            N += 1;
          }

                   
          // Adaptive speed controller
          if (fabs(speed*angle) <1){
             speed_value = 1.0;
	  }
	  else{
	     speed_value = -0.3 + 120/fabs(speed*angle);
          }

          
          // Apply brake and shift down
          if (isBrake){
            if (N > 2){ 
              N -= N-1; // shift to gear 1
              speed_value = -1.0;
              isBrake = false;
            }
          }
          
          steering_value = - pid.TotalError();
          steering_value = approxRollingAverage(steering_avg, steering_value, N);
          
          if (fabs(cte/delta_t)> p[6]){ // err over threshold
              
              isBrake = true;
          }

          std::cout << counter<<"\t"<<delta_t<<"sec;\t"<< "CTE rate: " << cte/delta_t <<"\t"<< "Gear" <<"\t"<< N << std::endl;
          
          
          json msgJson;
          msgJson["steering_angle"] = steering_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          counter += 1;	  


        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
