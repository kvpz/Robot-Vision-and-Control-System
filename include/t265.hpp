#ifndef T265_HPP
#define T265_HPP
#include "includes.hpp"
#include <boost/asio.hpp>
  std::mutex mutex;

class T265
{
public:
  T265()
  {

  }

  int Start() 
  {
    if (!device_with_streams({ RS2_STREAM_POSE }, serial_t265_str))
      return EXIT_SUCCESS;

    if (!serial_t265_str.empty())
      cfg.enable_device(serial_t265_str);

    // Add pose stream. The pose data packed as float arrays containing translation vector
    // rotation quaternion and prediction velocities and acceleration vectors
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    /*
    auto callback = [&](const rs2::frame& frame)
    {

    };
    */

    profiles = pipe.start(cfg, callback);
  }

  }

  // getters
  inline double getYaw() { return yaw; }
  
  // setters

private:
  rs2::pipeline_profile profiles; // = pipe.start(cfg, callback);
  std::string serial_t265_str;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  // The pipeline is used to simplify the user interaction with the device and CV modules. 
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  // T265 gyroscope data
  double y_quaternion; //double current_angle_quaternion;
  double yaw;
  
  // T265 accelerometer data
  double current_x;
  double current_y;


  
};

#endif // T265_HPP
