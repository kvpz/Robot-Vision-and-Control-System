// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>

#include <math.h>
#include <float.h>
#include "example-utils.hpp"

#include <boost/asio.hpp>

#define FEET_IN_METER 3.28084
#define INCHES_IN_METER 39.3701
#define DEBUG false

size_t waypointToVisit = 0;

/*
std::pair<float, float> origin(0.0f, 0.0f);
std::pair<float, float> origin_top(0.0f, -0.02f);
std::pair<float, float> bottom_left_attraction(-0.9f, 0.1524f);
std::pair<float, float> top_left_attraction(-0.9f, -0.45f);
std::pair<float, float> top_right_recycling(0.9f, -0.45f);
std::pair<float, float> bottom_right_recycling(0.9f, 0.0f);
*/

// waypoints (inches)
std::pair<float, float> origin(0.0f, 0.0f);
std::pair<float, float> origin_top(0.0f, -2.0f);
std::pair<float, float> bottom_left_attraction(-41.0f, 7.0f);
std::pair<float, float> top_left_attraction(-41.0f, -32.0f);
std::pair<float, float> top_right_recycling(41.0f, -32.0f);
std::pair<float, float> bottom_right_recycling(41.0f, 7.0f);

std::vector<std::pair<float,float>> waypoints({
    origin_top,
    bottom_left_attraction, 
    top_left_attraction, 
    top_right_recycling, 
    bottom_right_recycling
});

std::vector<bool> waypointReached(4);

bool is_rotating = false;
bool is_forward = false;
bool is_stop = false;
bool is_waypoint_0_half = false;
bool is_waypoint_1_half = false;
bool is_waypoint_2_half = false;
bool is_waypoint_3_half = false;
bool is_waypoint_4_half = false;

inline rs2_quaternion quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}


rs2_pose predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
            dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
            dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
            dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}

static void send_command(std::string message) 
{
    // send data via serial
    
    if(DEBUG)
    {
        std::cout << "(DEBUG) Serial message: " << message << std::endl;
    }
    else 
    {
        std::cout << "(RELEASE) Serial message: " << message << std::endl;
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, "/dev/ttyACM0");

        // Set the baud rate, character size, flow control, and parity options.
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        boost::asio::write(serial, boost::asio::buffer(message.c_str(), message.size()));
    }
}

static inline bool approximately(float current, float expected, float precision)
{
    return (current > (expected - precision)) && (current < (expected + precision));
}

int main(int argc, char * argv[]) try
{
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE }, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex mutex;
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();
            auto now = std::chrono::system_clock::now().time_since_epoch();
            double now_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
            double pose_time_ms = fp.get_timestamp();
            float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms)/1000.));
            rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
            float currentXinches = pose_data.translation.x * INCHES_IN_METER;
            float currentZinches = pose_data.translation.z * INCHES_IN_METER;
            float waypointX = (float)std::get<0>(waypoints[waypointToVisit]);
            float waypointZ = (float)std::get<1>(waypoints[waypointToVisit]);
            float precision = 1.75f;

            if(waypointToVisit == 0 && is_stop == false && is_waypoint_0_half == false && approximately(currentXinches, waypointX, precision) && 
                approximately(currentZinches, waypointZ, precision))
            {
                std::cout << "reached front of origin" << std::endl;
                waypointReached[0] = true;
                // rotate robot to the left.
                send_command("S");
                is_forward = false;
                is_stop = true;
                //send_command("Z");
                /*
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("Z");
                std::cout << "current yaw: " << pose_data.rotation.y << std::endl;
                //std::cout << "Rotating CCW until yaw is -90 degrees" << std::endl;
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                ++waypointToVisit;
                */
            }
            else if(waypointToVisit == 0 && is_stop && is_rotating == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                is_stop = false;
                send_command("C");
                is_rotating = true;
                is_waypoint_0_half = true;
            }
            else if(waypointToVisit == 0 && is_rotating && is_waypoint_0_half && !approximately(pose_data.rotation.y, 0.707f, 0.05))
            {
                // do nothing. robot is rotating
            }
            else if(is_rotating && waypointToVisit == 0)
            {
                // do nothing. robot is rotating
                send_command("S");
                is_rotating = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                is_forward = true;
                waypointToVisit++;
            }

            // go to bottom left
            else if(waypointToVisit == 1 && is_stop == false && is_waypoint_1_half == false && approximately(currentXinches, waypointX, precision) && 
                approximately(currentZinches, waypointZ, precision)
                ) 
            {
                std::cout << "reached bottom left attraction" << std::endl;
                waypointReached[1] = true;
                send_command("S");
                is_forward = false;
                is_stop = true;
                /*
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("C");
                std::cout << "current yaw: " << pose_data.rotation.y << std::endl;
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                ++waypointToVisit;
                */
            }
            else if(waypointToVisit == 1 && is_stop && is_rotating == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                is_stop = false;
                send_command("Z");
                is_rotating = true;
                is_waypoint_1_half = true;
            }
            else if(waypointToVisit == 1 && is_rotating && is_waypoint_1_half && !approximately(pose_data.rotation.y, 0.0f, 0.05))
            {
                // do nothing. robot is rotating
            }
            else if(is_rotating && waypointToVisit == 1)
            {
                // do nothing. robot is rotating
                send_command("S");
                is_rotating = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                is_forward = true;
                waypointToVisit++;
            }

            // go to top left
            else if(waypointToVisit == 2 && is_stop == false && is_waypoint_2_half == false && approximately(currentXinches, waypointX, precision) && 
                approximately(currentZinches, waypointZ, precision))
            {
                std::cout << "reached top left attraction" << std::endl;
                send_command("S");
                is_forward = false;
                is_stop = true;
                /*
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("C");
                std::cout << "current yaw: " << pose_data.rotation.y << std::endl;
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");                
                ++waypointToVisit;
                */
            } 
            else if(waypointToVisit == 2 && is_stop && is_rotating == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                is_stop = false;
                send_command("Z");
                is_rotating = true;
                is_waypoint_2_half = true;
            }
            else if(waypointToVisit == 2 && is_rotating && is_waypoint_2_half && !approximately(pose_data.rotation.y, -0.707f, 0.05))
            {
                // do nothing. robot is rotating
            }
            else if(is_rotating && waypointToVisit == 2)
            {
                // do nothing. robot is rotating
                send_command("S");
                is_rotating = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                is_forward = true;
                waypointToVisit++;
            }

            // go to bottom right corner
            else if(waypointToVisit == 3 && is_stop == false && is_waypoint_3_half == false && approximately(currentXinches, waypointX, precision) && 
                approximately(currentZinches, waypointZ, precision))
            {
                std::cout << "reached top right recycling area" << std::endl;
                send_command("S");
                is_forward = false;
                is_stop = true;

                /*
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("C");
                std::cout << "current yaw: " << pose_data.rotation.y << std::endl;
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                ++waypointToVisit;
                */
            }
            else if(waypointToVisit == 3 && is_stop && is_rotating == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                is_stop = false;
                send_command("Z");
                is_rotating = true;
                is_waypoint_3_half = true;
            }
            else if(waypointToVisit == 3 && is_rotating && is_waypoint_3_half && !approximately(pose_data.rotation.y, -0.999f, 0.05))
            {
                // do nothing. robot is rotating
            }
            else if(is_rotating && waypointToVisit == 3)
            {
                // do nothing. robot is rotating
                send_command("S");
                is_rotating = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                is_forward = true;
                waypointToVisit++;
            }

            else if(waypointToVisit == 4 && is_stop == false && is_waypoint_4_half == false && approximately(currentXinches, waypointX, precision) && 
                approximately(currentZinches, waypointZ, precision))
            {
                std::cout << "reached bottom right recycling area" << std::endl;
                send_command("S");
                is_stop = true;
                is_forward = false;
                waypointToVisit = 1;
                /*
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("C");
                std::cout << "current yaw: " << pose_data.rotation.y << std::endl;
                if(DEBUG) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                send_command("F");
                waypointToVisit = 1;
                */
            } 
            else 
            {
                std::cout << "Predicted " << std::fixed << std::setprecision(3) << dt_s*1000 << "ms " <<
                "Confidence: " << pose_data.tracker_confidence << " T: " <<
                currentXinches  << " " <<
                //predicted_pose.translation.y * FOOT  << " " <<
                currentZinches << " (inches) "
                << "Next waypoint (" << waypointToVisit << "): ( " << waypointX << " , " << waypointZ << " )"
                << "   \r";

                is_waypoint_0_half = false;
                is_waypoint_1_half = false;
                is_waypoint_2_half = false;
                is_waypoint_3_half = false;
                is_waypoint_4_half = false;
            }
        }
    };

    send_command("S");
    send_command("F");
    is_forward = true;
    
    // Start streaming through the callback with default recommended configuration
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    std::cout << "started thread\n";
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
