#include <boost/asio.hpp>
#include <iostream>
#include <stack>
#include <vector>
#include <utility>
#include <memory>
#include <mqueue.h>
#include "robot.hpp"
#include "taskmanager.hpp"
#include "example-utils.hpp"
#include "settings.hpp"
#include <chrono>
#include <boost/timer/timer.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>

double _180_over_PI = 180.0 / M_PI;
double _PI_over_180 = M_PI / 180.0;

// xy offset from (0,0) xy base values initiated at startup of T265
double x_robot_camera_offset = 120.0;
double y_robot_camera_offset = 28.5; // 30.0
double robot_initial_orientation = 90.0;

int main(int argc, char* argv[]) try
{
    // get command line argument (tasks JSON file)
    std::string taskFile = "tasks.json";
    if(argc > 1) {
      taskFile = argv[1];
    }
    // send tasks JSON file to task manager

    std::unique_ptr<Robot> robot = std::make_unique<Robot>(x_robot_camera_offset, y_robot_camera_offset, robot_initial_orientation);

    // load tasks from JSON file
    robot->getTaskManager()->importTasksFromJSON(taskFile);

    // Setup T265 connection
    std::string serial_t265_str;
    
    if (!device_with_streams({ RS2_STREAM_POSE }, serial_t265_str))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // The pipeline is used to simplify the user interaction with the device and CV modules. 
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial_t265_str.empty())
        cfg.enable_device(serial_t265_str);

    // Add pose stream. The pose data packed as float arrays containing translation vector
    // rotation quaternion and prediction velocities and acceleration vectors
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

            double w = pose_data.rotation.w;
            double x = -1.0 * pose_data.rotation.z;
            double y = pose_data.rotation.x;
            double z = -1.0 * pose_data.rotation.y;
            double yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * _180_over_PI;
            double yaw_90_degree_offset = 90.0 - yaw;

            // translate current x,y provided by camera to center of robot relative to global map
            double current_x; // = (pose_data.translation.x) * 100.0 + x_robot_camera_offset;
            double current_y;
            if(pose_data.translation.z < 0.0) {
              current_x = (pose_data.translation.x) * 100.0 + x_robot_camera_offset;
              current_y = std::fabs(pose_data.translation.z) * 100.0 + y_robot_camera_offset;
            }
            else {
              current_x = (pose_data.translation.x) * 100.0 + x_robot_camera_offset;
              current_y = y_robot_camera_offset - ((pose_data.translation.z) * 100.0);
            }
            // calculate the (x,y) position
            int quadrant = quadrant_identifier(yaw, pose_data.rotation.y);
            if(quadrant == 1) {
              current_x = current_x - 15.0 * std::cos((90.0 - yaw) * _PI_over_180);
              current_y = current_y - 15.0 * std::sin((90.0 - yaw) * _PI_over_180);
            }
            else if(quadrant == 2) {
              current_x = current_x + 15.0 * std::cos((90.0 + yaw) * _PI_over_180);
              current_y = current_y - 15.0 * std::sin((90.0 + yaw) * _PI_over_180);
            }
            else if(quadrant == 3) {
              current_x = current_x + 15.0 * std::cos((90.0 - yaw) * _PI_over_180);
              current_y = current_y + 15.0 * std::sin((90.0 - yaw) * _PI_over_180);
            }
            else {
              current_x = current_x - 15.0 * std::cos((90.0 + yaw) * _PI_over_180);
              current_y = current_y + 15.0 * std::sin((90.0 + yaw) * _PI_over_180);
            }

            // Transfer T265 data to robot
            robot->setOrientation(yaw_to_degrees(yaw, pose_data.rotation.y));
            robot->setCurrentXY(current_x, current_y);
            // TODO: robot->getMap()->storeCurrentPositionInTime...
            
        }
    };

    // start T265 data pipeline
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    // allow time for camera to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
/*
    // wait until external LED turns on before starting procedure
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY); // open the serial port
    if (fd == -1) {
        std::cout << "Failed to open serial port" << std::endl;
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cout << "Failed to get serial port attributes" << std::endl;
        return 1;
    }

        // set serial port settings
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cout << "Failed to set serial port attributes" << std::endl;
        return 1;
    }

    while (true) {
        char buf[256];
        int n = read(fd, buf, 256); // read from serial port
        if (n > 0) {
            std::string data(buf, n);
            std::cout << "Received data: " << data << std::endl;
            std::size_t i = data.find('S');
            if(i < 256)
            {
                std::cout << "s found at: " << i << std::endl;
                break;
            }
        }
    }

    close(fd); // close the serial port
*/
    while(1) {
      boost::timer::cpu_timer timer;
      if(!robot->hasTasks()) {
        break;
      }

      robot->executeCurrentTask();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));	
      robot->printStatus();
      robot->getTaskManager()->printHighPriorityTasks();
      boost::timer::cpu_times elapsed = timer.elapsed();
      std::cout << "Elapsed time (main loop): " << (elapsed.wall / 1e9) << " seconds\n" << std::endl;
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
