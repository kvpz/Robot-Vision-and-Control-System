#include "t265.hpp"

void my_callback(const rs2::frame& frame)
{
    std::lock_guard<std::mutex> lock(mutex);
    std::cout << "Received frame " << frame.get_frame_number() << std::endl;
}

int main()
{
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
    /*
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();

        current_angle_quaternion = pose_data.rotation.y;

        current_x = pose_data.translation.x;
        current_y = pose_data.translation.z;

        double w = pose_data.rotation.w;
        double x = -1.0 * pose_data.rotation.z;
        double y = pose_data.rotation.x;
        double z = -1.0 * pose_data.rotation.y;

        //double pitch = -asin(2.0 * (x * z - w * y)) * (180.0 / M_PI);
        //double roll = atan((2.0 * (w*x + y*z)) /  (w*w - x*x - y*y + z*z)) * (180.0 / M_PI);
        yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * (180.0 / M_PI);

        //std::cout << "pitch: " << pitch << " roll: " << roll << " yaw: " << yaw << std::endl;
        //std::cout << "yaw                  : " << yaw << std::endl;
        //std::cout << "converter            : " << convert_quaternions_to_degrees(current_angle_quaternion) << std::endl;
        //std::cout << "pose_data.rotation.y : " << current_angle_quaternion << "\n" << std::endl;
        //std::cout << "Normalized angle     : " << yaw_to_degrees(yaw, current_angle_quaternion) << std::endl;
        //std::cout << "frame angle: " << convert_to_degrees(pose_data.rotation.y) << std::endl;
        }
    };
    */


    // Start the pipeline streaming according to the configuration.
    // The pipeline captures the samples from the device and delivers them
    // to the frame callback. 
    rs2::pipeline_profile profiles = pipe.start(cfg, my_callback);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}