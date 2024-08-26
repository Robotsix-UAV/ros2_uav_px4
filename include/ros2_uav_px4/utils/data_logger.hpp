#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <uav_cpp/utils/logger.hpp>


namespace uav_ros2::utils
{
class DataLogger
{
public:
    DataLogger()
    {
        std::string ros_log_directory = getRosLogDirectory();
        // Create a directory with the current date and time
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
        log_folder_ = ros_log_directory + ss.str();
        if (mkdir(log_folder_.c_str(), 0777) == -1) {
            UAVCPP_ERROR("[DataLogger] Error creating log directory: {}", log_folder_);
        }
    }

    void logToFile(std::string file_name, std::vector<std::chrono::nanoseconds> timestamps, std::vector<std::string> labels, std::vector<std::vector<double>> data)
    {
        // Check if the number of timestamps is the same as the number of data points
        if (timestamps.size() != data.size()) {
            UAVCPP_ERROR("[DataLogger] Number of timestamps ({}) is different from the number of data points ({})", timestamps.size(), data.size());
            return;
        }
        std::string log_file_path = log_folder_ + "/" + file_name;
        std::ofstream log_file(log_file_path, std::ios::out | std::ios::app);
        size_t timestamp_index = 0;
        if (log_file.is_open()) {
            log_file << "timestamp";
            // Write the labels
            for (size_t i = 0; i < labels.size(); ++i) {
                log_file << ",";
                log_file << labels[i];
            }
            log_file << std::endl;
            for (const auto &row : data) {
                if(timestamp_index < timestamps.size())
                {
                    log_file << timestamps[timestamp_index].count();
                    timestamp_index++;
                }
                for (size_t i = 0; i < row.size(); ++i) {
                    log_file << ",";
                    log_file << row[i];
                }
                log_file << std::endl;
            }
            log_file.close();
        } else {
            UAVCPP_ERROR("[DataLogger] Error opening log file: {}", log_file_path);
        }
    }

private:
    std::string log_folder_;

    std::string getRosLogDirectory() {
        auto ros_home = std::getenv("ROS_HOME");
        std::string default_log_dir;
        if(ros_home) {
            default_log_dir = std::string(ros_home) + "/log/";
        }
        else {
            std::string home = std::getenv("HOME") ? std::getenv("HOME") : "";
            default_log_dir = home + "/.ros/log/";
        }
        UAVCPP_INFO("[DataLogger] Using ROS log directory: {}", default_log_dir);
        return default_log_dir;
    }
};
}  // namespace uav_cpp::utils