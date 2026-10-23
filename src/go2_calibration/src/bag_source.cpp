#include "go2_calibration/bag_source.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

namespace go2_calibration {
    namespace {
    template <typename Message>
    Message deserialize(const rosbag2_storage::SerializedBagMessage &bag_message)
    {
        const rclcpp::Serialization<Message> serialization;
        const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
        Message message;
        serialization.deserialize_message(&serialized, &message);
        return message;
    }
    }  // namespace

    void read_bag(const std::string &bag, const Topics &topics, CalibrationBuffer &buffer)
    {
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_cpp::StorageOptions storage;
        storage.uri = bag;
        storage.storage_id = "sqlite3";
        try {
            reader.open(storage, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
        } catch (const std::exception &error) {
            throw std::runtime_error("cannot open bag " + bag + ": " + error.what());
        }

        // every topic must be in the bag, and only these are read.
        std::vector<std::string> in_bag;
        for (const auto &topic : reader.get_all_topics_and_types()) {
            in_bag.push_back(topic.name);
        }
        rosbag2_storage::StorageFilter filter;
        for (const std::string &topic : {topics.phase, topics.imu, topics.body_state, topics.cloud, topics.cloud_base}) {
            if (std::find(in_bag.begin(), in_bag.end(), topic) == in_bag.end()) {
                throw std::runtime_error(bag + " has no " + topic);
            }
            filter.topics.push_back(topic);
        }
        reader.set_filter(filter);

        while (reader.has_next()) {
            const auto message = reader.read_next();
            const std::string &topic = message->topic_name;
            const std::int64_t time_ns = message->time_stamp;

            if (topic == topics.phase) {
                buffer.set_phase(deserialize<std_msgs::msg::String>(*message).data);
            } else if (topic == topics.imu) {
                const auto imu = deserialize<sensor_msgs::msg::Imu>(*message);
                buffer.add_lidar_imu(time_ns, to_eigen(imu.angular_velocity), to_eigen(imu.linear_acceleration));
            } else if (topic == topics.body_state) {
                buffer.add_body_gyro(time_ns, body_gyro(deserialize<unitree_go::msg::SportModeState>(*message)));
            } else if (buffer.wants_scans()) {  // one of the two clouds; decoding them is the slow part
                const auto cloud = deserialize<sensor_msgs::msg::PointCloud2>(*message);
                buffer.add_scan(to_nanoseconds(cloud.header.stamp), to_scan_points(cloud), topic == topics.cloud_base);
            }
        }
    }
}
