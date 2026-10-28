// Unit tests for read_bag(): a small bag written here must reach the buffer as a live run would.
#include "go2_calibration/bag_source.hpp"

#include "go2_cloud.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

namespace go2_calibration {
namespace {

// A fresh path for a bag, named after the running test.
std::string bag_path()
{
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path path = std::filesystem::temp_directory_path() / ("go2_calibration_" + name);
    std::filesystem::remove_all(path);
    return path.string();
}

// Writes messages into a new bag; the bag is complete once the writer is destroyed.
class BagWriter {
public:
    explicit BagWriter(const std::string &path)
        : writer_(std::make_unique<rosbag2_cpp::writers::SequentialWriter>())
    {
        rosbag2_cpp::StorageOptions storage;
        storage.uri = path;
        storage.storage_id = "sqlite3";
        writer_.open(storage, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
    }

    template <typename Message>
    void write(const std::string &topic, const std::string &type, const Message &message, std::int64_t time_ns)
    {
        if (topics_.insert(topic).second) {
            writer_.create_topic(rosbag2_storage::TopicMetadata{topic, type, "cdr", ""});
        }
        // Foxy's writer caches messages after write() returns, so the bag message shares ownership
        // of the serialized bytes (aliasing constructor of shared_ptr).
        const auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<Message>().serialize_message(&message, serialized.get());

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data =
            std::shared_ptr<rcutils_uint8_array_t>(serialized, &serialized->get_rcl_serialized_message());
        bag_message->time_stamp = time_ns;
        bag_message->topic_name = topic;
        writer_.write(bag_message);
    }

private:
    rosbag2_cpp::Writer writer_;
    std::set<std::string> topics_;
};

std_msgs::msg::String phase(const std::string &name)
{
    std_msgs::msg::String message;
    message.data = name;
    return message;
}

sensor_msgs::msg::Imu imu(double x, double y, double z)
{
    sensor_msgs::msg::Imu message;
    message.angular_velocity.x = x;
    message.angular_velocity.y = y;
    message.angular_velocity.z = z;
    message.linear_acceleration.z = 9.8;
    return message;
}

unitree_go::msg::SportModeState body(float x, float y, float z)
{
    unitree_go::msg::SportModeState message;
    message.imu_state.gyroscope = {x, y, z};
    message.imu_state.accelerometer = {0.0F, 0.0F, 9.5F};
    return message;
}

sensor_msgs::msg::PointCloud2 cloud(std::int32_t stamp_s)
{
    auto message = test::go2_cloud({{1.0F, 2.0F, 3.0F, 50.0F, 7, 0.01F}, {4.0F, 5.0F, 6.0F, 60.0F, 8, 0.02F}});
    message.header.stamp.sec = stamp_s;
    return message;
}

// A still part with one scan and its copy, then a turning part, as a live run records them.
void write_run(BagWriter &bag, const Topics &topics, bool with_phase)
{
    if (with_phase) {
        bag.write(topics.phase, "std_msgs/msg/String", phase("still"), 1);
    }
    bag.write(topics.imu, "sensor_msgs/msg/Imu", imu(0.1, 0.2, 0.3), 2);
    bag.write(topics.body_state, "unitree_go/msg/SportModeState", body(9.0F, 9.0F, 9.0F), 3);
    bag.write(topics.cloud, "sensor_msgs/msg/PointCloud2", cloud(100), 4);
    bag.write(topics.cloud_base, "sensor_msgs/msg/PointCloud2", cloud(100), 5);
    if (with_phase) {
        bag.write(topics.phase, "std_msgs/msg/String", phase("spin_left"), 6);
    }
    bag.write(topics.imu, "sensor_msgs/msg/Imu", imu(1.0, 0.0, 0.0), 7);
    bag.write(topics.body_state, "unitree_go/msg/SportModeState", body(1.0F, 2.0F, 3.0F), 8);
}

TEST(ReadBag, ReplaysEveryTopicThroughThePhases)
{
    const std::string path = bag_path();
    const Topics topics;
    {
        BagWriter bag(path);
        write_run(bag, topics, true);
    }

    CalibrationBuffer buffer;
    read_bag(path, topics, buffer);
    std::filesystem::remove_all(path);

    const CalibrationData &data = buffer.data();
    ASSERT_EQ(data.still_gyro.size(), 1U);
    EXPECT_TRUE(data.still_gyro[0].isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
    ASSERT_EQ(data.still_body_gyro.size(), 1U);
    EXPECT_TRUE(data.still_body_gyro[0].isApprox(Eigen::Vector3d(9.0, 9.0, 9.0)));
    ASSERT_EQ(data.still_body_accel.size(), 1U);
    EXPECT_TRUE(data.still_body_accel[0].isApprox(Eigen::Vector3d(0.0, 0.0, 9.5)));
    EXPECT_EQ(data.scans, 1U);
    EXPECT_EQ(data.lidar_points.size(), 2U);
    ASSERT_EQ(data.moving_lidar_gyro.size(), 1U);
    EXPECT_EQ(data.moving_lidar_gyro[0].time_ns, 7);  // the bag's receive time
    ASSERT_EQ(data.moving_body_gyro.size(), 1U);
    EXPECT_EQ(data.moving_body_gyro[0].time_ns, 8);
    EXPECT_TRUE(data.moving_body_gyro[0].value.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

// Without the phase, nothing says which samples were still: the bag cannot be used.
TEST(ReadBag, ThrowsWithoutThePhaseTopic)
{
    const std::string path = bag_path();
    const Topics topics;
    {
        BagWriter bag(path);
        write_run(bag, topics, false);
    }

    CalibrationBuffer buffer;
    EXPECT_THROW(read_bag(path, topics, buffer), std::runtime_error);
    std::filesystem::remove_all(path);
}

TEST(ReadBag, ThrowsForAMissingBag)
{
    CalibrationBuffer buffer;

    EXPECT_THROW(read_bag("/nonexistent/go2_calibration_bag", Topics{}, buffer), std::runtime_error);
}

}  // namespace
}  // namespace go2_calibration
