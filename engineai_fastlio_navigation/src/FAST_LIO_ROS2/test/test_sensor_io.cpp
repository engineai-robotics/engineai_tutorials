#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_io.hpp"

using namespace std::chrono_literals;

struct Result { size_t received; double max_age; };

Result exercise(bool concurrent)
{
    auto receiver = std::make_shared<rclcpp::Node>(concurrent ? "sensor_io_separate" : "sensor_io_shared");
    auto sender = std::make_shared<rclcpp::Node>(concurrent ? "sensor_sender_separate" : "sensor_sender_shared");
    const auto topic = "/offline_sensor_io_"+std::to_string(getpid())+(concurrent ? "_separate" : "_shared");
    auto options = make_sensor_io_options(*receiver, concurrent, 200);
    std::vector<double> ages;
    auto sub = receiver->create_subscription<sensor_msgs::msg::Imu>(topic, options.imu_qos,
        [&](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            ages.push_back((receiver->now()-rclcpp::Time(msg->header.stamp)).seconds());
        }, options.subscription);
    auto timer = receiver->create_wall_timer(20ms, []() { std::this_thread::sleep_for(120ms); });
    auto pub = sender->create_publisher<sensor_msgs::msg::Imu>(topic, 200);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(receiver);
    std::thread spinning([&]() { executor.spin(); });
    try {
        const auto deadline = std::chrono::steady_clock::now()+3s;
        while (pub->get_subscription_count() == 0) {
            if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("offline discovery timeout");
            std::this_thread::sleep_for(10ms);
        }
        for (int i=0; i<200; ++i) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp=sender->now();
            pub->publish(msg);
            std::this_thread::sleep_for(5ms);
        }
        std::this_thread::sleep_for(50ms);
    } catch (...) {
        executor.cancel(); spinning.join(); throw;
    }
    executor.cancel(); spinning.join(); executor.remove_node(receiver);
    double maximum=0;
    for (double age:ages) maximum=std::max(maximum,age);
    return {ages.size(),maximum};
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    int exit_code=0;
    try {
        auto validation=std::make_shared<rclcpp::Node>("sensor_io_parameter_validation");
        bool rejected=false;
        try { make_sensor_io_options(*validation,true,0); }
        catch (const std::invalid_argument &) { rejected=true; }
        if (!rejected) throw std::runtime_error("zero-depth IMU queue accepted");
        const auto old=exercise(false), current=exercise(true);
        std::cout << "{\"shared_group\":{\"received\":" << old.received << ",\"max_age_sec\":" << old.max_age
                  << "},\"separate_group\":{\"received\":" << current.received << ",\"max_age_sec\":" << current.max_age << "}}\n";
        if (current.received<180 || current.received<old.received*3 || current.max_age>.15)
            throw std::runtime_error("sensor input still starved by the delayed processing callback");
    } catch (const std::exception & error) {
        std::cerr << error.what() << '\n';exit_code=1;
    }
    rclcpp::shutdown();
    return exit_code;
}
