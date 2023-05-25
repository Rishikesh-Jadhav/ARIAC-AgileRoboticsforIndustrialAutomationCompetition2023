#include <rclcpp/rclcpp.hpp>
#include "sensor_camera.hpp"


// ================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorCamera>("sensor_camera");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // node->StartCompetition();
    // executor.spin();
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    // Start Competition
    node->StartCompetition();
    // Replace the following line with functions to do pick and place
    while (rclcpp::ok())
    {
    }

    rclcpp::shutdown();
}
