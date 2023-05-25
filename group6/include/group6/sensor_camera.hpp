#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "tf2_ros/transform_broadcaster.h"

class SensorCamera : public rclcpp::Node
{
public:
    SensorCamera(std::string node_name) : Node(node_name)
    {

        /* These define the callback groups
         * They don't really do much on their own, but they have to exist in order to
         * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
         */
        cb_group_bin_cameras_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_kit_tray_cameras_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_competition_state_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread

        auto bin_cameras_option = rclcpp::SubscriptionOptions();
        bin_cameras_option.callback_group = cb_group_bin_cameras_;

        auto kit_tray_cameras_option = rclcpp::SubscriptionOptions();
        kit_tray_cameras_option.callback_group = cb_group_kit_tray_cameras_;

        auto competition_state_option = rclcpp::SubscriptionOptions();
        competition_state_option.callback_group = cb_group_competition_state_;

        // Subscribers

        kit_tray_table1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&SensorCamera::KitTrayTable1Callback, this, std::placeholders::_1), kit_tray_cameras_option);

        kit_tray_table2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&SensorCamera::KitTrayTable1Callback, this, std::placeholders::_1), kit_tray_cameras_option);

        left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&SensorCamera::LeftBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);

        right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&SensorCamera::RightBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);

        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 1, std::bind(&SensorCamera::CompetitionStateCallback, this, std::placeholders::_1),
            competition_state_option);

        start_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");

        // Initialize the transforms broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Listen to the buffer of transforms
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    ~SensorCamera() {}

    /*===================================
    Service Clients
    =====================================*/
    bool StartCompetition();

private:
    /*!< Current state of the competition. */
    int competition_state_ = -1;
    /*!< Callback group for subscribers. */
    rclcpp::CallbackGroup::SharedPtr cb_group_competition_state_;
    rclcpp::CallbackGroup::SharedPtr cb_group_bin_cameras_;
    rclcpp::CallbackGroup::SharedPtr cb_group_kit_tray_cameras_;

    /*==============
    Services
    ==============*/
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_competition_client_;

    /*==============
    Transforms
    ==============*/
    /*!< Transform broadcaster. */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /*!< Transform message to broadcast. */
    geometry_msgs::msg::TransformStamped transf_;
    /*!< Listener for the broadcast transform message. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Pose in source frame. */
    geometry_msgs::msg::PoseStamped pose_in_;
    /*!< Pose in target frame (world). */
    geometry_msgs::msg::PoseStamped pose_out_;

    /*==============
    Subscribers
    ==============*/
    /*!< Subscriber to camera image over kit tray table1. */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kit_tray_table1_camera_sub_;
    /*!< Subscriber to camera image over kit tray table2. */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kit_tray_table2_camera_sub_;
    /*!< Subscriber to camera image over left bins. */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    /*!< Subscriber to camera image over right bins. */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    /*!< Subscriber to the state of the competition. */
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::ConstSharedPtr competition_state_sub_;
    /*===================================
    Subscriber Callbacks
    =====================================*/
    void KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

    /*===================================
    Transforms
    =====================================*/
    void ListenTransform(const std::string &source_frame);

    /*===================================
    Helper functions
    =====================================*/

    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    /**
     * @brief Helper function to convert a part type to a string
     *
     * @param part_type  Part type as an unsigned int
     * @return std::string  Part type as a string
     */
    std::string ConvertPartTypeToString(unsigned int part_type);
    /**
     * @brief Helper function to convert a part color to a string
     *
     * @param part_color  Part color as an unsigned int
     * @return std::string  Part color as a string
     */
    std::string ConvertPartColorToString(unsigned int part_color);
};