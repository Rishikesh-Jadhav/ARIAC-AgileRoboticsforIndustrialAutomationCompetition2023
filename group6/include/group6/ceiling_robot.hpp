// #pragma once

// #include <rclcpp/qos.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

// #include <unistd.h>

// #include <cmath>

// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <geometric_shapes/shapes.h>
// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.h>

// #include "tf2/exceptions.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

// #include <kdl/frames.hpp>
// #include <tf2_kdl/tf2_kdl.h>

// #include <std_srvs/srv/trigger.hpp>

// #include <ariac_msgs/msg/order.hpp>
// #include <ariac_msgs/msg/competition_state.hpp>
// #include <ariac_msgs/msg/vacuum_gripper_state.hpp>
// #include <ariac_msgs/msg/assembly_state.hpp>
// #include <ariac_msgs/msg/agv_status.hpp>
// #include <ariac_msgs/srv/move_agv.hpp>


// #include <ariac_msgs/srv/change_gripper.hpp>
// #include <ariac_msgs/srv/vacuum_gripper_control.hpp>
// #include <ariac_msgs/srv/submit_order.hpp>
// #include <ariac_msgs/srv/perform_quality_check.hpp>
// #include <ariac_msgs/srv/get_pre_assembly_poses.hpp>

// #include <geometry_msgs/msg/pose.hpp>

// #include <ariac_msgs/msg/kit_tray_pose.hpp>
// #include <competitor_interfaces/msg/ceiling_robot_task.hpp>
// #include <competitor_interfaces/msg/completed_order.hpp>

// #include <geometry_msgs/msg/pose.hpp>

// class CeilingRobot : public rclcpp::Node
// {
// public:
//     /// Constructor
//    CeilingRobot();

//    ~CeilingRobot();

//   // Ceiling Robot Public Functions
//   void CeilingRobotSendHome();
//   bool CeilingRobotSetGripperState(bool enable);

//   // ARIAC Functions
//   bool StartCompetition();
//   bool EndCompetition();
//   bool SubmitOrder(std::string order_id);

//   //Orders
//   bool CompleteOrders();
//   bool CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task);

//     bool MoveAGV(int agv_number, int destination);
//     bool LockAGVTray(int agv_num);

// private:

//   bool CeilingRobotMovetoTarget();
//   bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
//   void CeilingRobotWaitForAttach(double timeout);
//   bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
//   bool CeilingRobotMoveToAssemblyStation(int station);
//   bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
//   bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);


//   geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

//   // Helper Functions
//   void LogPose(geometry_msgs::msg::Pose p);
//   geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
//   geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
//   geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
//   double GetYaw(geometry_msgs::msg::Pose pose);
//   geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

//   void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
//   void AddModelsToPlanningScene();

//   // Callback Groups
//   rclcpp::CallbackGroup::SharedPtr client_cb_group_;
//   rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

//   // MoveIt Interfaces 
//   moveit::planning_interface::MoveGroupInterface ceiling_robot_;
  
//   moveit::planning_interface::PlanningSceneInterface planning_scene_;

//   trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

//   // TF
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

// // Publishers
//    rclcpp::Publisher<competitor_interfaces::msg::CompletedOrder>::SharedPtr completed_order_pub_;


//   // Subscriptions
//   std::vector<competitor_interfaces::msg::CeilingRobotTask> orders_;

//   rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
//   rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;
//   rclcpp::Subscription<competitor_interfaces::msg::CeilingRobotTask>::SharedPtr ceiling_robot_task_sub_;



//   rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
//   rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
//   rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
//   rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

//   // Assembly States
//   std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

//   // Orders List
//   competitor_interfaces::msg::CeilingRobotTask current_order_;
//   unsigned int competition_state_;

//   // Gripper State
//   ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
//   ariac_msgs::msg::Part ceiling_robot_attached_part_;

//   // Trays
//   std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
//   std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;


//   // Gripper State Callback
//   void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);


//   //Task callback
//   void ceiling_robot_task_cb(const competitor_interfaces::msg::CeilingRobotTask::ConstSharedPtr msg);



//   void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
//   void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
//   void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
//   void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

//   // Orders Callback
// //   void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);

//   // Competition state callback
//   void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

//   // ARIAC Services
//   rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
//   rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
//   rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

//   // Constants
//   double kit_tray_thickness_ = 0.01;
//   double drop_height_ = 0.002;
//   double pick_offset_ = 0.003;
//   double battery_grip_offset_ = -0.05;

//   std::map<int, std::string> part_types_ = {
//     {ariac_msgs::msg::Part::BATTERY, "battery"},
//     {ariac_msgs::msg::Part::PUMP, "pump"},
//     {ariac_msgs::msg::Part::REGULATOR, "regulator"},
//     {ariac_msgs::msg::Part::SENSOR, "sensor"}
//   };

//   std::map<int, std::string> part_colors_ = {
//     {ariac_msgs::msg::Part::RED, "red"},
//     {ariac_msgs::msg::Part::BLUE, "blue"},
//     {ariac_msgs::msg::Part::GREEN, "green"},
//     {ariac_msgs::msg::Part::ORANGE, "orange"},
//     {ariac_msgs::msg::Part::PURPLE, "purple"},
//   };

//   // Part heights
//   std::map<int, double> part_heights_ = {
//     {ariac_msgs::msg::Part::BATTERY, 0.04},
//     {ariac_msgs::msg::Part::PUMP, 0.12},
//     {ariac_msgs::msg::Part::REGULATOR, 0.07},
//     {ariac_msgs::msg::Part::SENSOR, 0.07}
//   };

//   // Quadrant Offsets
//   std::map<int, std::pair<double, double>> quad_offsets_ = {
//     {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
//     {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
//     {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
//     {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
//   };


//   std::map<std::string, double> ceiling_as1_js_ = {
//     {"gantry_x_axis_joint", 1},
//     {"gantry_y_axis_joint", -3},
//     {"gantry_rotation_joint", 1.571},
//     {"ceiling_shoulder_pan_joint", 0},
//     {"ceiling_shoulder_lift_joint", -2.37},
//     {"ceiling_elbow_joint", 2.37},
//     {"ceiling_wrist_1_joint", 3.14},
//     {"ceiling_wrist_2_joint", -1.57},
//     {"ceiling_wrist_3_joint", 0}
//   };

//   std::map<std::string, double> ceiling_as2_js_ = {
//     {"gantry_x_axis_joint", -4},
//     {"gantry_y_axis_joint", -3},
//     {"gantry_rotation_joint", 1.571},
//     {"ceiling_shoulder_pan_joint", 0},
//     {"ceiling_shoulder_lift_joint", -2.37},
//     {"ceiling_elbow_joint", 2.37},
//     {"ceiling_wrist_1_joint", 3.14},
//     {"ceiling_wrist_2_joint", -1.57},
//     {"ceiling_wrist_3_joint", 0}
//   };

//   std::map<std::string, double> ceiling_as3_js_ = {
//     {"gantry_x_axis_joint", 1},
//     {"gantry_y_axis_joint", 3},
//     {"gantry_rotation_joint", 1.571},
//     {"ceiling_shoulder_pan_joint", 0},
//     {"ceiling_shoulder_lift_joint", -2.37},
//     {"ceiling_elbow_joint", 2.37},
//     {"ceiling_wrist_1_joint", 3.14},
//     {"ceiling_wrist_2_joint", -1.57},
//     {"ceiling_wrist_3_joint", 0}
//   };

//   std::map<std::string, double> ceiling_as4_js_ = {
//     {"gantry_x_axis_joint", -4},
//     {"gantry_y_axis_joint", 3},
//     {"gantry_rotation_joint", 1.571},
//     {"ceiling_shoulder_pan_joint", 0},
//     {"ceiling_shoulder_lift_joint", -2.37},
//     {"ceiling_elbow_joint", 2.37},
//     {"ceiling_wrist_1_joint", 3.14},
//     {"ceiling_wrist_2_joint", -1.57},
//     {"ceiling_wrist_3_joint", 0}
//   };

// };

