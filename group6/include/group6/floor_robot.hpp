                                                                                                                                                                                        #pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unistd.h>

#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>

#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
// #include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/agv_status.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>


#include <std_msgs/msg/bool.hpp>

#include <geometry_msgs/msg/pose.hpp>


#include <competitor_interfaces/msg/floor_robot_task.hpp>
#include <competitor_interfaces/msg/completed_order.hpp>
#include <competitor_interfaces/msg/ceiling_robot_task.hpp>
#include <chrono>
// #include <competitor_msgs/msg/robots_status.hpp>


#include <geometry_msgs/msg/pose.hpp>

struct repl_parts{
int agv_no;
ariac_msgs::msg::Part part_to_pick;
int quadrant;
};

class RoboCircus : public rclcpp::Node
{
public:
    /// Constructor
    RoboCircus();

    ~RoboCircus();

    // Floor Robot Public Functions
    void FloorRobotSendHome();
    void FloorRobotMoveToTrashBin(int agv_no);
    bool FloorRobotSetGripperState(bool enable);
    bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
    bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);
    bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick, int agv, int quadrant, bool priority);
    bool FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick, double offset, double rail_position);
    bool FloorRobotPlacePartonBin(int agv_number,ariac_msgs::msg::Part part_to_pick);
    bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);
    bool FloorRobotPickPartFromKitTray(int agv_num, int quadrant, ariac_msgs::msg::Part part_to_pick);
   
// new
    // Ceiling Robot Public Functions
    void CeilingRobotSendHome();
    bool CeilingRobotSetGripperState(bool enable);
// new

    bool CompleteOrders();
    bool SubmitOrder(std::string order_id);
    bool EndCompetition();
    bool ChecknComplete_HP_Orders();
    bool CompleteKittingTask(ariac_msgs::msg::KittingTask task, std::string current_order_id, bool priority);

// new
    bool CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task, std::string current_order_id, bool priority);
    // Combined
    bool CompleteCombinedTask(ariac_msgs::msg::CombinedTask task, std::string current_order_id, bool priority);
// new   
   
    std::vector<float> slotPositionCordinates( int );
    bool MoveAGV(int agv_number, int destination);
    bool LockAGVTray(int agv_num);
    
    // bool SubmitOrder(std::string order_id);



    int get_free_slot(int bin_no);

private:
    // Robot Move Functions
    // int free_slots[4][9] = {{0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}};
    float bin_cent[4][2] = {{-1.9, 3.375}, { -1.9, 3.375-0.718}, { -1.9, 3.375-5.958}, { -1.9, 3.375-6.726}};
    std::map<std::string, std::vector<int>> parts_on_tray;
    std::vector<repl_parts> replaced_parts;
    float bin_rail_pos[4] = {-3.168000, -2.460000, 2.810000, 3.545000};
//    using p1 = std::hash<ariac_msgs::msg::Part>;
//    using p2 = std::hash<geometry_msgs::msg::Pose>;
//    using p3 = std::hash<std::vector<p2>>;
//    std::map<ariac_msgs::msg::Part, std::vector<geometry_msgs::msg::Pose>> parts_location;
    bool FloorRobotMovetoTarget();
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
    void FloorRobotWaitForAttach(double timeout, double h);
    void FloorRobotWaitForAttachConveyor(double timeout, double h, double offset);
    void FloorRobotWaitForAttachPump(double timeout, double h);
    void FloorRobotWaitForAttachBattery(double timeout, double h);

// new  
    bool CeilingRobotMovetoTarget();
    bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    void CeilingRobotWaitForAttach(double timeout);
    bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
    bool CeilingRobotMoveToAssemblyStation(int station);
    bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
    bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);
// new  

    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    void LogPose(geometry_msgs::msg::Pose p);
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    double GetYaw(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddModelsToPlanningScene();

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    
// new    // 
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;  
// new    //   
   
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Publishers
    /*!< Publisher to the topic /ariac/orders */
    rclcpp::Publisher<competitor_interfaces::msg::CompletedOrder>::SharedPtr completed_order_pub_;

    // Subscriptions
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    // ceiling robot

// new
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;
// new
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_;
    // rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr break_beam_sub_;
    rclcpp::Subscription<competitor_interfaces::msg::FloorRobotTask>::SharedPtr lowP_robot_task_sub_;
    rclcpp::Subscription<competitor_interfaces::msg::FloorRobotTask>::SharedPtr highP_robot_task_sub_;
    rclcpp::Subscription<competitor_interfaces::msg::CeilingRobotTask>::SharedPtr ceiling_robot_task_sub_;
    

// new
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;
// new
// new
    // Assembly States
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

    // Orders List
    competitor_interfaces::msg::FloorRobotTask current_order_;
    std::vector<competitor_interfaces::msg::FloorRobotTask> lowP_orders_;
    std::vector<competitor_interfaces::msg::FloorRobotTask> highP_orders_;
    unsigned int competition_state_;

    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::Part floor_robot_attached_part_;
    // neww
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_;
    geometry_msgs::msg::Pose kts2_camera_pose_;
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    geometry_msgs::msg::Pose right_bins_camera_pose_;
    geometry_msgs::msg::Pose conveyor_camera_pose_;

    // geometry_msgs::msg::Pose basic_conveyor_camera_pose_;




    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
    // std::vector<ariac_msgs::msg::PartPose> conveyor_parts_;

    std::vector<ariac_msgs::msg::PartPose> conveyor_parts_;


    // std::vector<ariac_msgs::msg::PartPose> conveyor_parts_copy;

//new vectors to copy part type, pose(y direction), distance(from time and vel=0.2m/s) from conveyor camera  parts.
    std::vector<double> offset;
    std::vector<std::chrono::steady_clock::time_point> conveyor_parts_time;


    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

    // Sensor Callbacks
    bool kts1_camera_received_data = false;
    bool kts2_camera_received_data = false;
    bool left_bins_camera_received_data = false;
    bool right_bins_camera_received_data = false;
    bool floor_robot_task_received_data_ = false;
    bool conveyor_camera_received_data = false;

    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    // void break_beam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    // Competition state callback
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    // Floor Robot Task Callback
    void lowP_robot_task_cb(const competitor_interfaces::msg::FloorRobotTask::ConstSharedPtr msg);
    void highP_robot_task_cb(const competitor_interfaces::msg::FloorRobotTask::ConstSharedPtr msg);
    // void ceiling_robot_task_cb(const competitor_interfaces::msg::CeilingRobotTask::ConstSharedPtr msg);
// new
    void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);



    // ARIAC Services
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
//ceiling
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;

    // Constants
    double kit_tray_thickness_ = 0.01;
    double drop_height_ = 0.002;
    double pick_offset_ = 0.005;
    double battery_grip_offset_ = -0.05;

    std::vector<std::string> part_types_string{"battery", "pump", "regulator", "sensor"};
    std::vector<std::string> part_colors_string{"red", "blue", "green", "orange", "purple"};

    std::vector<int> part_colors_int{ariac_msgs::msg::Part::RED, ariac_msgs::msg::Part::BLUE, ariac_msgs::msg::Part::GREEN, ariac_msgs::msg::Part::ORANGE, ariac_msgs::msg::Part::PURPLE};
    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}};

//    std::map<int, int> part_types_int = {
//        {ariac_msgs::msg::Part::BATTERY, 0},
//        {ariac_msgs::msg::Part::PUMP, 1},
//        {ariac_msgs::msg::Part::REGULATOR, 2},
//        {ariac_msgs::msg::Part::SENSOR, "sensor"}};

    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };

    // Part heights
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}};

    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };

    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};

    // Joint value targets for kitting stations
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};
        
       //joint positions for conveyor pickup zone
    std::map<std::string, double> floor_conv_js_test = {
        {"linear_actuator_joint", -2.7},
        {"floor_shoulder_pan_joint", -0.125664},
        {"floor_shoulder_lift_joint", -2.261947},
        {"floor_elbow_joint", -2.073451},
        {"floor_wrist_1_joint", -0.376991},
        {"floor_wrist_2_joint", 1.589000},
        {"floor_wrist_3_joint", 0.0}};



    
    std::map<std::string, double> floor_conv_js_right_bins_1_4 = {
        {"linear_actuator_joint", -3.168000},
        {"floor_shoulder_pan_joint", 0.000000},
        {"floor_shoulder_lift_joint", -1.256615},
        {"floor_elbow_joint", 2.136297},
        {"floor_wrist_1_joint", 0.753932},
        {"floor_wrist_2_joint", 1.759296},
        {"floor_wrist_3_joint", 1.256639}};

    std::map<std::string, double> floor_conv_js_left_bins_5_8 = {
        {"linear_actuator_joint", 3.168000},
        {"floor_shoulder_pan_joint", 0.000000},
        {"floor_shoulder_lift_joint", -1.256615},
        {"floor_elbow_joint", 2.136297},
        {"floor_wrist_1_joint", 0.753932},
        {"floor_wrist_2_joint", 1.759296},
        {"floor_wrist_3_joint", 1.256639}};


    std::map<std::string, double> middle_trash_bin = {
        {"linear_actuator_joint", 0.0},
        {"floor_shoulder_pan_joint", -0.063628},
        {"floor_shoulder_lift_joint", -1.444336},
        {"floor_elbow_joint", 1.570000},
        {"floor_wrist_1_joint", -1.756640},
        {"floor_wrist_2_joint",-1.570000},
        {"floor_wrist_3_joint", 0.0}};


    std::map<std::string, double> start_trash_bin = {
        {"linear_actuator_joint", -4.1},
        {"floor_shoulder_pan_joint", -2.451238},
        {"floor_shoulder_lift_joint", -1.444336},
        {"floor_elbow_joint", 1.570000},
        {"floor_wrist_1_joint", -1.756640},
        {"floor_wrist_2_joint",-1.570000},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> end_trash_bin = {
        {"linear_actuator_joint", 4.1},
        {"floor_shoulder_pan_joint", 2.136283},
        {"floor_shoulder_lift_joint", -1.444336},
        {"floor_elbow_joint", 1.570000},
        {"floor_wrist_1_joint", -1.756640},
        {"floor_wrist_2_joint",-1.570000},
        {"floor_wrist_3_joint", 0.0}};



    std::map<std::string, double> floor_conv_js_bin_front = {
        {"linear_actuator_joint", -2.460000},
        {"floor_shoulder_pan_joint", 3.141591},
        {"floor_shoulder_lift_joint", -2.010635},
        {"floor_elbow_joint", -2.261947},
        {"floor_wrist_1_joint", -0.450000},
        {"floor_wrist_2_joint", 1.560004},
        {"floor_wrist_3_joint", -0.000001}};

    std::map<std::string, double> ceiling_as1_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as2_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as3_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as4_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };



};

