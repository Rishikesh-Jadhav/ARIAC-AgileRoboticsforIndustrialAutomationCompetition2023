#include "floor_robot.hpp"

RoboCircus::RoboCircus()
    : Node("robot_circus_node"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
    // Use upper joint velocity and acceleration limits
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);
    ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
    ceiling_robot_.setMaxVelocityScalingFactor(1.0);

//    ariac_msgs::msg::Part part_constructor;
//    part_constructor.type = ariac_msgs::msg::Part::BATTERY;
//    part_constructor.color = ariac_msgs::msg::Part::RED;
//
//    std::vector<geometry_msgs::msg::Pose> pose_constructor;
//
//    parts_location[part_constructor] = pose_constructor;

    // Subscribe to topics
    rclcpp::SubscriptionOptions options;

    topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = topic_cb_group_;

    competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 1,
        std::bind(&RoboCircus::competition_state_cb, this, std::placeholders::_1), options);

    kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RoboCircus::kts1_camera_cb, this, std::placeholders::_1), options);

    kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RoboCircus::kts2_camera_cb, this, std::placeholders::_1), options);

    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RoboCircus::left_bins_camera_cb, this, std::placeholders::_1), options);

    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RoboCircus::right_bins_camera_cb, this, std::placeholders::_1), options);

    conveyor_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/conveyor_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RoboCircus::conveyor_camera_cb, this, std::placeholders::_1), options);

    // break_beam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
    //     "/ariac/sensors/break_beam/status", rclcpp::SensorDataQoS(),
    //     std::bind(&FloorRobot::break_beam_cb, this, std::placeholders::_1), options);

    lowP_robot_task_sub_ = this->create_subscription<competitor_interfaces::msg::FloorRobotTask>(
        "/competitor/lowP_robot_task", 1,
        std::bind(&RoboCircus::lowP_robot_task_cb, this, std::placeholders::_1), options);

    highP_robot_task_sub_ = this->create_subscription<competitor_interfaces::msg::FloorRobotTask>(
        "/competitor/highP_robot_task", 1,
        std::bind(&RoboCircus::highP_robot_task_cb, this, std::placeholders::_1), options);

    // ceiling_robot_task_sub_ = this->create_subscription<competitor_interfaces::msg::CeilingRobotTask>(
    //     "/competitor/floor_robot_task", 1,
    //     std::bind(&RoboCircus::ceiling_robot_task_cb, this, std::placeholders::_1), options);
    
    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::floor_gripper_state_cb, this, std::placeholders::_1), options);

    ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

    as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::as1_state_cb, this, std::placeholders::_1), options);
    
    as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::as2_state_cb, this, std::placeholders::_1), options);

    as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::as3_state_cb, this, std::placeholders::_1), options);

    as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(), 
        std::bind(&RoboCircus::as4_state_cb, this, std::placeholders::_1), options);

    // Initialize publishers
    completed_order_pub_ = this->create_publisher<competitor_interfaces::msg::CompletedOrder>("/competitor/completed_order", 10);

    // Initialize service clients
    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
    floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");
    pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("ariac/get_pre_assembly_poses");
    AddModelsToPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

RoboCircus::~RoboCircus()
{
    floor_robot_.~MoveGroupInterface();
    ceiling_robot_.~MoveGroupInterface();

}

void RoboCircus::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
}


void RoboCircus::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts1_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_camera_received_data = true;
    }

    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void RoboCircus::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts2_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_camera_received_data = true;
    }

    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void RoboCircus::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!left_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_camera_received_data = true;
    }

    left_bins_parts_ = msg->part_poses;
    left_bins_camera_pose_ = msg->sensor_pose;

    
}

void RoboCircus::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!right_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_camera_received_data = true;
    }

    right_bins_parts_ = msg->part_poses;
    right_bins_camera_pose_ = msg->sensor_pose;
    
}

void RoboCircus::conveyor_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!conveyor_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from conveyor camera");
        conveyor_camera_received_data = true;
    }

    conveyor_camera_pose_ = msg->sensor_pose;
//    conveyor_parts_ = msg->part_poses;

    for (auto part : msg->part_poses)
    {   auto part_pose = MultiplyPose(conveyor_camera_pose_, part.pose);
        if (abs(part_pose.position.y - 3.8)<=0.01)
            {conveyor_parts_.push_back(part);
            RCLCPP_INFO(get_logger(), "========================================================================================");
            RCLCPP_INFO(get_logger(), "PART PASSING");
            RCLCPP_INFO(get_logger(), "========================================================================================");
             std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
             conveyor_parts_time.push_back(t);
            }
    }

//new
    // std::cout << typeid(conveyor_parts_).name() << std::endl;

    // for (auto part : conveyor_parts_)
    // {
    //     // Add the y coordinate of each pose to the offset vector
    //     offset.push_back(part.pose.position.x);
    // }

    // for (int i = 0; i < offset.size(); i++)
    // {
    //     RCLCPP_INFO(get_logger(), "Offset[%d]: %f", i, offset[i]);
    // }
    // RCLCPP_INFO_STREAM(get_logger(), "Offset size-------------" << offset.size());

// //new rwa3
//     for (auto part : conveyor_parts_)
//     {
//         // Add the y coordinate of each pose to the offset vector
//         conveyor_parts_copy.push_back(part.part.type);
//     }


}

// void FloorRobot::break_beam_cb(
//     const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){

// //    if (msg->object_detected)
// //    {
//         RCLCPP_INFO_STREAM(get_logger(), "Received data from Break Beam camera");
// //    }

// }

void RoboCircus::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

void RoboCircus::ceiling_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  ceiling_gripper_state_ = *msg;
}

void RoboCircus::lowP_robot_task_cb(
    const competitor_interfaces::msg::FloorRobotTask::ConstSharedPtr msg)
{
    lowP_orders_.push_back(*msg);
}

void RoboCircus::highP_robot_task_cb(
    const competitor_interfaces::msg::FloorRobotTask::ConstSharedPtr msg)
{
    highP_orders_.push_back(*msg);
}

// void RoboCircus::ceiling_robot_task_cb(
//     const competitor_interfaces::msg::CeilingRobotTask::ConstSharedPtr msg)
// {
//     orders_.push_back(*msg);
// }

void RoboCircus::as1_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void RoboCircus::as2_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void RoboCircus::as3_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}
void RoboCircus::as4_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

geometry_msgs::msg::Pose RoboCircus::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void RoboCircus::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose RoboCircus::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose RoboCircus::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double RoboCircus::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion RoboCircus::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void RoboCircus::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void RoboCircus::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

geometry_msgs::msg::Quaternion RoboCircus::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}



// FLOOR ROBOT FUNCTIONS ---------------------------------------------------------------


bool RoboCircus::FloorRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool RoboCircus::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_.execute(trajectory));
}


void RoboCircus::FloorRobotWaitForAttach(double timeout, double h = 0.001){
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= h;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.01, 0.01);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

void RoboCircus::FloorRobotWaitForAttachConveyor(double timeout, double h, double offset)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;



    int i =0;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();

        if(i==0){

		starting_pose.position.z -= h;
        starting_pose.position.x = offset;

		waypoints.push_back(starting_pose);

		FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
		i++;
	}

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }

    // waypoints.clear();
    // starting_pose = floor_robot_.getCurrentPose().pose;
    // starting_pose.position.z += 0.1;
    // waypoints.push_back(starting_pose);
    // FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

}

void RoboCircus::FloorRobotWaitForAttachPump(double timeout, double h)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;



    int i =0;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();

        if(i==0){

		starting_pose.position.z -= h;

		waypoints.push_back(starting_pose);

		FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    FloorRobotWaitForAttach(7.0,0.00005);
    i++;
	}

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }

}



void RoboCircus::FloorRobotWaitForAttachBattery(double timeout, double h)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;



    int i =0;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();

        if(i==0){

		starting_pose.position.z -= h;

		waypoints.push_back(starting_pose);

		FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    FloorRobotWaitForAttach(7.0,0.00013);
    i++;
	}

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }

}


bool RoboCircus::FloorRobotPickPartFromKitTray(int agv_num, int quadrant, ariac_msgs::msg::Part part_to_pick)
{
    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;

    //  RCLCPP_INFO(get_logger(), "Here------------------------------1");

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    double part_rotation = GetYaw(part_drop_pose);

    RCLCPP_INFO(get_logger(), "Here------------------------------2");

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[part_to_pick.type] + 0.015, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
// if (part_to_pick.type == ariac_msgs::msg::Part::BATTERY){
//   FloorRobotWaitForAttachBattery(7.0,0.01);
// }
// else{
    FloorRobotWaitForAttach(7.0);

// }

      // Move up slightly
      waypoints.clear();
      waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

      FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    return true;

}

void RoboCircus::FloorRobotSendHome()
{
    // Move floor robot to home joint state
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool RoboCircus::FloorRobotSetGripperState(bool enable)
{
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto result = floor_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

bool RoboCircus::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    // Call service to change gripper
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    if (gripper_type == "trays")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    }
    else if (gripper_type == "parts")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    auto result = floor_robot_tool_changer_->async_send_request(request);
    result.wait();
    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
        return false;
    }

    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    return true;
}

bool RoboCircus::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
    // Check if kit tray is on one of the two tables
    geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;

    // Check table 1
    for (auto tray : kts1_trays_)
    {
        if (tray.id == tray_id)
        {
            station = "kts1";
            tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
            found_tray = true;
            break;
        }
    }
    // Check table 2
    if (!found_tray)
    {
        for (auto tray : kts2_trays_)
        {
            if (tray.id == tray_id)
            {
                station = "kts2";
                tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
                found_tray = true;
                break;
            }
        }
    }
    if (!found_tray)
    {   RCLCPP_INFO_STREAM(get_logger(),"==========================Given Tray Not Found========================");
        return false;}

    double tray_rotation = GetYaw(tray_pose);

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
        floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    // Change gripper to tray gripper
    if (floor_gripper_state_.type != "tray_gripper")
    {
        FloorRobotChangeGripper(station, "trays");
    }

    // Move to tray
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0);

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    FloorRobotSetGripperState(false);

    floor_robot_.detachObject(tray_name);

    // publish to robot state
    LockAGVTray(agv_num);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}
bool RoboCircus::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick,int agv, int quadrant, bool priority = false)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;
  std::map<std::string, std::vector<geometry_msgs::msg::Pose>> parts_location;

  for (auto type: part_types_string){
    for (auto color: part_colors_string)
     {
      parts_location[type + "_" + color] = std::vector<geometry_msgs::msg::Pose>();
        }
  }

  for (auto part: left_bins_parts_) {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      parts_location[part_types_[part.part.type]+"_"+part_colors_[part.part.color]].push_back(part_pose);
      RCLCPP_INFO_STREAM(get_logger(),part_types_[part.part.type]+"_"+part_colors_[part.part.color]);}
//
  for (auto part: right_bins_parts_) {
      part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
      parts_location[part_types_[part.part.type]+"_"+part_colors_[part.part.color]].push_back(part_pose);
      RCLCPP_INFO_STREAM(get_logger(),part_types_[part.part.type]+"_"+part_colors_[part.part.color]);}
    RCLCPP_INFO(get_logger(), "Here-------------------------2");

  if (parts_location[part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]].size() == 0) {
        if (priority){
        if(parts_on_tray.find(part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]) != parts_on_tray.end()){
//        part_pose = parts_on_tray[part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]][0];
        RCLCPP_INFO_STREAM(get_logger(),"Pick Part from tray - "<<part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]<<"Agv : "<<parts_on_tray[part_types_[part_to_pick.type]+
        "_"+part_colors_[part_to_pick.color]][0]<<" Quadrant : " << parts_on_tray[part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]][1]);

          if (floor_gripper_state_.type != "part_gripper") {
    std::string station;
    if (parts_on_tray[part_types_[part_to_pick.type]+
        "_"+part_colors_[part_to_pick.color]][0] >=3) {
      station = "kts1";
    } else {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

        FloorRobotPickPartFromKitTray(parts_on_tray[part_types_[part_to_pick.type]+
        "_"+part_colors_[part_to_pick.color]][0], parts_on_tray[part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]][1], part_to_pick);
        repl_parts prt;
        prt.agv_no = parts_on_tray[part_types_[part_to_pick.type]+
        "_"+part_colors_[part_to_pick.color]][0];
        prt.part_to_pick = part_to_pick;
        prt.quadrant = parts_on_tray[part_types_[part_to_pick.type]+"_"+part_colors_[part_to_pick.color]][1];
        replaced_parts.push_back(prt);
        return true;
        }

     }
     for (auto colr: part_colors_int){
     if (parts_location[part_types_[part_to_pick.type]+"_"+part_colors_string[colr]].size() > 0)
    {//RCLCPP_INFO_STREAM(get_logger(),part_types_[part_to_pick.type]+"_"+colr);
     part_to_pick.type = part_to_pick.type;
     part_to_pick.color = part_colors_int[colr];
     break;}
     }
}

  // Check left bins
  for (auto part : left_bins_parts_)
  {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
    {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part)
  {
    for (auto part : right_bins_parts_)
    {
      if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
      {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    }
  }

RCLCPP_INFO(get_logger(), "Here-------------------------3");


  if (!found_part)
  {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = GetYaw(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station;
    if (part_pose.position.y < 0)
    {
      station = "kts1";
    }
    else
    {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  FloorRobotSetGripperState(true);

  if(part_to_pick.type == ariac_msgs::msg::Part::PUMP){
  FloorRobotWaitForAttachPump(4.0, 0.00487);
  }
  else{FloorRobotWaitForAttach(3.0);}

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  return true;
}

bool RoboCircus::FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick, double offset, double rail_position)
{     

    if (floor_gripper_state_.type != "part_gripper") {
    std::string station;
//    if (part_pose.position.y < 0) {
//      station = "kts1";
//    } else {
//      station = "kts2";
//    }
    station = "kts1";
    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }


            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::Pose starting_pose;

            if (part_to_pick.type == ariac_msgs::msg::Part::REGULATOR)
            {
                RCLCPP_INFO_STREAM(get_logger(),"============REGULATOR================");

                floor_robot_.setJointValueTarget(floor_conv_js_test); // sets the joint values to pivck up the part
                floor_robot_.setJointValueTarget("linear_actuator_joint", rail_position);

            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            FloorRobotWaitForAttachConveyor(25.0, 0.02916, offset);

            //move up slightly
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                        starting_pose.position.z + 0.3, SetRobotOrientation(0)));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

            }

            if (part_to_pick.type == ariac_msgs::msg::Part::PUMP)
            {
                RCLCPP_INFO_STREAM(get_logger(),"============PUMP===============");                
                floor_robot_.setJointValueTarget(floor_conv_js_test); // sets the joint values to pivck up the part
                floor_robot_.setJointValueTarget("linear_actuator_joint", rail_position);

            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            FloorRobotWaitForAttachConveyor(25.0, -0.02001, offset);

            //move up slightly
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                        starting_pose.position.z + 0.3, SetRobotOrientation(0)));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            }
            

            if (part_to_pick.type == ariac_msgs::msg::Part::BATTERY)
            {
                RCLCPP_INFO_STREAM(get_logger(),"============BATTERY===============");                
                floor_robot_.setJointValueTarget(floor_conv_js_test); // sets the joint values to pivck up the part
                floor_robot_.setJointValueTarget("linear_actuator_joint", rail_position);

            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true); 
            FloorRobotWaitForAttachConveyor(25.0, 0.05921, offset);

            //move up slightly
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                        starting_pose.position.z + 0.3, SetRobotOrientation(0)));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);     
            }


            if (part_to_pick.type == ariac_msgs::msg::Part::SENSOR)
            {
                RCLCPP_INFO_STREAM(get_logger(),"============SENSOR===============");                
                floor_robot_.setJointValueTarget(floor_conv_js_test); // sets the joint values to pivck up the part
                floor_robot_.setJointValueTarget("linear_actuator_joint", rail_position);

            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            FloorRobotWaitForAttachConveyor(25.0, 0.02921, offset);
            
            //move up slightly
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                        starting_pose.position.z + 0.3, SetRobotOrientation(0)));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            }

    return true;
}


int RoboCircus::get_free_slot(int bin_no){

    int free_slots[4][9] = {{0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}};

    geometry_msgs::msg::Pose part_pose;
    int bin;
    float x_offset;
    float y_offset;
    
    if (bin_no<=2)
    {for (auto part : right_bins_parts_)
    {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        
        x_offset = bin_cent[0][0] - part_pose.position.x;
        y_offset = bin_cent[0][1] - part_pose.position.y;
        if (abs(x_offset) < 0.27 and abs(y_offset) <0.27)
        { bin = 0;
          for(int i=1;i<=9;i++)
          {
          if (abs(part_pose.position.x - bin_cent[0][0] - slotPositionCordinates(i)[0]) <0.09 and abs(part_pose.position.y - bin_cent[0][1] - slotPositionCordinates(i)[1]) <0.09)
          {free_slots[bin][i-1] = -1;}
          }
        }
        
        x_offset = bin_cent[1][0] - part_pose.position.x;
        y_offset = bin_cent[1][1] - part_pose.position.y;
        if (abs(x_offset) < 0.27 and abs(y_offset) <0.27)
        { bin = 1;
          for(int i=1;i<=9;i++)
          {
          if (abs(part_pose.position.x - bin_cent[1][0] - slotPositionCordinates(i)[0]) <0.09 and abs(part_pose.position.y - bin_cent[1][1] - slotPositionCordinates(i)[1]) <0.09)
          {free_slots[bin][i-1] = -1;}
          }
        }
     }}
     
   else{
   for (auto part : left_bins_parts_)
    {
        part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
        
        x_offset = bin_cent[2][0] - part_pose.position.x;
        y_offset = bin_cent[2][1] - part_pose.position.y;
        if (abs(x_offset) < 0.27 and abs(y_offset) <0.27)
        { bin = 2;
          for(int i=1;i<=9;i++)
          {
          if (abs(part_pose.position.x - bin_cent[2][0] - slotPositionCordinates(i)[0]) <0.09 and abs(part_pose.position.y - bin_cent[2][1] - slotPositionCordinates(i)[1]) <0.09)
          {free_slots[bin][i-1] = -1;}
          }
        }
        
        x_offset = bin_cent[3][0] - part_pose.position.x;
        y_offset = bin_cent[3][1] - part_pose.position.y;
        if (abs(x_offset) < 0.27 and abs(y_offset) <0.27)
        { bin = 3;
          for(int i=1;i<=9;i++)
          {
          if (abs(part_pose.position.x - bin_cent[3][0] - slotPositionCordinates(i)[0]) <0.09 and abs(part_pose.position.y - bin_cent[3][1] - slotPositionCordinates(i)[1]) <0.09)
          {free_slots[bin][i-1] = -1;}
          }
        }
     }
    }



    for(int i=0;i<9;i++)
    {if (free_slots[bin_no-1][i]==0)
     {return i+1;}
    } 
    
    return -1;

}

std::vector<float> RoboCircus::slotPositionCordinates( int slotno){
    std::vector<float> v;

    int newSlotno = slotno - 1;

    int x = newSlotno / 3;
    x--;
    int y = newSlotno % 3;
    y--;

    v.push_back(-1*x*0.18);
    v.push_back(-1*y*0.18);
    return v;
}

bool RoboCircus::FloorRobotPlacePartonBin(int agv_no, ariac_msgs::msg::Part part_to_pick)
{   std::vector<geometry_msgs::msg::Pose> waypoints;

    float z_down = 0.15;

    if (agv_no == 1){
        int bin_no = 1;
        floor_robot_.setJointValueTarget(floor_conv_js_bin_front);
        // FloorRobotMovetoTarget();
        floor_robot_.setJointValueTarget("linear_actuator_joint", -3.168000);
        FloorRobotMovetoTarget();


        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

        int fs = get_free_slot(bin_no);
        if (fs==-1)
        {bin_no = 4;}
        std::vector<float> v = slotPositionCordinates(get_free_slot(bin_no));

 if(part_to_pick.type == ariac_msgs::msg::Part::PUMP){

        std::cout << v[0] << std::endl;
        // starting_pose.position.x += v[0];
        // starting_pose.position.y += v[1] + 0.05;
        
        // waypoints.clear();
        // waypoints.push_back(starting_pose);
        // FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
            
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1) + 0.05,
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

        FloorRobotSetGripperState(false);
    }
else{
        std::cout << v[0] << std::endl;
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1) ,
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
             FloorRobotSetGripperState(false);

    }

    }
    
    if (agv_no == 2)
    {   int bin_no = 2;
        floor_robot_.setJointValueTarget(floor_conv_js_bin_front);
        // FloorRobotMovetoTarget();
        floor_robot_.setJointValueTarget("linear_actuator_joint", -2.460000);
        FloorRobotMovetoTarget();

        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;


        int fs = get_free_slot(bin_no);
        if (fs==-1)
        {bin_no = 1;}

        std::vector<float> v = slotPositionCordinates(get_free_slot(bin_no));

    if(part_to_pick.type == ariac_msgs::msg::Part::PUMP){

        std::cout << v[0] << std::endl;
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1) + 0.05 ,
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
             FloorRobotSetGripperState(false);
    }
    else{
        std::cout << v[0] << std::endl;
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1),
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
             FloorRobotSetGripperState(false);
    }    
    }

    if (agv_no == 3)
    {   int bin_no = 3;
        floor_robot_.setJointValueTarget(floor_conv_js_bin_front);
        // FloorRobotMovetoTarget();
        floor_robot_.setJointValueTarget("linear_actuator_joint", 2.810000);
        FloorRobotMovetoTarget();


        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
        int fs = get_free_slot(bin_no);
        if (fs==-1)
        {bin_no = 2;}
        std::vector<float> v = slotPositionCordinates(get_free_slot(bin_no));

    if(part_to_pick.type == ariac_msgs::msg::Part::PUMP){

        std::cout << v[0] << std::endl;
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1) + 0.05 ,
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            FloorRobotSetGripperState(false);
    }
    else{
        std::cout << v[0] << std::endl;
            
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1),
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            FloorRobotSetGripperState(false);
    }
    }  
    if (agv_no == 4)
    {   int bin_no = 4;
        floor_robot_.setJointValueTarget(floor_conv_js_bin_front);
        // FloorRobotMovetoTarget();
        floor_robot_.setJointValueTarget("linear_actuator_joint", 3.545000);
        FloorRobotMovetoTarget();


        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
        int fs = get_free_slot(bin_no);
        if (fs==-1)
        {bin_no = 3;}
        std::vector<float> v = slotPositionCordinates(get_free_slot(bin_no));

    if(part_to_pick.type == ariac_msgs::msg::Part::PUMP){

        std::cout << v[0] << std::endl;
           
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1) + 0.05 ,
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            FloorRobotSetGripperState(false);
    }
    else{
        std::cout << v[0] << std::endl;
            
            waypoints.clear();
            starting_pose = floor_robot_.getCurrentPose().pose;
            waypoints.push_back(BuildPose(starting_pose.position.x + v.at(0), starting_pose.position.y + v.at(1),
                                        starting_pose.position.z - z_down, starting_pose.orientation));
            FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
            FloorRobotSetGripperState(false);
       
        // starting_pose.position.x += v[0];
        // starting_pose.position.y += v[1];
        // waypoints.clear();
        // waypoints.push_back(starting_pose);
        // FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
        // FloorRobotSetGripperState(false);}
    }
    }
    return true;
}

bool RoboCircus::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_ + 0.134,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    return true;
}


// FLOOR ROBOT FUNCTIONS ---------------------------------------------------------------


// CEILING ROBOT FUNCTIONS ---------------------------------------------------------------
void RoboCircus::CeilingRobotSendHome()
{
  // Move ceiling robot to home joint state
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}

bool RoboCircus::CeilingRobotSetGripperState(bool enable)
{
  if (ceiling_gripper_state_.enabled == enable) {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = ceiling_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

void RoboCircus::CeilingRobotWaitForAttach(double timeout)
{
 // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  } 
}

bool RoboCircus::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  bool assembled = false;
  while (!assembled) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type) {
      case ariac_msgs::msg::Part::BATTERY:
        assembled = assembly_station_states_[station].battery_attached;
        break;
      case ariac_msgs::msg::Part::PUMP:
        assembled = assembly_station_states_[station].pump_attached;
        break;
      case ariac_msgs::msg::Part::SENSOR:
        assembled = assembly_station_states_[station].sensor_attached;
        break;
      case ariac_msgs::msg::Part::REGULATOR:
        assembled = assembly_station_states_[station].regulator_attached;
        break;
      default:
        RCLCPP_WARN(get_logger(), "Not a valid part type");
        return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(5)){
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceiling_robot_.stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");
  
  return true;
}

bool RoboCircus::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool RoboCircus::CeilingRobotMoveCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

bool RoboCircus::CeilingRobotMoveToAssemblyStation(int station)
{
  switch (station) {
    case 1:
      ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
      break;
    case 2:
      ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
      break;
    case 3:
      ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
      break;
    case 4:
      ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  return CeilingRobotMovetoTarget();
}

bool RoboCircus::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    dx = battery_grip_offset_*cos(part_rotation);
    dy = battery_grip_offset_*sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
    part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
    part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;
  current_pose.position.z += 0.2;
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;

}

bool RoboCircus::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Check that part is attached and matches part to assemble
  if (!ceiling_gripper_state_.attached) {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }
      
  if (part.part != ceiling_robot_attached_part_){
    RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
    return false;
  }
  
  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station) {
    case 1:
      insert_frame_name = "as1_insert_frame";
      break;
    case 2:
      insert_frame_name = "as2_insert_frame";
      break;
    case 3:
      insert_frame_name = "as3_insert_frame";
      break;
    case 4:
      insert_frame_name = "as4_insert_frame";
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));    

  } else {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }
  
  // Move to approach position
  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilingRobotWaitForAssemble(station, part);

  CeilingRobotSetGripperState(false);

  std::string part_name = part_colors_[ceiling_robot_attached_part_.color] + 
    "_" + part_types_[ceiling_robot_attached_part_.type];
  ceiling_robot_.detachObject(part_name);

  // Move away slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;

  if (part.part.type == ariac_msgs::msg::Part::REGULATOR) {
    current_pose.position.x -= 0.05;
  }
  else {
    current_pose.position.z += 0.1;
  }
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);
  
  return true;

}

// CEILING ROBOT FUNCTIONS ---------------------------------------------------------------

bool RoboCircus::ChecknComplete_HP_Orders()
{
//     while(conveyor_parts_.size() == 0 || conveyor_parts_time.size() == 0){
//
//    }


    geometry_msgs::msg::Pose part_pose;
    bool flag = false;
    while(conveyor_parts_.size()>0 && conveyor_parts_time.size()>0) // How many parts are on the convery belt
    { flag =true;
    if (floor_gripper_state_.type != "part_gripper") {
    std::string station;
//    if (part_pose.position.y < 0) {
//      station = "kts1";
//    } else {
//      station = "kts2";
//    }
    station = "kts1";
    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

        auto part_start_time = conveyor_parts_time.at(0);
        auto part = conveyor_parts_.at(0); // first part is fetched and stored into variable
        part_pose = MultiplyPose(conveyor_camera_pose_, part.pose); //  part position in the world frame
        auto elapsed_time = (std::chrono::steady_clock::now() - part_start_time);


        auto part_y_loc = 3.8 - 1e-9*0.2*elapsed_time.count(); // extracting y coordinate
        RCLCPP_INFO_STREAM(get_logger(), "++++++++++++++++++++++++++++++++++++");
        RCLCPP_INFO_STREAM(get_logger(), part_y_loc);
        RCLCPP_INFO_STREAM(get_logger(), "------------------------------------");

        //auto part_start_time = std::chrono::steady_clock::now(); // variable storing time
        geometry_msgs::msg::Pose current_pose = floor_robot_.getCurrentPose().pose;
        auto d1 = part_y_loc - current_pose.position.y ;
        RCLCPP_INFO_STREAM(get_logger(), d1);
        RCLCPP_INFO_STREAM(get_logger(), "++++++++++++++++++++++++++++++++++++");
        auto t=0;
        if (d1<0){
           t = -1.0*(abs(d1) + 1.4)/0.8;}
        else { t = (abs(d1) - 1.0)/1.2;}
        if (abs(current_pose.position.y + t) >= 3.8)
          {conveyor_parts_.erase(conveyor_parts_.begin());
          conveyor_parts_time.erase(conveyor_parts_time.begin());
          RCLCPP_INFO(get_logger(), "Part has left the conveyor");
          continue;
          RCLCPP_INFO(get_logger(), "Part has been popped out");
           }
        floor_robot_.setJointValueTarget("linear_actuator_joint", -1.0*(current_pose.position.y + t));
        FloorRobotMovetoTarget();

        FloorRobotPickConveyorPart(part.part, part_pose.position.x, -1.0*(current_pose.position.y + t));
        current_pose = floor_robot_.getCurrentPose().pose;
        int bin_no;
        if (current_pose.position.y > 3.0)
        bin_no = 1;
        else if (current_pose.position.y  > 0.0 and current_pose.position.y < 3.0)
        bin_no = 2;
        else if (current_pose.position.y  < 0.0 and current_pose.position.y > -3.0)
        bin_no = 3;
        else bin_no = 4;
        FloorRobotPlacePartonBin(bin_no,part.part);
        conveyor_parts_time.erase(conveyor_parts_time.begin());
        conveyor_parts_.erase(conveyor_parts_.begin());
        RCLCPP_INFO(get_logger(), "Part has been popped out");

        floor_robot_.setNamedTarget("home");
        floor_robot_.setJointValueTarget("linear_actuator_joint", -1.0*current_pose.position.y);
        FloorRobotMovetoTarget();
    }

    // if (flag)
    // {FloorRobotSendHome();
    //   // floor_robot_.setJointValueTarget(rail_positions_["agv" + std::to_string(agv_num)]);
    //   // FloorRobotMovetoTarget();
    // flag=false;}


 if (highP_orders_.size() > 0)
    {CompleteOrders();}
    return true;

}

bool RoboCircus::CompleteOrders()
{
    // Wait for first order to be published
    while (lowP_orders_.size() == 0 && highP_orders_.size() == 0)
    {//RCLCPP_INFO(get_logger(), "Waiting for orders...");
    ChecknComplete_HP_Orders();
    }

    if (highP_orders_.size() > 0)
    {
    bool success;
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            success = false;
            break;
        }

        // complete each order from the queue
        if (highP_orders_.size() == 0)
        {
            if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
            {
                // wait for more orders
                RCLCPP_INFO(get_logger(), "Waiting for orders...");
                while (highP_orders_.size() == 0)
                {
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Completed all orders");
                success = true;
                break;
            }
        }

        current_order_ = highP_orders_.front();
        highP_orders_.erase(highP_orders_.begin());

        if (current_order_.type == ariac_msgs::msg::Order::KITTING)
        {
            RoboCircus::CompleteKittingTask(current_order_.kitting_task, current_order_.id, current_order_.priority);
        }
        else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY) 
        {
        RCLCPP_INFO(get_logger(), "INSIDE ASSEMBLY CONDITION__________________________");    
        RoboCircus::CompleteAssemblyTask(current_order_.assembly_task, current_order_.id, current_order_.priority);
        } 
        else if (current_order_.type == ariac_msgs::msg::Order::COMBINED) 
        {
        RoboCircus::CompleteCombinedTask(current_order_.combined_task, current_order_.id, current_order_.priority);
        }

//        RoboCircus::SubmitOrder(current_order_.id);
        // publish status
        // auto completed_order = competitor_interfaces::msg::CompletedOrder();
        // completed_order.order_id = current_order_.id;
        // completed_order_pub_->publish(completed_order);
        //RoboCircus::FloorRobotSendHome();
    }

    return success;
    }

    else
    {
    bool success;
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            success = false;
            break;
        }

        // complete each order from the queue
        if (lowP_orders_.size() == 0)
        {
            if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
            {
                // wait for more orders
                RCLCPP_INFO(get_logger(), "Waiting for orders...");
                while (lowP_orders_.size() == 0)
                {
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Completed all orders");
                success = true;
                break;
            }
        }

        current_order_ = lowP_orders_.front();
        lowP_orders_.erase(lowP_orders_.begin());

        if (current_order_.type == ariac_msgs::msg::Order::KITTING)
        {
            RoboCircus::CompleteKittingTask(current_order_.kitting_task, current_order_.id, current_order_.priority);
        }
        else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY)
        {
        RCLCPP_INFO(get_logger(), "INSIDE ASSEMBLY CONDITION__________________________");
        RoboCircus::CompleteAssemblyTask(current_order_.assembly_task, current_order_.id, current_order_.priority);
        }
        else if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
        {
        RoboCircus::CompleteCombinedTask(current_order_.combined_task, current_order_.id, current_order_.priority);
        }

//       RoboCircus::SubmitOrder(current_order_.id);

        // publish status
        // auto completed_order = competitor_interfaces::msg::CompletedOrder();
        // completed_order.order_id = current_order_.id;
        // completed_order_pub_->publish(completed_order);
        // RoboCircus::FloorRobotSendHome();
    }

    return success;
    }
    // return true;
}

void RoboCircus::FloorRobotMoveToTrashBin(int agv_no){
if(agv_no==1){
             floor_robot_.setJointValueTarget(start_trash_bin);
             FloorRobotMovetoTarget();    
             FloorRobotSetGripperState(false);
}
if(agv_no==2){
             floor_robot_.setJointValueTarget(middle_trash_bin);
             FloorRobotMovetoTarget();    
             FloorRobotSetGripperState(false);
}
if(agv_no==3){
             floor_robot_.setJointValueTarget(middle_trash_bin);
             FloorRobotMovetoTarget();    
             FloorRobotSetGripperState(false);
}
if(agv_no==4){
             floor_robot_.setJointValueTarget(end_trash_bin);
             FloorRobotMovetoTarget();    
             FloorRobotSetGripperState(false);
}

}

bool RoboCircus::CompleteKittingTask(ariac_msgs::msg::KittingTask task, std::string current_order_id, bool priority)
{

FloorRobotSendHome();
ChecknComplete_HP_Orders();


int id;

bool tray_available = false;

for (const auto& tray : kts1_trays_) {
  if (tray.id == task.tray_id) {
    RCLCPP_INFO(get_logger(), "TRAY AVAILABLE IN KTS1_TRAYS");
    id = task.tray_id;
    tray_available = true;
    break;
  }
}
if (!tray_available) {
  for (const auto& tray : kts2_trays_) {
    if (tray.id == task.tray_id) {
      RCLCPP_INFO(get_logger(), "TRAY AVAILABLE IN KTS2_TRAYS");
      id = task.tray_id;
      tray_available = true;
      break;
    }
  }
}
if (!tray_available) {
    if (kts2_trays_.size() != 0) {
    RCLCPP_INFO(get_logger(), "Replacement Extra TRAY AVAILABLE IN KTS2_TRAYS");
    id = kts2_trays_[0].id;
  } else if (kts1_trays_.size() != 0) {
    RCLCPP_INFO(get_logger(), "Replacement Extra TRAY AVAILABLE IN KTS1_TRAYS");
    id = kts1_trays_[0].id;
  } else {
    RCLCPP_ERROR(get_logger(), "No trays available.");
    return false;
  }
}

    FloorRobotPickandPlaceTray(id, task.agv_number);
    ChecknComplete_HP_Orders();

    ariac_msgs::msg::Part part_to_pick[4];
    bool faulty_part[4] = {false,false,false,false};

    for (auto kit_part : task.parts)
    {
        FloorRobotPickBinPart(kit_part.part, task.agv_number, kit_part.quadrant, priority);
        FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
        part_to_pick[kit_part.quadrant] = kit_part.part;

            auto request2 = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
            request2->order_id = current_order_id;
            auto result2 = quality_checker_->async_send_request(request2);
            result2.wait();
       
            std::this_thread::sleep_for(std::chrono::seconds(3));

            auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
            request->order_id = current_order_id;
            auto result = quality_checker_->async_send_request(request);
            result.wait();
   
      if (result.get()->quadrant1.faulty_part && faulty_part[kit_part.quadrant-1] == false)
      {
          RCLCPP_INFO(get_logger(), "========================Quadrant 1=============");
          int quadrant = 1;
          FloorRobotMoveToTrashBin(task.agv_number);
          FloorRobotPickBinPart(part_to_pick[quadrant], task.agv_number, kit_part.quadrant, priority);
          FloorRobotPlacePartOnKitTray(task.agv_number, quadrant);
          faulty_part[quadrant-1] = true;

      }
      if (result.get()->quadrant2.faulty_part && faulty_part[kit_part.quadrant-1] == false)
      {
          RCLCPP_INFO(get_logger(), "========================Quadrant 2=============");
          int quadrant = 2;
          FloorRobotMoveToTrashBin(task.agv_number);
          FloorRobotPickBinPart(part_to_pick[quadrant], task.agv_number, kit_part.quadrant, priority);
          FloorRobotPlacePartOnKitTray(task.agv_number, quadrant);
          faulty_part[quadrant-1] = true;
      }
      if (result.get()->quadrant3.faulty_part && faulty_part[kit_part.quadrant-1] == false)
      {
          RCLCPP_INFO(get_logger(), "========================Quadrant 3=============");
          int quadrant = 3;
          FloorRobotMoveToTrashBin(task.agv_number);
          FloorRobotPickBinPart(part_to_pick[quadrant], task.agv_number, kit_part.quadrant, priority);
          FloorRobotPlacePartOnKitTray(task.agv_number, quadrant);
          faulty_part[quadrant-1] = true;
      }
       if(result.get()->quadrant4.faulty_part && faulty_part[kit_part.quadrant-1] == false)
      {
          RCLCPP_INFO(get_logger(), "========================Quadrant 4=============");
          int quadrant = 4;
          FloorRobotMoveToTrashBin(task.agv_number);    
          FloorRobotPickBinPart(part_to_pick[quadrant], task.agv_number, kit_part.quadrant, priority);
          FloorRobotPlacePartOnKitTray(task.agv_number, quadrant);
          faulty_part[quadrant-1] = true;
      }
      // Drop part in trash
      FloorRobotSetGripperState(false);

     geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

     std::vector<geometry_msgs::msg::Pose> waypoints;
     waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                   starting_pose.position.z + 0.3,
                                   SetRobotOrientation(0)));

     FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

     std::vector<int> agv_quad{task.agv_number, kit_part.quadrant};
     parts_on_tray[part_types_[kit_part.part.type]+"_"+part_colors_[kit_part.part.color]] = agv_quad;

     ChecknComplete_HP_Orders();   
    }

    if (!priority)
    {for (auto prt: replaced_parts)
    {FloorRobotPickBinPart(prt.part_to_pick, prt.agv_no, prt.quadrant);
    FloorRobotPlacePartOnKitTray(prt.agv_no, prt.quadrant);

    FloorRobotSetGripperState(false);

    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                   starting_pose.position.z + 0.3,
                                   SetRobotOrientation(0)));

     FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

     ChecknComplete_HP_Orders();
    }

    replaced_parts.clear();}

    ChecknComplete_HP_Orders();
    MoveAGV(task.agv_number, task.destination);

    parts_on_tray.clear();
    RoboCircus::SubmitOrder(current_order_id);

    return true;
}



bool RoboCircus::CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task, std::string current_order_id, bool priority)
{


  RCLCPP_INFO(get_logger(), "INSIDE CompleteAssemblyTask________________________________");
  // Send AGVs to assembly station
  for (auto const &agv : task.agv_numbers){
    int destination;
    if (task.station == ariac_msgs::msg::AssemblyTask::AS1 || task.station == ariac_msgs::msg::AssemblyTask::AS3) {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
    } else if (task.station == ariac_msgs::msg::AssemblyTask::AS2 || task.station == ariac_msgs::msg::AssemblyTask::AS4) {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
    }

    LockAGVTray(agv);
    MoveAGV(agv, destination);
    ChecknComplete_HP_Orders();
  }
  
  CeilingRobotMoveToAssemblyStation(task.station);
  ChecknComplete_HP_Orders();

  // Get Assembly Poses
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_id;
  auto result = pre_assembly_poses_getter_->async_send_request(request);
  
  result.wait();

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
  if (result.get()->valid_id) {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0) {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }
  
  for (auto const &part_to_assemble : task.parts) {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part: agv_part_poses) {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color) {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists) {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << 
        " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);
    
    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
    ChecknComplete_HP_Orders();
  }
  RoboCircus::SubmitOrder(current_order_id);
  return true;
}


bool RoboCircus::CompleteCombinedTask(ariac_msgs::msg::CombinedTask task, std::string current_order_id, bool priority)
{
// Decide on a tray to use
int id = 0;
bool tray_available = false;

for (const auto& tray : kts1_trays_) {
  if (tray.id == id) {

    RCLCPP_INFO(get_logger(), "COMBINED TASK TRAY 0 AVAILABLE IN KTS1_TRAYS");
    tray_available = true;
    break;
  }
}
if (!tray_available) {
  for (const auto& tray : kts2_trays_) {
    if (tray.id == id) {
    RCLCPP_INFO(get_logger(), "COMBINED TASK TRAY 0 AVAILABLE IN KTS2_TRAYS");
    tray_available = true;
    break;
    
    }
  }
}
if (!tray_available) {
    if (kts2_trays_.size() != 0) {
    RCLCPP_INFO(get_logger(), "Replacement Extra TRAY AVAILABLE IN KTS2_TRAYS");
    id = kts2_trays_[0].id;
  } else if (kts1_trays_.size() != 0) {
    RCLCPP_INFO(get_logger(), "Replacement Extra TRAY AVAILABLE IN KTS1_TRAYS");
    id = kts1_trays_[0].id;
  } else {
    RCLCPP_ERROR(get_logger(), "No trays available.");
    return false;
  }
}
  // Decide which AGV to use
  int agv_number;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2) {
    agv_number = 1;
  } else {
    agv_number = 4;
  }
  ChecknComplete_HP_Orders();
  MoveAGV(agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING);
  ChecknComplete_HP_Orders();

  FloorRobotPickandPlaceTray(id, agv_number);
  ChecknComplete_HP_Orders();

  int count = 1;
//  for (auto assembly_part: task.parts) {
//    FloorRobotPickBinPart(assembly_part.part);
//    FloorRobotPlacePartOnKitTray(agv_number, count);
//    count++;
//    ChecknComplete_HP_Orders();
  ariac_msgs::msg::Part part_to_pick[4];
  for (auto kit_part : task.parts)
    {
        FloorRobotPickBinPart(kit_part.part, agv_number, count, priority);
        FloorRobotPlacePartOnKitTray(agv_number, count);
        part_to_pick[count] = kit_part.part;
        FloorRobotSetGripperState(false);
        std::vector<int> agv_quad{agv_number, count};
        count++;
        parts_on_tray[part_types_[kit_part.part.type]+"_"+part_colors_[kit_part.part.color]] = agv_quad;

        ChecknComplete_HP_Orders();
      }

  if (!priority)
    {for (auto prt: replaced_parts)
    {FloorRobotPickBinPart(prt.part_to_pick, prt.agv_no, prt.quadrant);
    FloorRobotPlacePartOnKitTray(prt.agv_no, prt.quadrant);

    FloorRobotSetGripperState(false);

    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(starting_pose.position.x, starting_pose.position.y,
                                   starting_pose.position.z + 0.3,
                                   SetRobotOrientation(0)));

     FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

     ChecknComplete_HP_Orders();
    }

    replaced_parts.clear();}
  

  int destination;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS3) {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
  } else {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
  }

  MoveAGV(agv_number, destination);
  parts_on_tray.clear();
  ChecknComplete_HP_Orders();

  CeilingRobotMoveToAssemblyStation(task.station);
  ChecknComplete_HP_Orders();

  // Get Assembly Poses
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_id;
  auto result = pre_assembly_poses_getter_->async_send_request(request);
  
  result.wait();

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
  if (result.get()->valid_id) {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0) {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts) {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part: agv_part_poses) {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color) {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists) {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << 
        " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);
    
    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
    ChecknComplete_HP_Orders();
  }

  CeilingRobotSendHome();
  FloorRobotSendHome();
  RoboCircus::SubmitOrder(current_order_id);

  return true;
}


bool RoboCircus::MoveAGV(int agv_num, int destination)
{
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool RoboCircus::LockAGVTray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}



bool RoboCircus::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool RoboCircus::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

// ================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robo_circus = std::make_shared<RoboCircus>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robo_circus);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    robo_circus->FloorRobotSendHome();
    // robo_circus->CeilingRobotSendHome();
    robo_circus->CompleteOrders();
    robo_circus->EndCompetition();
    rclcpp::shutdown();
}
