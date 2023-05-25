#include "ceiling_robot.hpp"

CeilingRobot::CeilingRobot()
    : Node("ceiling_robot_node"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
    // Use upper joint velocity and acceleration limits
    ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
    ceiling_robot_.setMaxVelocityScalingFactor(1.0);

    // Subscribe to topics
    rclcpp::SubscriptionOptions options;

    topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = topic_cb_group_;


    competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 1,
        std::bind(&CeilingRobot::competition_state_cb, this, std::placeholders::_1), options);

    ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&CeilingRobot::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

    ceiling_robot_task_sub_ = this->create_subscription<competitor_interfaces::msg::CeilingRobotTask>(
        "/competitor/ceiling_robot_task", 1,
        std::bind(&CeilingRobot::ceiling_robot_task_cb, this, std::placeholders::_1), options);


    // Initialize publishers
    completed_order_pub_ = this->create_publisher<competitor_interfaces::msg::CompletedOrder>("/competitor/completed_order", 10);

    // Initialize service clients
    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");
    pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");

    AddModelsToPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

CeilingRobot::~CeilingRobot()
{
    ceiling_robot_.~MoveGroupInterface();
}

void CeilingRobot::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
}



void CeilingRobot::ceiling_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    ceiling_gripper_state_ = *msg;
}

void CeilingRobot::ceiling_robot_task_cb(
    const competitor_interfaces::msg::CeilingRobotTask::ConstSharedPtr msg)
{
    orders_.push_back(*msg);
}

geometry_msgs::msg::Pose CeilingRobot::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void CeilingRobot::LogPose(geometry_msgs::msg::Pose p)
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

geometry_msgs::msg::Pose CeilingRobot::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose CeilingRobot::FrameWorldPose(std::string frame_id)
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

double CeilingRobot::GetYaw(geometry_msgs::msg::Pose pose)
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

geometry_msgs::msg::Quaternion CeilingRobot::QuaternionFromRPY(double r, double p, double y)
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

void CeilingRobot::AddModelToPlanningScene(
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

void CeilingRobot::AddModelsToPlanningScene()
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

geometry_msgs::msg::Quaternion CeilingRobot::SetRobotOrientation(double rotation)
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




void CeilingRobot::CeilingRobotSendHome()
{
    // Move floor robot to home joint state
    ceiling_robot_.setNamedTarget("home");
    CeilingRobotMovetoTarget();
}


bool CeilingRobot::CeilingRobotSetGripperState(bool enable)
{
    if (ceiling_gripper_state_.enabled == enable)
    {
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

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

bool CeilingRobot::CeilingRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(ceiling_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(ceiling_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool CeilingRobot::CeilingRobotMoveCartesian(
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


void CeilingRobot::CeilingRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

    while (!ceiling_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        CeilingRobotMoveCartesian(waypoints, 0.1, 0.1,false);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

bool CeilingRobot::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part){

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


bool CeilingRobot::CeilingRobotMoveToAssemblyStation(int station){

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

bool CeilingRobot::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part){
  
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


bool CeilingRobot::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part){

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




// bool CeilingRobot::CeilingRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
// {
//     RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);

//     // Check if part is in one of the bins
//     geometry_msgs::msg::Pose part_pose;
//     bool found_part = false;
//     std::string bin_side;

//     // Check left bins
//     for (auto part : left_bins_parts_)
//     {
//         if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//         {
//             part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
//             found_part = true;
//             bin_side = "left_bins";
//             break;
//         }
//     }
//     // Check right bins
//     if (!found_part)
//     {
//         for (auto part : right_bins_parts_)
//         {
//             if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//             {
//                 part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
//                 found_part = true;
//                 bin_side = "right_bins";
//                 break;
//             }
//         }
//     }
//     if (!found_part)
//     {
//         RCLCPP_ERROR(get_logger(), "Unable to locate part");
//         return false;
//     }

//     double part_rotation = GetYaw(part_pose);

//     // Change gripper at location closest to part
//     if (floor_gripper_state_.type != "part_gripper")
//     {
//         std::string station;
//         if (part_pose.position.y < 0)
//         {
//             station = "kts1";
//         }
//         else
//         {
//             station = "kts2";
//         }

//         // Move floor robot to the corresponding kit tray table
//         if (station == "kts1")
//         {
//             ceiling_robot_.setJointValueTarget(floor_kts1_js_);
//         }
//         else
//         {
//             ceiling_robot_.setJointValueTarget(floor_kts2_js_);
//         }
//         CeilingRobotMovetoTarget();

//         CeilingRobotChangeGripper(station, "parts");
//     }

//     ceiling_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
//     ceiling_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
//     CeilingRobotMovetoTarget();

//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

//     CeilingRobotMoveCartesian(waypoints, 0.3, 0.3);

//     CeilingRobotSetGripperState(true);

//     CeilingRobotWaitForAttach(3.0);

//     // Add part to planning scene
//     std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
//     AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
//     ceiling_robot_.attachObject(part_name);
//     ceiling_robot_attached_part_ = part_to_pick;

//     // Move up slightly
//     waypoints.clear();
//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.3, SetRobotOrientation(0)));

//     CeilingRobotMoveCartesian(waypoints, 0.3, 0.3);

//     return true;
// }

bool CeilingRobot::CompleteOrders()
{
    // Wait for first order to be published
    while (orders_.size() == 0)
    {
    }
    
    RCLCPP_INFO(get_logger(), "Orders Recieved------------------------------------------------------------");

    bool success_ceiling_robot;
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            success_ceiling_robot = false;
            break;
        }

        // complete each order from the queue
        if (orders_.size() == 0)
        {
            if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
            {
                // wait for more orders
                RCLCPP_INFO(get_logger(), "Waiting for orders...");
                while (orders_.size() == 0)
                {
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Completed all orders");
                success_ceiling_robot = true;
                break;
            }
        }

        current_order_ = orders_.front();
        orders_.erase(orders_.begin());

        if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY)
        {
            CeilingRobot::CompleteAssemblyTask(current_order_.assembly_task);
        }

        // CeilingRobot::SubmitOrder(current_order_.id);

        // publish status
        auto completed_order = competitor_interfaces::msg::CompletedOrder();
        completed_order.order_id = current_order_.id;
        completed_order_pub_->publish(completed_order);
        CeilingRobot::CeilingRobotSendHome();
    }

    return success_ceiling_robot;
    // return true;
}

bool CeilingRobot::SubmitOrder(std::string order_id)
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

bool CeilingRobot::CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task)
{
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
    }

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
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

  }

  return true;
}

bool CeilingRobot::MoveAGV(int agv_num, int destination)
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

bool CeilingRobot::LockAGVTray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}


// ================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto ceiling_robot = std::make_shared<CeilingRobot>();
    rclcpp::executors::MultiThreadedExecutor executor1;
    executor1.add_node(ceiling_robot);
    std::thread([&executor1]()
                { executor1.spin(); }).detach();
    ceiling_robot->CompleteOrders();

    rclcpp::shutdown();
}
