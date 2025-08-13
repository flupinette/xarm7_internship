/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "xarm_moveit_servo/xarm_keyboard_input.h"


// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_G 0x67
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79
#define KEYCODE_H 0x68
#define KEYCODE_O 0x6F
#define KEYCODE_C 0x63
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C

KeyboardReader keyboard_reader_;


KeyboardServoPub::KeyboardServoPub(rclcpp::Node::SharedPtr& node)
: dof_(6), ros_queue_size_(10),
cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
robot_link_command_frame_("link_base"), 
ee_frame_name_("link_eef"),
planning_frame_("link_base"),
joint_vel_cmd_(1.0),
linear_pos_cmd_(0.5)
{
    node_ = node;
    // init parameter from node
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }

    // Setup pub/sub
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(joint_command_in_topic_, ros_queue_size_);
    collision_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    rail_set_pos_client_ = node_->create_client<xarm_msgs::srv::LinearTrackSetPos>("/xarm/set_linear_track_pos");
    rail_set_speed_client_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_linear_track_speed");
    rail_back_origin_client_ = node_->create_client<xarm_msgs::srv::LinearTrackBackOrigin>("/xarm/set_linear_track_back_origin");
    gripper_mode_client_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_gripper_mode");
    gripper_enable_client_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_gripper_enable");
    gripper_speed_client_ = node_->create_client<xarm_msgs::srv::SetFloat32>("/xarm/set_gripper_speed");
    gripper_set_pos_client_ = node_->create_client<xarm_msgs::srv::GripperMove>("/xarm/set_gripper_position");
    gripper_get_position_client_ = node_->create_client<xarm_msgs::srv::GetFloat32>("/xarm/get_gripper_position");

    if (!rail_set_pos_client_->wait_for_service(std::chrono::milliseconds(1000))) {
        simulation_mode_ = true;
        RCLCPP_WARN(node_->get_logger(), "Simulation detected: rail will not be controlled.");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Real mode detected: rail control enabled.");
    }

    if(!simulation_mode_) {
        // Wait for services to be available
        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_gripper_mode...");
        gripper_mode_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_gripper_enable...");
        gripper_enable_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_gripper_speed...");
        gripper_speed_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_gripper_position...");
        gripper_set_pos_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/get_gripper_position...");
        gripper_get_position_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_linear_track_pos...");
        rail_set_pos_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_linear_track_speed...");
        rail_set_speed_client_->wait_for_service();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for /xarm/set_linear_track_back_origin...");
        rail_back_origin_client_->wait_for_service();
    }

    // Create a service client to start the ServoServer
    servo_start_client_ = node_->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    
    RCLCPP_DEBUG(node_->get_logger(), "\n*********dof=%d, ros_queue_size=%d\n", dof_, ros_queue_size_);
    
    if(!simulation_mode_) {
        auto speed_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        speed_req->data = 50;
        rail_set_speed_client_->async_send_request(speed_req);
        RCLCPP_DEBUG(node_->get_logger(), "Rail speed set to 50 mm/s");

        rail_back_to_origin();

        // Set initial gripper mode
        gripper_set_mode(0);  

        // Set initial gripper enable state
        gripper_set_enable(1);  

        // Set initial gripper speed
        gripper_set_speed(1000.0);  

        // Get initial gripper position
        gripper_pos_ = gripper_get_position();
        RCLCPP_DEBUG(node_->get_logger(), "Initial gripper position: %d", gripper_pos_);
    }
}

template <typename T>
void KeyboardServoPub::_declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
{
    try
    {
        if (node_->has_parameter(param_name))
        {
            node_->get_parameter<T>(param_name, output_value);
        }
        else
        {
            output_value = node_->declare_parameter<T>(param_name, default_value);
        }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
        throw e;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Found parameter - " << param_name << ": " << output_value);
}

void KeyboardServoPub::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}



void KeyboardServoPub::move_rail_to(int pos)
{
    if (simulation_mode_) {
        return;
    }
    rail_pos_ = std::max(rail_min_, std::min(rail_max_, pos));
    auto req = std::make_shared<xarm_msgs::srv::LinearTrackSetPos::Request>();
    req->pos = rail_pos_;
    rail_set_pos_client_->async_send_request(req);
    RCLCPP_DEBUG(node_->get_logger(), "Moving rail to %d mm", rail_pos_);
}

void KeyboardServoPub::move_rail_by(int delta)
{
    if (simulation_mode_) {
        return;
    }
    move_rail_to(rail_pos_ + delta);
}

void KeyboardServoPub::rail_back_to_origin()
{
    if (simulation_mode_) { 
        return;
    }
    auto req = std::make_shared<xarm_msgs::srv::LinearTrackBackOrigin::Request>();
    rail_back_origin_client_->async_send_request(req);
    rail_pos_ = 0;
    RCLCPP_DEBUG(node_->get_logger(), "Rail returned to origin");
}

void KeyboardServoPub::gripper_set_mode(int mode){
    if (simulation_mode_) {
        return;
    }
    auto req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req->data = mode;
    gripper_mode_client_->async_send_request(req);
}

void KeyboardServoPub::gripper_set_enable(int enable){
    if (simulation_mode_) {
        return;
    }
    auto req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req->data = enable;
    gripper_enable_client_->async_send_request(req);
}

void KeyboardServoPub::gripper_set_speed(float speed){
    if (simulation_mode_) {
        return;
    }
    auto req = std::make_shared<xarm_msgs::srv::SetFloat32::Request>();
    req->data = speed;
    gripper_speed_client_->async_send_request(req);
}

void KeyboardServoPub::gripper_set_pos(int pos){
    if (simulation_mode_) {
        return;
    }
    gripper_pos_ = std::max(gripper_min_, std::min(gripper_max_, pos));
    auto req = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
    req->pos = gripper_pos_;
    gripper_set_pos_client_->async_send_request(req);
    RCLCPP_DEBUG(node_->get_logger(), "Gripper moving to position: %d", gripper_pos_);
}

int KeyboardServoPub::gripper_get_position() {
    if (simulation_mode_) {
        return gripper_pos_;  // Return current position in simulation mode
    }
    auto req = std::make_shared<xarm_msgs::srv::GetFloat32::Request>();
    auto future_result = gripper_get_position_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
        return future_result.get()->data;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get gripper position");
        return -1;
    }
}

void KeyboardServoPub::keyLoop()
{
    char c;
    bool publish_twist = false;
    bool publish_joint = false;

    std::thread{ std::bind(&KeyboardServoPub::spin, this) }.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
    puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
    puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
    if(!simulation_mode_){
        puts("Use G/H to move rail left/right, Y to go to origin.");
        puts("O: Open gripper | C: Close | K/L: fine control");
    }
    puts("'Q' to quit.");
    
    for (;;) {
        try {
            keyboard_reader_.readOne(&c);
        }
        catch (const std::runtime_error&) {
            perror("read():");
            return;
        }
        RCLCPP_DEBUG(node_->get_logger(), "value: 0x%02X", c);

        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // Use read key-press
        switch (c)
        {
        case KEYCODE_LEFT:
            RCLCPP_DEBUG(node_->get_logger(), "LEFT");
            twist_msg->twist.linear.y = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_RIGHT:
            RCLCPP_DEBUG(node_->get_logger(), "RIGHT");
            twist_msg->twist.linear.y = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_UP:
            RCLCPP_DEBUG(node_->get_logger(), "UP");
            twist_msg->twist.linear.x = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_DOWN:
            RCLCPP_DEBUG(node_->get_logger(), "DOWN");
            twist_msg->twist.linear.x = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_PERIOD:
            RCLCPP_DEBUG(node_->get_logger(), "PERIOD");
            twist_msg->twist.linear.z = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_SEMICOLON:
            RCLCPP_DEBUG(node_->get_logger(), "SEMICOLON");
            twist_msg->twist.linear.z = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_E:
            RCLCPP_DEBUG(node_->get_logger(), "E");
            planning_frame_ = ee_frame_name_;
            break;
        case KEYCODE_W:
            RCLCPP_DEBUG(node_->get_logger(), "W");
            planning_frame_ = robot_link_command_frame_;
            break;
        case KEYCODE_1:
            RCLCPP_DEBUG(node_->get_logger(), "1");
            joint_msg->joint_names.push_back("joint1");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_2:
            RCLCPP_DEBUG(node_->get_logger(), "2");
            joint_msg->joint_names.push_back("joint2");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_3:
            RCLCPP_DEBUG(node_->get_logger(), "3");
            joint_msg->joint_names.push_back("joint3");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_4:
            RCLCPP_DEBUG(node_->get_logger(), "4");
            joint_msg->joint_names.push_back("joint4");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_5:
            RCLCPP_DEBUG(node_->get_logger(), "5");
            joint_msg->joint_names.push_back("joint5");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_6:
            RCLCPP_DEBUG(node_->get_logger(), "6");
            joint_msg->joint_names.push_back("joint6");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_7:
            RCLCPP_DEBUG(node_->get_logger(), "7");
            joint_msg->joint_names.push_back("joint7");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_R:
            RCLCPP_DEBUG(node_->get_logger(), "R");
            joint_vel_cmd_ *= -1;
            break;
        case KEYCODE_Q:
            RCLCPP_DEBUG(node_->get_logger(), "quit");
            return;
        case KEYCODE_G:  // Move rail left
            move_rail_by(-rail_step_);
            break;
        case KEYCODE_H:  // Move rail right
            move_rail_by(rail_step_);
            break;
        case KEYCODE_Y:  // Go to rail origin
            move_rail_to(0);
            break;
        case KEYCODE_O:  // open gripper
            RCLCPP_DEBUG(node_->get_logger(), "Open gripper");
            gripper_set_pos(gripper_max_);
            break;
        case KEYCODE_C:  // close gripper
            RCLCPP_DEBUG(node_->get_logger(), "Close gripper");
            gripper_set_pos(gripper_min_);
            break;
        case KEYCODE_K:  // fine control - decrease position
            gripper_set_pos(gripper_pos_ - gripper_step_);
            break;
        case KEYCODE_L:  // fine control - increase position
            gripper_set_pos(gripper_pos_ + gripper_step_);
            break;
        }
        
        // If a key requiring a publish was pressed, publish the message now
        if (publish_twist)
        {
            twist_msg->header.stamp = node_->now();
            twist_msg->header.frame_id = planning_frame_;
            twist_pub_->publish(std::move(twist_msg));
            publish_twist = false;
        }
        else if (publish_joint)
        {
            joint_msg->header.stamp = node_->now();
            joint_msg->header.frame_id = "joint";
            joint_pub_->publish(std::move(joint_msg));
            publish_joint = false;
        }
    }
}

void exit_sig_handler(int sig)
{
  (void)sig;
  keyboard_reader_.shutdown();
  rclcpp::shutdown();
  exit(-1);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_moveit_servo_keyboard_node", node_options);

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    KeyboardServoPub keyboard_servo_pub(node);
    signal(SIGINT, exit_sig_handler);
    keyboard_servo_pub.keyLoop();
    keyboard_reader_.shutdown();
    
    // rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_moveit_servo_keyboard_node over");

    return 0;
}
