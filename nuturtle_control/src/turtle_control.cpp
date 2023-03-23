/// \file
/// \brief Launches interface to control the turtlebot3 robot through command velocities
///
/// PARAMETERS:
///     rate (double): The frequency at which the main loop of the node runs
///     wheel_radius (double): Radius of Turtlebot3 Wheel
///     track_width (double): Track Width from Turtlebot3 robot
///     motor_cmd_max (double): Maximum / Minimum values for motor commands
///     motor_cmd_per_rad_sec (double): Motor Commands per Radians
///     encoder_ticks_per_rad (double): Radians per encoder tick
///     collision_radius (double): Radius of Collision for robot
/// PUBLISHES:
///     wheelcmd_pub_ (nuturtlebot_msgs::msg::WheelCommands): Publishes wheel commands to turtlebot3
///     joint_states_pub_ (sensor_msgs::msg::JointState): Sends wheel joint angle and angular velocity
/// SUBSCRIBES:
///     cmdvel_sub_ (geometry_msgs::msg::Twist): Receives velocity commands to move the Turtlebot
///     sensor_data_sub_ (<nuturtlebot_msgs::msg::SensorData): Receives encoder values


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>


using namespace std::chrono;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Declares all node parameters from diff_params
    declare_parameter("rate", 50.0);         // "Rate of nusim Simulation (Hz)"

    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);
    declare_parameter("motor_cmd_max", 265.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 4096);
    declare_parameter("collision_radius", 0.11);

    // Checks if all parameters are loaded correctly
    rate_ = get_parameter("rate").as_double();
    auto duration_ = std::chrono::duration<double>(1 / rate_);

    if (!get_parameter("wheel_radius", wheel_radius)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: wheel_radius");
      rclcpp::shutdown();
    }
    if (!get_parameter("track_width", track_width)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: track_width");
      rclcpp::shutdown();
    }
    if (!get_parameter("motor_cmd_max", motor_cmd_max)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: motor_cmd_max");
      rclcpp::shutdown();
    }
    if (!get_parameter("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: motor_cmd_per_rad_sec");
      rclcpp::shutdown();
    }
    if (!get_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: encoder_ticks_per_rad");
      rclcpp::shutdown();
    }
    if (!get_parameter("collision_radius", collision_radius)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: collision_radius");
      rclcpp::shutdown();
    }

    // Creates the Diff Drive Class using the basic constructor
    diffDrive = turtlelib::DiffDrive();

    // Initializes the publishers and subscribers
    cmdvel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TurtleControl::cmd_vel_cb, this, std::placeholders::_1));
    wheelcmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "/sensor_data",
      10,
      std::bind(&TurtleControl::sensor_data_cb, this, std::placeholders::_1));
    joint_states_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_state_msg.header.stamp = rclcpp::Clock{}.now();
    joint_state_msg.name = {left_joint, right_joint};
    joint_state_msg.position = {0.0, 0.0};
    joint_state_msg.velocity = {0.0, 0.0};

    // Creates the main loop
    timer_ = create_wall_timer(duration_, std::bind(&TurtleControl::turtlectrl_main, this));
  }

private:
  // Rate and timer
  rclcpp::TimerBase::SharedPtr timer_;
  double rate_;

  // Diff Drive Class
  turtlelib::DiffDrive diffDrive;

  // Diff Params: Double Value
  double wheel_radius;
  double track_width;
  double motor_cmd_max;
  double motor_cmd_per_rad_sec;
  int encoder_ticks_per_rad;
  double collision_radius;

  // Creating Publishers and Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcmd_pub_;

  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Initializing Parameters for publisher and subscribers
  turtlelib::Twist2D input_tw;
  turtlelib::WheelVelocity wheel_vel;
  turtlelib::WheelAngle wheel_ang;
  double left_tick, right_tick;

  std::string left_joint = "wheel_left_joint";
  std::string right_joint = "wheel_right_joint";

  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
  sensor_msgs::msg::JointState joint_state_msg;

  void turtlectrl_main()
  {
    joint_state_msg.header.stamp = get_clock()->now();
    joint_state_msg.name = {left_joint, right_joint};
    joint_state_msg.position = {wheel_ang.left, wheel_ang.right};
    joint_state_msg.velocity = {wheel_vel.left, wheel_vel.right};

    joint_states_pub_->publish(joint_state_msg);
    wheelcmd_pub_->publish(wheel_cmd_msg);
  }

  void cmd_vel_cb(const geometry_msgs::msg::Twist & twist)
  {
    input_tw.dtheta = twist.angular.z;
    input_tw.dx = twist.linear.x;
    input_tw.dy = twist.linear.y;
    wheel_vel = diffDrive.inverseKinematics(input_tw);

    wheel_cmd_msg.left_velocity = (int) (wheel_vel.left / motor_cmd_per_rad_sec);
    if (wheel_cmd_msg.left_velocity > motor_cmd_max) {
      wheel_cmd_msg.left_velocity = motor_cmd_max;
    } else if (wheel_cmd_msg.left_velocity < -1 * motor_cmd_max) {
      wheel_cmd_msg.left_velocity = -1 * motor_cmd_max;
    }

    wheel_cmd_msg.right_velocity = (int) (wheel_vel.right / motor_cmd_per_rad_sec);
    if (wheel_cmd_msg.right_velocity > motor_cmd_max) {
      wheel_cmd_msg.right_velocity = motor_cmd_max;
    } else if (wheel_cmd_msg.right_velocity < -1 * motor_cmd_max) {
      wheel_cmd_msg.right_velocity = -1 * motor_cmd_max;
    }
  }

  void sensor_data_cb(const nuturtlebot_msgs::msg::SensorData & sensor_data)
  {
    left_tick = sensor_data.left_encoder;
    right_tick = sensor_data.right_encoder;
    wheel_ang.left = left_tick / encoder_ticks_per_rad;
    wheel_ang.right = right_tick / encoder_ticks_per_rad;
  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating Turtle Control Node. ");

  rclcpp::spin(std::make_shared<TurtleControl>());

  rclcpp::shutdown();
  return 0;
}
