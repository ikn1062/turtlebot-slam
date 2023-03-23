/// \file
/// \brief The odometry node updates odomoetry between odom turtlebot
///
/// PARAMETERS:
///     rate (double): The frequency at which the main loop of the node runs
///     body_id (string): The body frame id
///     odom_id (string): The odom frame id
///     wheel_left (string): The name of the left wheel joint
///     wheel_right (string): The name of the right wheel joint
///     theta_init (double): The initial theta position of robot
///     x_init (double): The initial x position of robot
///     y_init (double): The initial y position of robot
/// PUBLISHES:
///     odom_pub_ (nav_msgs::msg::Odometry): Updates Odometry state based on current turtlebot config
///     path_trace_pub_ (nav_msgs::msg::Path: Publishes a path trace of the Blue robot over time
/// SUBSCRIBES:
///     joint_states_sub_ (sensor_msgs::msg::JointState): Receives wheel joint angle and angular velocity
/// SERVERS:
///     initial_pose_srv_ (nuturtle_control::srv::Control): Sets the initial position for the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/pose.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>


using namespace std::chrono;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Declares all node parameters for Odometry node
    declare_parameter("rate", 50.0);         // "Rate of nusim Simulation (Hz)"

    declare_parameter("body_id", "blue/base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "wheel_left_joint");
    declare_parameter("wheel_right", "wheel_right_joint");

    declare_parameter("w0", 0.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);

    // Gets parameters for the odometry node
    rate_ = get_parameter("rate").as_double();
    rate_path_ = 10.0;
    auto duration_ = std::chrono::duration<double>(1 / rate_);
    auto duration_path_ = std::chrono::duration<double>(1 / rate_path_);

    get_parameter("odom_id", odom_id);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!get_parameter("body_id", body_id)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: body_id");
      rclcpp::shutdown();
    }
    if (!get_parameter("wheel_left", wheel_left)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: wheel_left");
      rclcpp::shutdown();
    }
    if (!get_parameter("wheel_right", wheel_right)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"), "Error: Parameter not set correctly: wheel_right");
      rclcpp::shutdown();
    }

    get_parameter("w0", config.theta);
    get_parameter("x0", config.x);
    get_parameter("y0", config.y);

    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    diffDrive = turtlelib::DiffDrive(config);

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&Odometry::joint_states_cb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    initial_pose_srv_ = create_service<nuturtle_control::srv::Pose>(
      "/initial_pose", std::bind(
        &Odometry::initial_pose_cb, this, std::placeholders::_1, std::placeholders::_2));

    path_trace_pub_ = create_publisher<nav_msgs::msg::Path>("/blue_path", 10);

    timer_ = create_wall_timer(duration_, std::bind(&Odometry::odom_main, this));
    timer_path_ = create_wall_timer(duration_path_, std::bind(&Odometry::create_path, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double rate_;

  rclcpp::TimerBase::SharedPtr timer_path_;
  double rate_path_;

  // Initialize parameters for odomoetry Node
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;

  // Internal Parameters in the odometry Node
  turtlelib::DiffDrive diffDrive;

  nav_msgs::msg::Odometry odom;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_stamped;

  turtlelib::WheelAngle curr_angle;
  turtlelib::WheelAngle old_angle;
  turtlelib::WheelVelocity curr_vel;
  turtlelib::Config config;
  turtlelib::Twist2D twist;

  tf2::Quaternion q;

  // Publisher and Subscribers for the Odometry node
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<nuturtle_control::srv::Pose>::SharedPtr initial_pose_srv_;

  nav_msgs::msg::Path path_trace;
  geometry_msgs::msg::PoseStamped pose_trace;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_trace_pub_;

  void joint_states_cb(const sensor_msgs::msg::JointState & joint_state)
  {
    curr_angle.left = joint_state.position[0];
    curr_angle.right = joint_state.position[1];
    curr_vel.left = joint_state.velocity[0];
    curr_vel.right = joint_state.velocity[1];
  }

  bool initial_pose_cb(
    std::shared_ptr<nuturtle_control::srv::Pose::Request> pose,
    std::shared_ptr<nuturtle_control::srv::Pose::Response>)
  {
    config.theta = pose->theta;
    config.x = pose->x;
    config.y = pose->y;
    diffDrive = turtlelib::DiffDrive(config);
    return true;
  }

  void odom_main()
  {
    twist = diffDrive.bodyTwist(curr_vel);
    config = diffDrive.forwardKinematics(curr_angle);

    odom.header.stamp = get_clock()->now();
    odom.pose.pose.position.x = config.x;
    odom.pose.pose.position.y = config.y;
    odom.pose.pose.position.z = 0.0;

    q.setRPY(0, 0, config.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.angular.z = twist.dtheta;
    odom.twist.twist.linear.x = twist.dx;
    odom.twist.twist.linear.y = twist.dy;

    tf_stamped.header.frame_id = odom_id;
    tf_stamped.child_frame_id = body_id;
    tf_stamped.header.stamp = get_clock()->now();
    tf_stamped.transform.translation.x = config.x;
    tf_stamped.transform.translation.y = config.y;
    // tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    //create_path();
    odom_pub_->publish(odom);
    tf_broadcaster_->sendTransform(tf_stamped);
  }

  void create_path()
  {
    path_trace.header.stamp = get_clock()->now();
    path_trace.header.frame_id = odom_id;

    pose_trace.header.stamp = get_clock()->now();
    pose_trace.header.frame_id = body_id;
    pose_trace.pose.position.x = config.x;
    pose_trace.pose.position.y = config.y;
    pose_trace.pose.position.z = 0;

    std::cout << "odom x: " << config.x << ", odom y: " << config.y << ", odom rad: " <<
      config.theta << std::endl;

    tf2::Quaternion q;
    q.setRPY(0, 0, config.theta);
    pose_trace.pose.orientation.x = q.x();
    pose_trace.pose.orientation.y = q.y();
    pose_trace.pose.orientation.z = q.z();
    pose_trace.pose.orientation.w = q.w();

    path_trace.poses.push_back(pose_trace);
    path_trace_pub_->publish(path_trace);
  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating Odometry Node. ");

  rclcpp::spin(std::make_shared<Odometry>());

  rclcpp::shutdown();
  return 0;
}
