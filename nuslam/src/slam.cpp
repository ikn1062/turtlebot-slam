/// \file
/// \brief The SLAM node applies an EKF filter to the robot using sensor measurements and obstacles
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
///     max_range (double): Max range of obstacle detection
///     obstacle_name (double): Where obstacles are coming from
/// PUBLISHES:
///     odom_pub_ (nav_msgs::msg::Odometry): Updates Odometry state based on current turtlebot config
///     path_trace_pub_ (nav_msgs::msg::Path: Publishes a path trace of the Blue robot over time
///     slam_marker_pub_ (visualization_msgs::msg::MarkerArray): Publishes estimates of obstacles from SLAM
///     g_path_trace_pub_ (nav_msgs::msg::Path: Publishes a path trace of the Green robot over time
/// SUBSCRIBES:
///     joint_states_sub_ (sensor_msgs::msg::JointState): Receives wheel joint angle and angular velocity
///     obstacles_sub_ (visualization_msgs::msg::MarkerArray): Receives obstacle location data from sensor
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
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"


#include <armadillo>
#include <nuslam/ekf_slam.hpp>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>


using namespace std::chrono;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // Declares all node parameters for Slam node
    declare_parameter("rate", 50.0);         // "Rate of nusim Simulation (Hz)"

    declare_parameter("body_id", "green/base_footprint");
    declare_parameter("odom_id", "green/odom");
    declare_parameter("wheel_left", "wheel_left_joint");
    declare_parameter("wheel_right", "wheel_right_joint");

    declare_parameter("w0", 0.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);

    declare_parameter("max_range", 0.5);
    declare_parameter("obstacle_name", "landmarks/obstacles");

    // Gets parameters for the Slam node
    rate_ = get_parameter("rate").as_double();
    rate_path_ = 10.0;
    auto duration_ = std::chrono::duration<double>(1 / rate_);
    auto duration_path_ = std::chrono::duration<double>(1 / rate_path_);

    get_parameter("odom_id", odom_id);

    tf_MO_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_MG_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
    max_range = get_parameter("max_range").as_double();
    obstacle_name = get_parameter("obstacle_name").as_string();

    EKF_INIT = false;
    visited_obs = {};
    state_matrix = arma::mat(100, 100, arma::fill::zeros);

    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    diffDrive = turtlelib::DiffDrive(config);

    // Initialize Transforms
    tf_MO_stamped.header.frame_id = "green/map";
    tf_MO_stamped.child_frame_id = odom_id;
    tf_MO_stamped.header.stamp = get_clock()->now();
    tf_MO_stamped.transform.translation.x = 0.0;
    tf_MO_stamped.transform.translation.y = 0.0;
    tf_MO_stamped.transform.translation.z = 0.0;
    q_MG.setRPY(0.0, 0.0, 0.0);
    tf_MO_stamped.transform.rotation.x = q.x();
    tf_MO_stamped.transform.rotation.y = q.y();
    tf_MO_stamped.transform.rotation.z = q.z();
    tf_MO_stamped.transform.rotation.w = q.w();

    tf_MG_stamped.header.frame_id = odom_id;
    tf_MG_stamped.child_frame_id = body_id;
    tf_MG_stamped.header.stamp = get_clock()->now();
    tf_MG_stamped.transform.translation.x = 0.0;
    tf_MG_stamped.transform.translation.y = 0.0;
    tf_MG_stamped.transform.translation.z = 0.0;
    q_MG.setRPY(0.0, 0.0, 0.0);
    tf_MG_stamped.transform.rotation.x = q.x();
    tf_MG_stamped.transform.rotation.y = q.y();
    tf_MG_stamped.transform.rotation.z = q.z();
    tf_MG_stamped.transform.rotation.w = q.w();

    obstacles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      obstacle_name, 10, std::bind(
        &Slam::obstacles_cb, this,
        std::placeholders::_1));
    slam_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/slam_marker", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &Slam::joint_states_cb, this,
        std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/green_odom", 10);
    initial_pose_srv_ =
      create_service<nuturtle_control::srv::Pose>(
      "/g_initial_pose",
      std::bind(&Slam::initial_pose_cb, this, std::placeholders::_1, std::placeholders::_2));

    g_path_trace_pub_ = create_publisher<nav_msgs::msg::Path>("/green_path", 10);

    timer_ = create_wall_timer(duration_, std::bind(&Slam::odom_main, this));
    timer_path_ = create_wall_timer(duration_path_, std::bind(&Slam::create_path_g, this));
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
  std::string obstacle_name;

  // Internal Parameters in the odometry Node
  turtlelib::DiffDrive diffDrive;

  nav_msgs::msg::Odometry odom;

  turtlelib::WheelAngle curr_angle;
  turtlelib::WheelAngle old_angle;
  turtlelib::WheelVelocity curr_vel;
  turtlelib::Config config;
  turtlelib::Twist2D twist;

  turtlelib::Transform2D T_odom_robot;
  turtlelib::Transform2D T_map_robot;
  turtlelib::Transform2D T_map_odom;

  tf2::Quaternion q;
  tf2::Quaternion q_MO;
  tf2::Quaternion q_MG;

  // Transform Publishers
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_MO_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_MO_stamped;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_MG_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_MG_stamped;

  // EKF SLAM objects
  turtlelib::Config green_config;
  bool EKF_INIT;
  double max_range;
  int num_obstacles;
  std::set<int> visited_obs;
  ekf_slam::ExtendedKalmanFilter EKF_slam;
  arma::mat state_matrix;
  visualization_msgs::msg::MarkerArray slam_obstacle_arr;

  // Publisher and Subscribers for the Odometry node
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<nuturtle_control::srv::Pose>::SharedPtr initial_pose_srv_;

  nav_msgs::msg::Path g_path_trace;
  geometry_msgs::msg::PoseStamped g_pose_trace;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr g_path_trace_pub_;

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

  void obstacles_cb(const visualization_msgs::msg::MarkerArray & fake_obstacles)
  {
    turtlelib::Vector2D p_obs;
    std::vector<int> vec_check(100, 0);
    num_obstacles = (int) fake_obstacles.markers.size();

    // Initialize EKF if not initialized
    if (!EKF_INIT) {
      EKF_slam = ekf_slam::ExtendedKalmanFilter(green_config, 20, 1, 0.1);
      EKF_INIT = true;
    }

    EKF_slam.predict_step(twist);

    for (int id = 0; id < num_obstacles; id++) {
      // Check if the marker is set to delelte, if it is -> skip
      if (fake_obstacles.markers[id].action == 2) {
        continue;
      }

      p_obs.x = fake_obstacles.markers[id].pose.position.x;
      p_obs.y = fake_obstacles.markers[id].pose.position.y;

      // Check if the obstacle has been initialized - not needed for data association
      if (obstacle_name == "nusim_node/fake_sensor") {
        if (!visited_obs.count(id)) {
          EKF_slam.init_landmark(id, p_obs);
          visited_obs.insert(id);
        }
        state_matrix = EKF_slam.update_step(id, p_obs);
      } else {
        //std::cout << "id: " << id << std::endl;
        //std::cout << "pos: " << p_obs << std::endl;
        int landmark_idx = EKF_slam.dataAssociation(p_obs);
        if (vec_check[landmark_idx] == 0) {
          vec_check[landmark_idx] = 1;
          state_matrix = EKF_slam.update_step(landmark_idx, p_obs);
        }
      }
    }

    green_config.theta = turtlelib::normalize_angle(state_matrix(0, 0));
    green_config.x = state_matrix(1, 0);
    green_config.y = state_matrix(2, 0);

    create_slam_obstacles();
    slam_marker_pub_->publish(slam_obstacle_arr);

    T_odom_robot = turtlelib::Transform2D({config.x, config.y}, config.theta);
    T_map_robot = turtlelib::Transform2D({green_config.x, green_config.y}, green_config.theta);
    T_map_odom = T_map_robot * (T_odom_robot.inv());

    //std::cout << "T_map_odom: " << T_map_odom << std::endl;
    //std::cout << "T_map_odom norm ang: " << turtlelib::normalize_angle(T_map_odom.rotation()) << std::endl;

    tf_MO_stamped.header.frame_id = "green/map";
    tf_MO_stamped.child_frame_id = odom_id;
    tf_MO_stamped.header.stamp = get_clock()->now();
    tf_MO_stamped.transform.translation.x = T_map_odom.translation().x;
    tf_MO_stamped.transform.translation.y = T_map_odom.translation().y;
    tf_MO_stamped.transform.translation.z = 0.0;
    q_MO.setRPY(0, 0, turtlelib::normalize_angle(T_map_odom.rotation()));
    tf_MO_stamped.transform.rotation.x = q_MO.x();
    tf_MO_stamped.transform.rotation.y = q_MO.y();
    tf_MO_stamped.transform.rotation.z = q_MO.z();
    tf_MO_stamped.transform.rotation.w = q_MO.w();

    tf_MO_broadcaster_->sendTransform(tf_MO_stamped);
  }

  void create_slam_obstacles()
  {
    turtlelib::Vector2D p_obs;
    slam_obstacle_arr.markers.resize(num_obstacles);

    for (int i = 0; i < num_obstacles; i++) {
      p_obs.x = state_matrix(3 + 2 * i, 0);
      p_obs.y = state_matrix(4 + 2 * i, 0);

      double dist_ro = sqrt(pow(config.x - p_obs.x, 2) + pow(config.y - p_obs.y, 2));
      if (dist_ro > max_range || !(visited_obs.count(i))) {
        slam_obstacle_arr.markers[i].action = visualization_msgs::msg::Marker::ADD;
      } else {
        slam_obstacle_arr.markers[i].action = visualization_msgs::msg::Marker::ADD;
      }

      slam_obstacle_arr.markers[i].header.stamp = get_clock()->now();
      slam_obstacle_arr.markers[i].header.frame_id = "nusim/world";
      slam_obstacle_arr.markers[i].ns = "slam_obstacles";

      slam_obstacle_arr.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;

      slam_obstacle_arr.markers[i].id = i;
      slam_obstacle_arr.markers[i].pose.position.x = p_obs.x;
      slam_obstacle_arr.markers[i].pose.position.y = p_obs.y;
      slam_obstacle_arr.markers[i].pose.position.z = .125;

      slam_obstacle_arr.markers[i].pose.orientation.x = 0.0;
      slam_obstacle_arr.markers[i].pose.orientation.y = 0.0;
      slam_obstacle_arr.markers[i].pose.orientation.z = 0.0;
      slam_obstacle_arr.markers[i].pose.orientation.w = 1.0;

      slam_obstacle_arr.markers[i].scale.x = 2 * 0.038;
      slam_obstacle_arr.markers[i].scale.y = 2 * 0.038;
      slam_obstacle_arr.markers[i].scale.z = .25;

      slam_obstacle_arr.markers[i].color.r = 0.0;
      slam_obstacle_arr.markers[i].color.g = 1.0;
      slam_obstacle_arr.markers[i].color.b = 0.0;
      slam_obstacle_arr.markers[i].color.a = 1.0;
    }
  }

  void odom_main()
  {
    twist = diffDrive.bodyTwist(curr_vel);
    config = diffDrive.forwardKinematics(curr_angle);

    create_odom();
    create_transforms();
    create_path_g();

    odom_pub_->publish(odom);
    tf_MG_broadcaster_->sendTransform(tf_MG_stamped);
  }

  void create_transforms()
  {
    tf_MG_stamped.header.frame_id = odom_id;
    tf_MG_stamped.child_frame_id = body_id;
    tf_MG_stamped.header.stamp = get_clock()->now();
    tf_MG_stamped.transform.translation.x = config.x;
    tf_MG_stamped.transform.translation.y = config.y;
    tf_MG_stamped.transform.translation.z = 0.0;
    q_MG.setRPY(0, 0, config.theta);
    tf_MG_stamped.transform.rotation.x = q_MG.x();
    tf_MG_stamped.transform.rotation.y = q_MG.y();
    tf_MG_stamped.transform.rotation.z = q_MG.z();
    tf_MG_stamped.transform.rotation.w = q_MG.w();
  }

  void create_path_g()
  {
    g_path_trace.header.stamp = get_clock()->now();
    g_path_trace.header.frame_id = odom_id;

    g_pose_trace.header.stamp = get_clock()->now();
    g_pose_trace.header.frame_id = "green/base_footprint";
    g_pose_trace.pose.position.x = green_config.x;
    g_pose_trace.pose.position.y = green_config.y;
    g_pose_trace.pose.position.z = 0;

    std::cout << "slam x: " << green_config.x << ", slam y: " << green_config.y << ", slam rad: " <<
      green_config.theta << std::endl;

    tf2::Quaternion q2;
    q2.setRPY(0, 0, green_config.theta);
    g_pose_trace.pose.orientation.x = q2.x();
    g_pose_trace.pose.orientation.y = q2.y();
    g_pose_trace.pose.orientation.z = q2.z();
    g_pose_trace.pose.orientation.w = q2.w();

    g_path_trace.poses.push_back(g_pose_trace);
    g_path_trace_pub_->publish(g_path_trace);
  }


  void create_odom()
  {
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
  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating Slam Node. ");

  rclcpp::spin(std::make_shared<Slam>());

  rclcpp::shutdown();
  return 0;
}
