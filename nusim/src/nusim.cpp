/// \file
/// \brief The nusim node initializes a turtlebot in the rviz environment with obstacles
///  and creates services for teleport and reset.
///
/// PARAMETERS:
///     rate (double): The frequency at which the main loop of the node runs
///     x0 (double): The initial position of the turtlebot in x
///     y0 (double): The initial position of the turtlebot in y
///     w0 (double): The initial position of the turtlebot in theta
///     obstacles/x (std::vector<double>): A list of the obstacles x-position
///     obstacles/y (std::vector<double>): A list of the obstacles y-position
///     obstacles/r (double): The radius of the obstacles
///     x_length (double): X length of arena wall
///     y_length (double): Y length of arena wall
///     motor_cmd_per_rad_sec (double): Motor Commands per Radians
///     encoder_ticks_per_rad (double): Radians per encoder tick
///     input_noise (double): zero mean Guassian noise with variance input_noise to commands
///     slip_fraction (double): Adding wheel slippage
///     basic_sensor_variance (double): Zero mean gaussian noise with variance added to obstacles
///     max_range (double): Max range of obstacle detection
///     collision_radius (double): Collision radius of robot object
///     angle_max (double): Max angle of lidar
///     angle_min (double): min angle of lidar
///     angle_increment (double): increment angle of lidar
///     scan_time (double): Scan time of lidar
///     range_min (double): Min range of lidar
///     range_max (double): Max range of Lidar
///     draw_only (double): Bool to check to draw fake obstacles
/// PUBLISHES:
///     timestep_pub_ (std_msgs::msg::UInt64): Internal topic publisher to track nusim simulations current timestep
///     marker_pub_ (visualization_msgs::msg::MarkerArray): Creates obstacles in RVIZ simulation environment
///     wall_pub_ (visualization_msgs::msg::MarkerArray): Creates arena walls in RVIZ simulation environment
///     sensor_pub_ (nuturtlebot_msgs::msg::WheelCommands): Publishes sensor data to the turtlebot3
///     fake_marker_pub_ (visualization_msgs::msg::MarkerArray): Publishes fake sensor data
///     laser_pub_ (sensor_msgs::msg::LaserScan): Publishes lidar scan data
///     path_trace_pub_ (nav_msgs::msg::Path: Publishes a path trace of the Red robot over time
/// SUBSCRIBES:
///     wheelcmd_sub_ (nuturtlebot_msgs::msg::WheelCommands): Receives wheel commands to turtlebot3
/// SERVERS:
///     nusim/reset (Empty): Resets the turtlebot to the original position in the world frame
///     nusim/teleport (nusim::srv::Teleport): Moves the turtlebot to the position provided

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nusim/srv/teleport.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>

using namespace std::chrono;

class Simulation : public rclcpp::Node
{
public:
  Simulation()
  : Node("nusim_node")
  {
    // NUSIM SIMULATION CONSTRUCTOR

    // Declare all Parameters here
    declare_parameter("rate", 50.0);

    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("w0", 0.0);

    declare_parameter("obstacles/x", cylinder_x_list);
    declare_parameter("obstacles/y", cylinder_y_list);
    declare_parameter("obstacles/r", cylinder_r);

    declare_parameter("x_length", 4.0);
    declare_parameter("y_length", 4.0);

    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 4096);

    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);

    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 0.5);

    declare_parameter("collision_radius", 0.0);

    declare_parameter("laser_variance", 0.0);
    declare_parameter("angle_max", 0.0);
    declare_parameter("angle_min", 0.0);
    declare_parameter("angle_increment", 0.0);
    declare_parameter("scan_time", 0.0);
    declare_parameter("range_min", 0.0);
    declare_parameter("range_max", 0.0);

    declare_parameter("draw_only", false);

    // Assign Parameters to variables
    rate_ = get_parameter("rate").as_double();
    auto duration_ = std::chrono::duration<double>(1.0 / rate_);
    auto sensor_duration_ = std::chrono::duration<double>(1.0 / 5.0);

    init_pos.x = curr_pos.x = get_parameter("x0").as_double();
    init_pos.y = curr_pos.y = get_parameter("y0").as_double();
    init_pos.theta = curr_pos.theta = get_parameter("w0").as_double();

    x_length = curr_pos.theta = get_parameter("x_length").as_double();
    y_length = curr_pos.theta = get_parameter("y_length").as_double();

    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_int();

    input_noise = get_parameter("input_noise").as_double();
    slip_fraction = get_parameter("slip_fraction").as_double();

    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    max_range = get_parameter("max_range").as_double();

    collision_rad = get_parameter("collision_radius").as_double();

    draw_only = get_parameter("draw_only").as_bool();

    cylinder_x_list = get_parameter("obstacles/x").as_double_array();
    cylinder_y_list = get_parameter("obstacles/y").as_double_array();
    cylinder_r = get_parameter("obstacles/r").as_double();
    if (cylinder_x_list.size() != cylinder_y_list.size()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "rclcpp"), "Error: cylinder obstacles x and y not of same length");
      rclcpp::shutdown();
    }

    laser_variance = get_parameter("laser_variance").as_double();
    angle_max = get_parameter("angle_max").as_double();
    angle_min = get_parameter("angle_min").as_double();
    angle_inc = get_parameter("angle_increment").as_double();
    scan_time = get_parameter("scan_time").as_double();
    range_min = get_parameter("range_min").as_double();
    range_max = get_parameter("range_max").as_double();

    // Create Publisher / Subscribers / Services
    timestep_.data = 0;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 50);

    reset_srv_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Simulation::reset, this, std::placeholders::_1, std::placeholders::_2));
    teleport_srv_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Simulation::teleport, this, std::placeholders::_1, std::placeholders::_2));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wall_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10);
    wheelcmd_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd",
      10,
      std::bind(&Simulation::wheel_cmd_cb, this, std::placeholders::_1));

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

    fake_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", 10);

    path_trace_pub_ = create_publisher<nav_msgs::msg::Path>("~/red_path", 10);

    laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("~/sim_laser", 10);

    // Initialize Variables
    sensor_.stamp = get_clock()->now();
    sensor_.left_encoder = 0;
    sensor_.right_encoder = 0;

    create_tf_msg_(init_pos);
    tf_broadcaster_->sendTransform(tf_stamped);

    curr_pos.x = init_pos.x;
    curr_pos.y = init_pos.y;
    curr_pos.theta = init_pos.theta;

    diffDrive = turtlelib::DiffDrive(curr_pos);

    wheel_ang = {0.0, 0.0};
    wheel_vel = {0.0, 0.0};
    wheel_ang_slip = {0.0, 0.0};

    joint_state_msg.header.stamp = rclcpp::Clock{}.now();
    joint_state_msg.name = {left_joint, right_joint};
    joint_state_msg.position = {0.0, 0.0};
    joint_state_msg.velocity = {0.0, 0.0};

    sensor_.left_encoder = 0;
    sensor_.right_encoder = 0;

    timestep_.data = 0;
    timestep_pub_->publish(timestep_);

    // Create main callback
    timer_ = create_wall_timer(duration_, std::bind(&Simulation::nusim_main, this));
    sensor_timer_ = create_wall_timer(sensor_duration_, std::bind(&Simulation::nusim_sensor, this));
  }

private:
  // Main loop publisher and timer
  rclcpp::TimerBase::SharedPtr timer_;
  double rate_;

  // Publishes the current timestep of the simulation
  std_msgs::msg::UInt64 timestep_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;

  // Creates the service for reset and teleport
  turtlelib::Config init_pos;
  turtlelib::Config curr_pos;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;

  // Transform broadcaster message and pubslisher
  geometry_msgs::msg::TransformStamped tf_stamped;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Creates publsiher and variables for markers
  std::vector<double> cylinder_x_list;
  std::vector<double> cylinder_y_list;
  double cylinder_r = 0.0;
  visualization_msgs::msg::MarkerArray obstacle_arr;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Creates publsiher and variables for wall
  double x_length;
  double y_length;
  double wall_width = 0.2;
  visualization_msgs::msg::MarkerArray walls;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;

  // Sub and Pub for sensor data and wheel cmds for red robot
  turtlelib::DiffDrive diffDrive;
  turtlelib::WheelVelocity wheel_vel;
  turtlelib::WheelAngle wheel_ang;
  double motor_cmd_per_rad_sec;
  int encoder_ticks_per_rad;
  std::string left_joint = "wheel_left_joint";
  std::string right_joint = "wheel_right_joint";
  nuturtlebot_msgs::msg::SensorData sensor_;
  sensor_msgs::msg::JointState joint_state_msg;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Adding Error/Slip to the turtlebot3 simulation
  double input_noise;
  double slip_fraction;
  double left_slip_noise;
  double right_slip_noise;
  double left_cmd_noise;
  double right_cmd_noise;
  double collision_rad;
  turtlelib::WheelAngle wheel_ang_slip;

  // Timer and rate for publishing relative markers and sensor data
  rclcpp::TimerBase::SharedPtr sensor_timer_;

  // Relative measurements of the markers
  bool draw_only;
  double basic_sensor_variance;
  double max_range;
  visualization_msgs::msg::MarkerArray fake_obstacle_arr;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_marker_pub_;

  // Laser scan of the robot
  double laser_variance;
  double angle_max;
  double angle_min;
  double angle_inc;
  double scan_time;
  double range_min;
  double range_max;
  sensor_msgs::msg::LaserScan laser_scan;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

  // Path trace of the robot
  nav_msgs::msg::Path path_trace;
  geometry_msgs::msg::PoseStamped pose_trace;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_trace_pub_;

  void nusim_main()
  {
    // Make tf_stamped into a function
    obstacle_arr = create_obstacles(cylinder_x_list, cylinder_y_list, cylinder_r);
    marker_pub_->publish(obstacle_arr);

    create_walls();
    wall_pub_->publish(walls);

    std::uniform_real_distribution<> slip(-1 * slip_fraction, slip_fraction);
    double left_slip_noise = slip(get_random());
    double right_slip_noise = slip(get_random());

    wheel_ang_slip.left = ((1.0 + left_slip_noise) * (wheel_vel.left / rate_)) +
      wheel_ang_slip.left;
    wheel_ang_slip.right = ((1.0 + right_slip_noise) * (wheel_vel.right / rate_)) +
      wheel_ang_slip.right;
    wheel_ang.left = (wheel_vel.left / rate_) + wheel_ang.left;
    wheel_ang.right = (wheel_vel.right / rate_) + wheel_ang.right;

    joint_state_msg.header.stamp = get_clock()->now();
    joint_state_msg.name = {left_joint, right_joint};
    joint_state_msg.position = {wheel_ang.left, wheel_ang.right};
    joint_state_msg.velocity = {wheel_vel.left, wheel_vel.right};

    joint_states_pub_->publish(joint_state_msg);

    curr_pos = diffDrive.forwardKinematics(wheel_ang);

    check_collision_obs();

    sensor_.stamp = get_clock()->now();
    sensor_.left_encoder = (int) (wheel_ang_slip.left * encoder_ticks_per_rad);
    sensor_.right_encoder = (int) (wheel_ang_slip.right * encoder_ticks_per_rad);

    create_tf_msg_(curr_pos);
    tf_broadcaster_->sendTransform(tf_stamped);

    sensor_pub_->publish(sensor_);

    timestep_.data += (1 / rate_) * 1000;   // converts to an int, note that this is in ms
    timestep_pub_->publish(timestep_);
  }

  void nusim_sensor()
  {
    nusim_create_path();
    nusim_obstacle_noise();
    nusim_lidar();
    path_trace_pub_->publish(path_trace);
    laser_pub_->publish(laser_scan);
    fake_marker_pub_->publish(fake_obstacle_arr);
  }

  void reset(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    sensor_.stamp = get_clock()->now();

    create_tf_msg_(init_pos);
    tf_broadcaster_->sendTransform(tf_stamped);

    curr_pos.x = init_pos.x;
    curr_pos.y = init_pos.y;
    curr_pos.theta = init_pos.theta;

    diffDrive = turtlelib::DiffDrive(curr_pos);

    wheel_ang = {0.0, 0.0};
    wheel_vel = {0.0, 0.0};
    wheel_ang_slip = {0.0, 0.0};

    sensor_.left_encoder = 0;
    sensor_.right_encoder = 0;

    timestep_.data = 0;
    timestep_pub_->publish(timestep_);
  }

  bool teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> pose,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    curr_pos.x = pose->x;
    curr_pos.y = pose->y;
    curr_pos.theta = pose->theta;

    return true;
  }

  visualization_msgs::msg::MarkerArray create_obstacles(
    std::vector<double> obstacles_x,
    std::vector<double> obstacles_y,
    double obstacles_r)
  {
    obstacle_arr.markers.resize(obstacles_x.size());

    for (unsigned int i = 0; i < obstacles_x.size(); i++) {
      obstacle_arr.markers[i].header.stamp = get_clock()->now();
      obstacle_arr.markers[i].header.frame_id = "nusim/world";
      obstacle_arr.markers[i].ns = "nusim_node/obstacles";

      obstacle_arr.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_arr.markers[i].action = visualization_msgs::msg::Marker::ADD;

      obstacle_arr.markers[i].id = i;
      obstacle_arr.markers[i].pose.position.x = obstacles_x[i];
      obstacle_arr.markers[i].pose.position.y = obstacles_y[i];
      obstacle_arr.markers[i].pose.position.z = .125;

      obstacle_arr.markers[i].pose.orientation.x = 0.0;
      obstacle_arr.markers[i].pose.orientation.y = 0.0;
      obstacle_arr.markers[i].pose.orientation.z = 0.0;
      obstacle_arr.markers[i].pose.orientation.w = 1.0;

      obstacle_arr.markers[i].scale.x = 2 * obstacles_r;
      obstacle_arr.markers[i].scale.y = 2 * obstacles_r;
      obstacle_arr.markers[i].scale.z = .25;

      obstacle_arr.markers[i].color.r = 1.0;
      obstacle_arr.markers[i].color.g = 0.0;
      obstacle_arr.markers[i].color.b = 0.0;
      obstacle_arr.markers[i].color.a = 1.0;

    }
    return obstacle_arr;
  }

  void create_walls()
  {
    walls.markers.resize(4);

    for (unsigned int i = 0; i < 4; i++) {
      if (i == 0) {
        walls.markers[i].pose.position.x = x_length / 2.0;
        walls.markers[i].pose.position.y = 0.0;
        walls.markers[i].scale.x = wall_width;
        walls.markers[i].scale.y = y_length + wall_width;
      }
      if (i == 1) {
        walls.markers[i].pose.position.x = -1.0 * x_length / 2.0;
        walls.markers[i].pose.position.y = 0.0;
        walls.markers[i].scale.x = wall_width;
        walls.markers[i].scale.y = y_length + wall_width;
      }
      if (i == 2) {
        walls.markers[i].pose.position.x = 0.0;
        walls.markers[i].pose.position.y = y_length / 2.0;
        walls.markers[i].scale.x = x_length + wall_width;
        walls.markers[i].scale.y = wall_width;
      }
      if (i == 3) {
        walls.markers[i].pose.position.x = 0.0;
        walls.markers[i].pose.position.y = -y_length / 2.0;
        walls.markers[i].scale.x = x_length + wall_width;
        walls.markers[i].scale.y = wall_width;
      }

      walls.markers[i].header.stamp = get_clock()->now();
      walls.markers[i].header.frame_id = "nusim/world";
      walls.markers[i].ns = "nusim_node/walls";
      walls.markers[i].type = visualization_msgs::msg::Marker::CUBE;
      walls.markers[i].action = visualization_msgs::msg::Marker::ADD;
      walls.markers[i].id = i;

      walls.markers[i].pose.position.z = 0.25;

      walls.markers[i].pose.orientation.x = 0.0;
      walls.markers[i].pose.orientation.y = 0.0;
      walls.markers[i].pose.orientation.z = 0.0;
      walls.markers[i].pose.orientation.w = 1.0;

      walls.markers[i].scale.z = 0.25;

      walls.markers[i].color.r = 1.0;
      walls.markers[i].color.g = 0.0;
      walls.markers[i].color.b = 0.0;
      walls.markers[i].color.a = 1.0;
    }
  }

  void wheel_cmd_cb(const nuturtlebot_msgs::msg::WheelCommands & wheel_cmd)
  {
    // Creating zero_mean gaussian nosie
    double left_noise = 0.0;
    double right_noise = 0.0;
    std::normal_distribution<> d(0.0, input_noise);
    if (!turtlelib::almost_equal(
        wheel_cmd.left_velocity,
        0.01) && !turtlelib::almost_equal(wheel_cmd.right_velocity, 0.01))
    {
      left_cmd_noise = d(get_random());
      right_cmd_noise = d(get_random());
    }

    // Adding noise to wheel velocity
    wheel_vel.left = (1 + left_noise) * (wheel_cmd.left_velocity * motor_cmd_per_rad_sec);
    wheel_vel.right = (1 + right_noise) * (wheel_cmd.right_velocity * motor_cmd_per_rad_sec);
  }

  void create_tf_msg_(turtlelib::Config input_position)
  {
    tf_stamped.header.stamp = get_clock()->now();

    tf_stamped.header.frame_id = "nusim/world";
    tf_stamped.child_frame_id = "red/base_footprint";
    tf_stamped.transform.translation.x = input_position.x;
    tf_stamped.transform.translation.y = input_position.y;
    tf_stamped.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, input_position.theta);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
  }

  std::mt19937 & get_random()
  {
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
  }

  void check_collision_obs()
  {
    // Using this resource in circle collision
    // https://flatredball.com/documentation/tutorials/math/circle-collision/
    double collision_dist;
    double collision_radius = cylinder_r + collision_rad;
    for (unsigned int i = 0; i < cylinder_x_list.size(); i++) {
      collision_dist =
        sqrt(pow(curr_pos.x - cylinder_x_list[i], 2) + pow(curr_pos.y - cylinder_y_list[i], 2));
      if (collision_dist < collision_radius) {
        double coll_angle = atan2(curr_pos.y - cylinder_y_list[i], curr_pos.x - cylinder_x_list[i]);
        double dist_to_move = collision_radius - collision_dist;
        curr_pos.x += cos(coll_angle) * dist_to_move;
        curr_pos.y += sin(coll_angle) * dist_to_move;
      }
    }
  }

  struct intersection_points
  {
    turtlelib::Vector2D p1;
    turtlelib::Vector2D p2;
  };

  void nusim_lidar()
  {
    // Create Laser Scan Message
    std::normal_distribution<> laser_noise(0, laser_variance);
    int len_measurements = (angle_max - angle_min) / angle_inc;

    laser_scan.header.stamp = get_clock()->now();
    laser_scan.header.frame_id = "red/base_scan";
    laser_scan.angle_min = angle_min;
    laser_scan.angle_max = angle_max;
    laser_scan.angle_increment = angle_inc;
    laser_scan.scan_time = scan_time;
    laser_scan.time_increment = 0.5 / 1000000000.0;
    laser_scan.range_min = range_min;
    laser_scan.range_max = range_max;
    laser_scan.ranges.resize(len_measurements);

    // Store Laser Range for each angle
    double laser_range;

    // Get Current Robot Transformation
    turtlelib::Config robot_pos;
    robot_pos.x = curr_pos.x;
    robot_pos.y = curr_pos.y;
    robot_pos.theta = curr_pos.theta;
    turtlelib::Transform2D T_world_robot({robot_pos.x, robot_pos.y}, robot_pos.theta);
    turtlelib::Transform2D T_robot_world = T_world_robot.inv();
    turtlelib::Vector2D worls_obs_trans;
    turtlelib::Transform2D T_world_obs;
    turtlelib::Transform2D T_obs_world;
    turtlelib::Transform2D T_obs_robot;
    turtlelib::Transform2D T_robot_obs;

    // Create Objects for Obstacle Laser Intersection
    turtlelib::Vector2D range_min_vec;
    turtlelib::Vector2D range_max_vec;
    turtlelib::Vector2D vec_obs_min;
    turtlelib::Vector2D vec_obs_max;
    double dx, dy, dr, D, disc;
    int sign;
    turtlelib::Vector2D intersection1;
    turtlelib::Vector2D intersection2;
    turtlelib::Vector2D world_int_1;
    turtlelib::Vector2D world_int_2;
    double dist1, dist2;
    double point_x, point_y;
    double min_obs_dist;

    // Create objects for Wall Laser Intersection

    std::vector<intersection_points> wall_vector_points = get_wall_points();
    intersection_points robot_coordinates;
    int check_intersection;
    turtlelib::Vector2D wall_int;
    turtlelib::Vector2D robot_int;
    double wall_dist;

    // Loop for everu single laser in the range of angle min to max with inc
    int iter = 0;
    for (double angle = angle_min; angle <= angle_max; angle += angle_inc) {
      laser_range = range_max + 1; // distance to use for laser

      range_min_vec.x = range_min * cos(angle);
      range_min_vec.y = range_min * sin(angle);
      range_max_vec.x = range_max * cos(angle);
      range_max_vec.y = range_max * sin(angle);

      // Obstacles
      for (size_t obs_i = 0; obs_i < cylinder_x_list.size(); obs_i++) {
        worls_obs_trans.x = cylinder_x_list[obs_i];
        worls_obs_trans.y = cylinder_y_list[obs_i];
        T_world_obs = turtlelib::Transform2D(worls_obs_trans);
        T_obs_world = T_world_obs.inv();
        T_obs_robot = T_obs_world * T_world_robot;
        T_robot_obs = T_obs_robot.inv();

        vec_obs_min = T_obs_robot(range_min_vec);
        vec_obs_max = T_obs_robot(range_max_vec);

        dx = vec_obs_max.x - vec_obs_min.x;
        dy = vec_obs_max.y - vec_obs_min.y;
        dr = sqrt(pow(dx, 2) + pow(dy, 2));
        D = vec_obs_min.x * vec_obs_max.y - vec_obs_max.x * vec_obs_min.y;
        disc = pow(cylinder_r * 2, 2) * pow(dr, 2) - pow(D, 2);
        sign = (dy < 0) ? -1 : 1;

        // intersection exists
        if (disc >= 0.0) {
          // find points of intersection
          intersection1.x = (D * dy + sign * dx * sqrt(disc)) / pow(dr, 2);
          intersection1.y = (-(D * dx) + abs(dy) * sqrt(disc)) / pow(dr, 2);
          intersection2.x = (D * dy - sign * dx * sqrt(disc)) / pow(dr, 2);
          intersection2.y = (-(D * dx) - abs(dy) * sqrt(disc)) / pow(dr, 2);

          world_int_1 = T_robot_obs(intersection1);
          world_int_2 = T_robot_obs(intersection2);

          dist1 = sqrt(pow(world_int_1.x, 2) + pow(world_int_1.y, 2));
          dist2 = sqrt(pow(world_int_2.x, 2) + pow(world_int_2.y, 2));

          min_obs_dist = (dist1 < dist2) ? dist1 : dist2;
          point_x = (dist1 < dist2) ? world_int_1.x : world_int_2.x;
          point_y = (dist1 < dist2) ? world_int_1.y : world_int_2.y;

          // TODO: May need a more involved check here (Can check points to see if in range of x and y):

          if (min_obs_dist <
            sqrt(pow(range_min_vec.x - point_x, 2) + pow(range_min_vec.y - point_y, 2)))
          {
            min_obs_dist = range_max + 1;
          }
        } else {
          min_obs_dist = range_max + 1;
        }
        laser_range = std::min(laser_range, min_obs_dist);
      }

      // Walls - if the laser did not hit any obstacles
      if (laser_range >= range_max) {
        // get current robot points
        robot_coordinates.p1.x = robot_pos.x;
        robot_coordinates.p1.y = robot_pos.y;
        robot_coordinates.p2.x = robot_pos.x + cos(angle + robot_pos.theta);
        robot_coordinates.p2.y = robot_pos.y + sin(angle + robot_pos.theta);

        // for each wall points -> find points of intersection
        for (int w = 0; w < 4; w++) {
          check_intersection =
            get_intersection(robot_coordinates, wall_vector_points[w], &wall_int);
          if (!check_intersection) {
            continue;
          }

          // convert intersection to robot frame using T_robot_world
          robot_int = T_robot_world(wall_int);

          // check distance and range_max
          wall_dist = sqrt(pow(robot_int.x, 2) + pow(robot_int.y, 2));
          if (wall_dist <
            sqrt(pow(range_min_vec.x - robot_int.x, 2) + pow(range_min_vec.y - robot_int.y, 2)))
          {
            wall_dist = range_max + 1;
          }
          if (wall_dist < range_max) {
            laser_range = std::min(laser_range, wall_dist);
          }
        }
      }

      if (laser_range >= range_max) {
        laser_range = 0.0;
      }
      // Add noise to laser scan range
      laser_scan.ranges[iter] = laser_range + laser_noise(get_random());
      iter++;
    }
  }

  std::vector<intersection_points> get_wall_points()
  {
    intersection_points wall_vec_1;
    intersection_points wall_vec_2;
    intersection_points wall_vec_3;
    intersection_points wall_vec_4;

    wall_vec_1.p1.x = -x_length / 2.0;
    wall_vec_1.p2.x = x_length / 2.0;
    wall_vec_1.p1.y = (y_length / 2.0) - wall_width;
    wall_vec_1.p2.y = (y_length / 2.0) - wall_width;

    wall_vec_2.p1.x = -x_length / 2.0;
    wall_vec_2.p2.x = x_length / 2.0;
    wall_vec_2.p1.y = (-y_length / 2.0) + wall_width;
    wall_vec_2.p2.y = (-y_length / 2.0) + wall_width;

    wall_vec_3.p1.x = (-x_length / 2.0) + wall_width;
    wall_vec_3.p2.x = (-x_length / 2.0) + wall_width;
    wall_vec_3.p1.y = y_length / 2.0;
    wall_vec_3.p2.y = -y_length / 2.0;

    wall_vec_4.p1.x = (x_length / 2.0) - wall_width;
    wall_vec_4.p2.x = (x_length / 2.0) - wall_width;
    wall_vec_4.p1.y = y_length / 2.0;
    wall_vec_4.p2.y = -y_length / 2.0;

    std::vector<intersection_points> wall_vector_points_out =
    {wall_vec_1, wall_vec_2, wall_vec_3, wall_vec_4};
    return wall_vector_points_out;
  }

  // Inspired by https://www.realtimerendering.com/resources/GraphicsGems/gemsii/xlines.c
  int get_intersection(
    intersection_points p_robot, intersection_points p_wall,
    turtlelib::Vector2D * wall_intersection)
  {
    double diff1_x, diff1_y, diff2_x, diff2_y;
    //double a;
    double b;
    double disc;

    diff1_x = p_robot.p2.x - p_robot.p1.x;
    diff1_y = p_robot.p2.y - p_robot.p1.y;
    diff2_x = p_wall.p2.x - p_wall.p1.x;
    diff2_y = p_wall.p2.y - p_wall.p1.y;

    disc = (-diff2_x * diff1_y + diff1_x * diff2_y);

    //a = (-diff1_y * (p_robot.p1.x - p_wall.p1.x) + diff1_x * (p_robot.p1.y - p_wall.p1.y)) / disc;
    b = (diff2_x * (p_robot.p1.y - p_wall.p1.y) - diff2_y * (p_robot.p1.x - p_wall.p1.x)) / disc;

    if (turtlelib::almost_equal((-diff2_x * diff1_y + diff1_x * diff2_y), 0.0)) {
      return 0;
    }

    wall_intersection->x = p_robot.p1.x + (b * diff1_x);
    wall_intersection->y = p_robot.p1.y + (b * diff1_y);
    return 1;
  }

  double get_D(double x1, double y1, double x2, double y2)
  {
    double D;
    D = x1 * y2 - x2 * y1;
    return D;
  }

  void nusim_obstacle_noise()
  {
    rclcpp::Duration nusim_sensor_dur = rclcpp::Duration::from_seconds(1.0 / 5.0);
    std::normal_distribution<> fake_noise(0.0, basic_sensor_variance);

    turtlelib::Transform2D T_world_robot({curr_pos.x, curr_pos.y}, curr_pos.theta);
    turtlelib::Transform2D T_robot_world = T_world_robot.inv();

    fake_obstacle_arr.markers.resize(cylinder_x_list.size());

    for (unsigned int i = 0; i < cylinder_x_list.size(); i++) {
      // Relative position from Robot -> Obstacle
      // T_robot_to_obs = T_robot_to_world * T_world_to_obs
      double obs_noise_x = fake_noise(get_random());
      double obs_noise_y = fake_noise(get_random());
      turtlelib::Vector2D fake_obs_pos(cylinder_x_list[i] + obs_noise_x,
        cylinder_y_list[i] + obs_noise_y);
      turtlelib::Transform2D T_world_obs(fake_obs_pos);
      turtlelib::Transform2D T_robot_obs = T_robot_world * T_world_obs;
      fake_obs_pos = T_robot_obs.translation();

      double dist_r_o =
        sqrt(pow(curr_pos.x - fake_obs_pos.x, 2) + pow(curr_pos.y - fake_obs_pos.y, 2));

      if (dist_r_o > max_range || draw_only) {
        fake_obstacle_arr.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      } else {
        fake_obstacle_arr.markers[i].action = visualization_msgs::msg::Marker::ADD;
      }

      fake_obstacle_arr.markers[i].header.stamp = get_clock()->now();
      fake_obstacle_arr.markers[i].header.frame_id = "red/base_footprint";
      fake_obstacle_arr.markers[i].ns = "nusim_node/fake_sensor";

      fake_obstacle_arr.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;

      fake_obstacle_arr.markers[i].id = i;
      fake_obstacle_arr.markers[i].pose.position.x = fake_obs_pos.x;
      fake_obstacle_arr.markers[i].pose.position.y = fake_obs_pos.y;
      fake_obstacle_arr.markers[i].pose.position.z = .125;

      fake_obstacle_arr.markers[i].pose.orientation.x = 0.0;
      fake_obstacle_arr.markers[i].pose.orientation.y = 0.0;
      fake_obstacle_arr.markers[i].pose.orientation.z = 0.0;
      fake_obstacle_arr.markers[i].pose.orientation.w = 1.0;

      fake_obstacle_arr.markers[i].scale.x = 2 * cylinder_r;
      fake_obstacle_arr.markers[i].scale.y = 2 * cylinder_r;
      fake_obstacle_arr.markers[i].scale.z = .25;

      fake_obstacle_arr.markers[i].color.r = 1.0;
      fake_obstacle_arr.markers[i].color.g = 1.0;
      fake_obstacle_arr.markers[i].color.b = 0.0;
      fake_obstacle_arr.markers[i].color.a = 1.0;

      fake_obstacle_arr.markers[i].lifetime = nusim_sensor_dur;
      fake_obstacle_arr.markers[i].frame_locked = true;
    }
  }

  void nusim_create_path()
  {
    path_trace.header.stamp = get_clock()->now();
    path_trace.header.frame_id = "nusim/world";

    pose_trace.header.stamp = get_clock()->now();
    pose_trace.header.frame_id = "red/base_footprint";
    pose_trace.pose.position.x = curr_pos.x;
    pose_trace.pose.position.y = curr_pos.y;
    pose_trace.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, curr_pos.theta);
    pose_trace.pose.orientation.x = q.x();
    pose_trace.pose.orientation.y = q.y();
    pose_trace.pose.orientation.z = q.z();
    pose_trace.pose.orientation.w = q.w();

    std::cout << "sim x: " << curr_pos.x << ", sim y: " << curr_pos.y << ", sim rad: " <<
      curr_pos.theta << std::endl;

    path_trace.poses.push_back(pose_trace);
  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating nusim Simulation Node. ");

  rclcpp::spin(std::make_shared<Simulation>());

  rclcpp::shutdown();
  return 0;
}
