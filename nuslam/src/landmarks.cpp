/// \file
/// \brief Landmark node associates circular landmarks with a given lidar scan
///
/// PARAMETERS:
///     turtle_id (string): The name of the turtle to associate landmarks with
///     laser_name (string): Name of laser topic to subscribe to
/// PUBLISHES:
///    obstacle_pub_ (sensor_msgs::msg::LaserScan): Creates circular obstacles using regression
/// SUBSCRIBES:
///     laser_sub_ (visualization_msgs::msg::MarkerArray): Receives laser scan messages


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
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"


#include <armadillo>
#include <nuslam/ekf_slam.hpp>
#include <nuslam/circle_library.hpp>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>


using namespace std::chrono;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Declares all node parameters for landmarks node
    declare_parameter("turtle_id", "red/base_footprint");
    declare_parameter("laser_name", "nusim_node/sim_laser");

    turtle_id = get_parameter("turtle_id").as_string();
    laser_name = get_parameter("laser_name").as_string();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      laser_name,
      rclcpp::SensorDataQoS(), std::bind(&Landmarks::laser_cb, this, std::placeholders::_1));
    // cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/clusters", 10);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
  }

private:
  std::string turtle_id;
  std::string laser_name;

  ekf_slam::landmarkCircle Circle = ekf_slam::landmarkCircle();
  std::vector<std::vector<turtlelib::Vector2D>> cluster_list;

  // visualization_msgs::msg::MarkerArray clusters;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;

  visualization_msgs::msg::MarkerArray obstacles;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;


  void laser_cb(const sensor_msgs::msg::LaserScan & laserScan)
  {
    cluster_list = Circle.landmarkCluster(
      laserScan.ranges, laserScan.angle_min,
      laserScan.angle_increment, laserScan.range_min,
      laserScan.range_max);
    create_obstacles();
    return;
  }

  void create_obstacles()
  {
    int cluster_size = cluster_list.size();
    std::vector<turtlelib::Vector2D> current_cluster;

    obstacles.markers.clear();

    for (int i = 0; i < cluster_size; i++) {
      current_cluster = cluster_list[i];
      // if (!Circle.classifyCircle(current_cluster)) {
      //     continue;
      // }
      double xcoord, ycoord, rad;
      bool check = Circle.circleFit(current_cluster, xcoord, ycoord, rad);
      if (!check) {
        continue;
      }

      visualization_msgs::msg::Marker cluster_circle;

      cluster_circle.header.stamp = get_clock()->now();
      cluster_circle.header.frame_id = turtle_id;
      cluster_circle.ns = "landmarks";

      cluster_circle.type = visualization_msgs::msg::Marker::CYLINDER;

      if (rad < 0.01 || rad > 0.1) {
        cluster_circle.action = visualization_msgs::msg::Marker::DELETE;
        continue;
      } else {
        //std::cout << "cluster size: " << current_cluster.size() << std::endl;
        //std::cout << "x: " << xcoord << ", y: " << ycoord << ", rad: " << rad << std::endl;
        cluster_circle.action = visualization_msgs::msg::Marker::ADD;
      }

      cluster_circle.id = i;
      cluster_circle.pose.position.x = xcoord;
      cluster_circle.pose.position.y = ycoord;
      cluster_circle.pose.position.z = 0.125;

      cluster_circle.pose.orientation.x = 0.0;
      cluster_circle.pose.orientation.y = 0.0;
      cluster_circle.pose.orientation.z = 0.0;
      cluster_circle.pose.orientation.w = 1.0;

      cluster_circle.scale.x = rad;       // rad
      cluster_circle.scale.y = rad;       // rad
      cluster_circle.scale.z = 0.5;

      cluster_circle.color.r = 0.0;
      cluster_circle.color.g = 0.2;
      cluster_circle.color.b = 0.8;
      cluster_circle.color.a = 0.5;

      //cluster_circle.frame_locked = true;

      obstacles.markers.push_back(cluster_circle);
    }

    obstacle_pub_->publish(obstacles);
  }

  void create_clusters()
  {

  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating Landmarks Node. ");

  rclcpp::spin(std::make_shared<Landmarks>());

  rclcpp::shutdown();
  return 0;
}
