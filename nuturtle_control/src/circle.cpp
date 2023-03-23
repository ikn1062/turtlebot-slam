/// \file
/// \brief The circle node makes the turtlebot3 move in a circle with a velocity and radius
///
/// PARAMETERS:
///     frequency (double): The frequency at which the main loop of the node runs
/// PUBLISHES:
///     cmdvel_pub_ (geometry_msgs::msg::Twist): Publishes a velocity twist command to make the robot move
/// SERVERS:
///     control (nuturtle_control::srv::Control): Controls the turtlebot given an angular velocity and radius
///     reverse (Empty): Reverses the direction of the turtlebot velocity
///     stop (Empty): Stops the turtlebot from moving


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"), radius(0.0), velocity(0.0), stop(true)
  {
    declare_parameter("frequency", 100.0);         // "Rate of Node (Hz)"

    get_parameter("frequency", freq_);

    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twist.linear.y = 0.0;

    control_srv_ =
      create_service<nuturtle_control::srv::Control>(
      "~/control",
      std::bind(&Circle::control_cb_, this, std::placeholders::_1, std::placeholders::_2));
    reverse_srv_ =
      create_service<std_srvs::srv::Empty>(
      "~/reverse",
      std::bind(&Circle::reverse_cb_, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ =
      create_service<std_srvs::srv::Empty>(
      "~/stop",
      std::bind(&Circle::stop_cb_, this, std::placeholders::_1, std::placeholders::_2));

    cmdvel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto duration_ = std::chrono::duration<double>(1 / freq_);
    timer_ = create_wall_timer(duration_, std::bind(&Circle::circle_main, this));

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double freq_;
  double radius;
  double velocity;
  bool stop;

  geometry_msgs::msg::Twist twist;

  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;

  bool control_cb_(
    std::shared_ptr<nuturtle_control::srv::Control::Request> ctrl,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    stop = false;
    radius = ctrl->radius;
    velocity = ctrl->velocity;

    twist.linear.x = velocity * radius;
    twist.angular.z = velocity;

    return true;
  }

  bool reverse_cb_(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    stop = false;
    twist.linear.x = -1 * velocity * radius;
    twist.angular.z = -1 * velocity;

    return true;
  }

  bool stop_cb_(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    stop = true;
    twist.linear.x = 0;
    twist.angular.z = 0;

    cmdvel_pub_->publish(twist);

    return true;
  }

  void circle_main()
  {
    if (!stop) {
      cmdvel_pub_->publish(twist);
    }
  }


};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating Circle Node. ");

  rclcpp::spin(std::make_shared<Circle>());

  rclcpp::shutdown();
  return 0;
}
