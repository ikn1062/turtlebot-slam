#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of the Differential Drive Robot


#include <iosfwd>
#include <cmath>
#include <iostream>
#include <string>
#include <turtlelib/rigid2d.hpp>

namespace turtlelib
{
/// \brief Radius of the wheel of the Differential Drive Robot
constexpr double WHEEL_RADIUS = .033;

/// \brief Distance between the wheels of the Differential Drive Robot
constexpr double WHEEL_TRACK = 0.16;

/// \brief A vector to track the angular positions of the robot's wheels
struct WheelAngle
{
  /// \brief The angle of the left wheel
  double left;

  /// \brief The angle of the right wheel
  double right;
};

/// \brief A vector to track the angular velocity of the robot's wheels
struct WheelVelocity
{
  /// \brief The rotational velocity of the left wheel
  double left;

  /// \brief The rotational velocity of the right wheel
  double right;
};

/// \brief A vector to track the configuration of the robot
struct Config
{
  /// \brief The current angle of the robot
  double theta;

  /// \brief The current x position of the robot
  double x;

  /// \brief The current y position of the robot
  double y;
};

/// \brief Contains kinematics functions for differential drive robot
class DiffDrive
{
public:
  /// \brief Initializes DiffDrive class with zero wheel angle, velocity, and configuration
  DiffDrive();

  /// \brief Initializes DiffDrive class with wheel angle, velocity, and configuration
  /// \param input_q Input robot configuration
  DiffDrive(Config input_q);

  /// \brief Initializes DiffDrive class with wheel angle, velocity, and configuration
  /// \param input_phi Input wheel angle
  /// \param input_phi_dot Input Wheel angular velocity
  /// \param input_q Input robot configuration
  DiffDrive(WheelAngle input_phi, WheelVelocity input_phi_dot, Config input_q);

  /// \brief Calculates the twist of the robot given the next angle of wheels
  /// Uses equations 3 and 4 within doc/diff-drive-kinematics.pdf
  /// \param phi_next Next Angle of wheels
  Twist2D bodyTwist(WheelAngle phi_next);

  /// \brief Calculates the twist of the robot given the next angle of wheels
  /// Uses equations 3 and 4 within doc/diff-drive-kinematics.pdf
  /// \param phi_next Next Angle of wheels
  /// \param phi_old Old Angle of wheels
  Twist2D bodyTwist(WheelAngle phi_next, WheelAngle phi_old);

  /// \brief Calculates the twist of the robot given the wheel velocity
  /// Uses equations 3 and 4 within doc/diff-drive-kinematics.pdf
  /// \param phi_dot Wheel velocity
  Twist2D bodyTwist(WheelVelocity phi_dot);

  /// \brief Implements forward kinematics - calculates new config given the next wheel angle
  /// \param phi_next Next wheel angles
  Config forwardKinematics(WheelAngle phi_next);

  /// \brief Implements forward kinematics - calculates new config given the next wheel angle
  /// \param phi_next Next wheel angles
  /// \param phi_old Old Angle of wheels
  Config forwardKinematics(WheelAngle phi_next, WheelAngle phi_old);

  /// \brief Implements inverse kinematics - calculates wheel velocity from twist
  /// Uses equations 1 and 2 within doc/diff-drive-kinematics.pdf
  /// \param twist Input twist of the differential drive robot
  WheelVelocity inverseKinematics(Twist2D twist);

private:
  WheelAngle phi;
  WheelVelocity phi_dot;
  Config q;
};
}

#endif
