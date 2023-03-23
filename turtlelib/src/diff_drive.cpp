#include <turtlelib/diff_drive.hpp>

namespace turtlelib
{
DiffDrive::DiffDrive()
: phi{0.0, 0.0}, phi_dot{0.0, 0.0}, q{0.0, 0.0, 0.0}
{}

DiffDrive::DiffDrive(Config input_q)
: phi{0.0, 0.0}, phi_dot{0.0, 0.0}, q{input_q.theta, input_q.x, input_q.y}
{}

DiffDrive::DiffDrive(WheelAngle input_phi, WheelVelocity input_phi_dot, Config input_q)
: phi{input_phi.left, input_phi.right}, phi_dot{input_phi_dot.left, input_phi_dot.right},
  q{input_q.theta, input_q.x, input_q.y}
{}

Twist2D DiffDrive::bodyTwist(WheelAngle phi_next)
{
  Twist2D twist;
  phi_dot.left = phi_next.left - phi.left;
  phi_dot.right = phi_next.right - phi.right;

  twist.dtheta = (WHEEL_RADIUS / WHEEL_TRACK) * (phi_dot.right - phi_dot.left);
  twist.dx = (0.5 * WHEEL_RADIUS) * (phi_dot.left + phi_dot.right);
  twist.dy = 0;
  return twist;
}

Twist2D DiffDrive::bodyTwist(WheelVelocity input_phi_dot)
{
  Twist2D twist;
  twist.dtheta = (WHEEL_RADIUS / WHEEL_TRACK) * (input_phi_dot.right - input_phi_dot.left);
  twist.dx = (0.5 * WHEEL_RADIUS) * (input_phi_dot.left + input_phi_dot.right);
  twist.dy = 0;
  return twist;
}

Twist2D DiffDrive::bodyTwist(WheelAngle phi_next, WheelAngle phi_old)
{
  Twist2D twist;
  phi_dot.left = phi_next.left - phi_old.left;
  phi_dot.right = phi_next.right - phi_old.right;

  twist.dtheta = (WHEEL_RADIUS / WHEEL_TRACK) * (phi_dot.right - phi_dot.left);
  twist.dx = (0.5 * WHEEL_RADIUS) * (phi_dot.left + phi_dot.right);
  twist.dy = 0;
  return twist;
}

Config DiffDrive::forwardKinematics(WheelAngle phi_next)
{
  Twist2D body_twist;
  Vector2D curr_pos;
  Transform2D Twb, Tbb_prime, Twb_prime;

  body_twist = bodyTwist(phi_next);
  if (body_twist.dy != 0) {
    throw std::logic_error("Invalid Twist - Twist cannot be accomplished without wheels slipping");
  }

  Twb = Transform2D({q.x, q.y}, q.theta);
  Tbb_prime = integrate_twist(body_twist);
  Twb_prime = Twb * Tbb_prime;

  q.theta = Twb_prime.rotation();
  q.x = Twb_prime.translation().x;
  q.y = Twb_prime.translation().y;
  phi.left = phi_next.left;
  phi.right = phi_next.right;
  return q;
}

Config DiffDrive::forwardKinematics(WheelAngle phi_next, WheelAngle phi_old)
{
  Twist2D body_twist;
  Vector2D curr_pos;
  Transform2D Twb, Tbb_prime, Twb_prime;

  body_twist = bodyTwist(phi_next, phi_old);
  if (body_twist.dy != 0) {
    throw std::logic_error("Invalid Twist - Twist cannot be accomplished without wheels slipping");
  }

  curr_pos.x = q.x;
  curr_pos.y = q.y;

  Twb = Transform2D(curr_pos, q.theta);
  Tbb_prime = integrate_twist(body_twist);
  Twb_prime = Twb * Tbb_prime;

  q.theta = Twb_prime.rotation();
  q.x = Twb_prime.translation().x;
  q.y = Twb_prime.translation().y;
  phi.left = phi_next.left;
  phi.right = phi_next.right;
  return q;
}

WheelVelocity DiffDrive::inverseKinematics(Twist2D twist)
{
  if (twist.dy != 0) {
    throw std::logic_error("Invalid Twist - Twist cannot be accomplished without wheels slipping");
  }

  WheelVelocity phi_dot;
  phi_dot.left = (1 / WHEEL_RADIUS) * (-twist.dtheta * (WHEEL_TRACK / 2.0) + twist.dx);
  phi_dot.right = (1 / WHEEL_RADIUS) * (twist.dtheta * (WHEEL_TRACK / 2.0) + twist.dx);
  return phi_dot;
}

}
