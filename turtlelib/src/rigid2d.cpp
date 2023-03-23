#include "turtlelib/rigid2d.hpp"
#include <iostream>


double turtlelib::normalize_angle(double rad)
{
  double new_rad = std::fmod(rad, 2.0 * PI);
  if (new_rad <= -1.0 * PI) {
    new_rad += 2.0 * PI;
  }
  if (new_rad > PI) {
    new_rad -= 2.0 * PI;
  }
  return new_rad;
}


// Vector2D Functions
turtlelib::Vector2D::Vector2D()
: x(0.0), y(0.0)
{}


turtlelib::Vector2D::Vector2D(double x_in, double y_in)
: x(x_in), y(y_in)
{}


void turtlelib::Vector2D::normalise()
{
  const auto norm_factor = sqrt(pow(x, 2) + pow(y, 2));
  x = x / norm_factor;
  y = y / norm_factor;
}

// Vector2D operations
turtlelib::Vector2D & turtlelib::Vector2D::operator+=(const turtlelib::Vector2D & rhs)
{
  this->x += rhs.x;
  this->y += rhs.y;
  return *this;
}

turtlelib::Vector2D & turtlelib::Vector2D::operator-=(const turtlelib::Vector2D & rhs)
{
  this->x -= rhs.x;
  this->y -= rhs.y;
  return *this;
}

turtlelib::Vector2D & turtlelib::Vector2D::operator*=(const double & scalar)
{
  this->x *= scalar;
  this->y -= scalar;
  return *this;
}

turtlelib::Vector2D turtlelib::operator+(Vector2D lhs, const Vector2D & rhs)
{
  lhs += rhs;
  return lhs;
}

turtlelib::Vector2D turtlelib::operator-(Vector2D lhs, const Vector2D & rhs)
{
  lhs -= rhs;
  return lhs;
}

turtlelib::Vector2D turtlelib::operator*(Vector2D lhs, const double & scalar)
{
  lhs *= scalar;
  return lhs;
}

turtlelib::Vector2D turtlelib::operator*(const double & scalar, Vector2D rhs)
{
  rhs *= scalar;
  return rhs;
}

double turtlelib::dot(Vector2D v1, Vector2D v2)
{
  double dot_prod = (v1.x * v2.x) + (v1.y * v2.y);
  return dot_prod;
}

double turtlelib::magnitude(Vector2D vector)
{
  double mag = sqrt(pow(vector.x, 2) + pow(vector.y, 2));
  return mag;
}

double turtlelib::angle(Vector2D v1, Vector2D v2)
{
  double dot_prod = dot(v1, v2);
  double m1 = magnitude(v1);
  double m2 = magnitude(v2);
  double rad = acos(dot_prod / (m1 * m2));
  // rad = normalize_angle(rad);
  return rad;
}


// Vector2D I/O Functions
std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v)
{
  os << '[' << v.x << ' ' << v.y << ']';
  return os;
}


std::istream & turtlelib::operator>>(std::istream & is, Vector2D & v)
{
  bool first_val = true;
  int mag = 1;
  // While loop is used to clear the buffer
  while (is) {
    char c_peek = is.peek();
    if (c_peek == '\n' || c_peek == '\0') {
      is.get();
      break;
    } else if (c_peek == '[' || c_peek == ']') {
      is.get();
      continue;
    } else if (c_peek == '-') {
      mag = -1;
      is.get();
      continue;
    } else if (!isdigit(c_peek)) {
      is.get();
      continue;
    } else {
      if (first_val) {
        is >> v.x;
        v.x *= mag;
        first_val = false;
        mag = 1;
      } else {
        is >> v.y;
        v.y *= mag;
      }
    }
  }
  return is;
}


// Turtlelib Transform 2D Functions
turtlelib::Transform2D::Transform2D()
: theta(0.0), cos_th(1.0), sin_th(0.0), transform{0.0, 0.0}
{}


turtlelib::Transform2D::Transform2D(Vector2D trans)
: theta(0.0), cos_th(1.0), sin_th(0.0), transform{trans.x, trans.y}
{}


turtlelib::Transform2D::Transform2D(double radians)
: theta(radians), cos_th(cos(radians)), sin_th(sin(radians)), transform{0.0, 0.0}
{}


turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
: theta(radians), cos_th(cos(radians)), sin_th(sin(radians)), transform{trans.x, trans.y}
{}


turtlelib::Vector2D turtlelib::Transform2D::operator()(Vector2D v) const
{
  double v_x = (v.x * cos_th) + (v.y * -sin_th) + transform.x;
  double v_y = (v.x * sin_th) + (v.y * cos_th) + transform.y;
  return {v_x, v_y};
}


turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
  Transform2D inv_T;

  inv_T.transform.x = -transform.x * cos_th - transform.y * sin_th;
  inv_T.transform.y = transform.x * sin_th - transform.y * cos_th;
  inv_T.theta = -1 * theta;
  inv_T.cos_th = cos_th;
  inv_T.sin_th = -1 * sin_th;

  return inv_T;
}


turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const Transform2D & rhs)
{
  double temp_x = (cos_th * rhs.translation().x) - (sin_th * rhs.translation().y) + transform.x;
  double temp_y = (sin_th * rhs.translation().x) + (cos_th * rhs.translation().y) + transform.y;

  transform.x = temp_x;
  transform.y = temp_y;
  theta = theta + rhs.theta;
  cos_th = cos(theta);
  sin_th = sin(theta);

  return *this;
}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
  return this->transform;
}


double turtlelib::Transform2D::rotation() const
{
  return this->theta;
}


// Turtlelib Operations for Transform2D
std::ostream & turtlelib::operator<<(std::ostream & os, const Transform2D & tf)
{
  os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " <<
    tf.translation().y;
  return os;
}


std::istream & turtlelib::operator>>(std::istream & is, Transform2D & tf)
{
  // TODO:: Need to check for wrong input errors here, and other methods of insertion
  double theta, x, y;

  int value = 0;
  // While loop is used to clear the buffer
  int mag = 1;
  while (is) {
    char c_peek = is.peek();
    if (c_peek == '\n' || c_peek == '\0') {
      is.get();
      break;
    } else if (c_peek == '[' || c_peek == ']') {
      is.get();
      continue;
    } else if (c_peek == '-') {
      mag = -1;
      is.get();
      continue;
    } else if (!isdigit(c_peek)) {
      is.get();
      continue;
    } else {
      if (value == 0) {
        is >> theta;
        theta *= mag;
        mag = 1;
        value = 1;
      } else if (value == 1) {
        is >> x;
        x *= mag;
        mag = 1;
        value = 2;
      } else {
        is >> y;
        y *= mag;
      }
    }
  }

  Vector2D new_v = Vector2D(x, y);
  tf = Transform2D(new_v, deg2rad(theta));
  return is;
}


turtlelib::Transform2D turtlelib::operator*(Transform2D lhs, const Transform2D & rhs)
{
  lhs *= rhs;
  return lhs;
}


turtlelib::Twist2D turtlelib::Transform2D::operator()(Twist2D tw) const
{
  double new_dx = (tw.dtheta * this->transform.y) + (tw.dx * this->cos_th) - (tw.dy * this->sin_th);
  double new_dy = -(tw.dtheta * this->transform.x) + (tw.dx * this->sin_th) +
    (tw.dy * this->cos_th);
  Twist2D new_tw = Twist2D(tw.dtheta, new_dx, new_dy);
  return new_tw;
}


turtlelib::Transform2D turtlelib::integrate_twist(const Twist2D & tw)
{
  Transform2D Tbb_prime;
  Vector2D v;
  if (tw.dtheta == 0.0) {
    v.x = tw.dx;
    v.y = tw.dy;
    return Transform2D({tw.dx, tw.dy}, 0.0);
  } else {
    v.x = tw.dy / tw.dtheta;
    v.y = -tw.dx / tw.dtheta;

    Transform2D Tsb = Transform2D(v, 0.0);
    Transform2D Tss_prime = Transform2D(Vector2D{0.0, 0.0}, tw.dtheta);
    Transform2D Tbs = Tsb.inv();
    Tbb_prime = (Tbs * Tss_prime) * Tsb;
    return Tbb_prime;
  }
}


// Twist2D Functions
turtlelib::Twist2D::Twist2D()
: dtheta(0.0), dx(0.0), dy(0.0)
{}


turtlelib::Twist2D::Twist2D(double dtheta_in, double dx_in, double dy_in)
: dtheta(dtheta_in), dx(dx_in), dy(dy_in)
{}


// Twist2D I/O Functions
std::ostream & turtlelib::operator<<(std::ostream & os, const Twist2D & tw)
{
  os << '[' << tw.dtheta << ' ' << tw.dx << ' ' << tw.dy << ']';
  return os;
}


std::istream & turtlelib::operator>>(std::istream & is, Twist2D & tw)
{
  int value = 0;
  int mag = 1;
  // While loop is used to clear the buffer
  while (is) {
    char c_peek = is.peek();
    if (c_peek == '\n' || c_peek == '\0') {
      is.get();
      break;
    } else if (c_peek == '[' || c_peek == ']') {
      is.get();
      continue;
    } else if (c_peek == '-') {
      mag = -1;
      is.get();
      continue;
    } else if (!isdigit(c_peek)) {
      is.get();
      continue;
    } else {
      if (value == 0) {
        is >> tw.dtheta;
        tw.dtheta *= -1;
        mag = 1;
        value = 1;
      } else if (value == 1) {
        is >> tw.dx;
        tw.dx *= mag;
        mag = 1;
        value = 2;
      } else {
        is >> tw.dy;
        tw.dy *= mag;
        mag = 1;
      }
    }
  }
  return is;
}
