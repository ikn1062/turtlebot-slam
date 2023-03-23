#include <catch2/catch_test_macros.hpp>
#include "nuslam/circle_library.hpp"
#include "turtlelib/rigid2d.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <armadillo>


using namespace ekf_slam;

TEST_CASE("CircleFit Basic Test 1", "[circleFit]")
{
  landmarkCircle checkCircle = landmarkCircle();
  turtlelib::Vector2D p1(1, 7);
  turtlelib::Vector2D p2(2, 6);
  turtlelib::Vector2D p3(5, 8);
  turtlelib::Vector2D p4(7, 7);
  turtlelib::Vector2D p5(9, 5);
  turtlelib::Vector2D p6(3, 7);
  std::vector<turtlelib::Vector2D> cluster = {p1, p2, p3, p4, p5, p6};

  double x_coord, y_coord, rad;
  bool check = checkCircle.circleFit(cluster, x_coord, y_coord, rad);
  std::cout << "x: " << x_coord << ", y: " << y_coord << ", rad: " << rad << std::endl;
  REQUIRE(check);
  REQUIRE(turtlelib::almost_equal(x_coord, 4.615482, 1e-4));
  REQUIRE(turtlelib::almost_equal(y_coord, 2.807354, 1e-4));
  REQUIRE(turtlelib::almost_equal(rad, 4.8275, 1e-4));
  std::cout << "test 1 pass" << std::endl;
}

TEST_CASE("CircleFit Basic Test 2", "[circleFit]")
{
  landmarkCircle checkCircle = landmarkCircle();
  turtlelib::Vector2D p1(-1, 0);
  turtlelib::Vector2D p2(-0.3, -0.06);
  turtlelib::Vector2D p3(0.3, 0.1);
  turtlelib::Vector2D p4(1, 0);
  std::vector<turtlelib::Vector2D> cluster = {p1, p2, p3, p4};

  double x_coord, y_coord, rad;
  bool check = checkCircle.circleFit(cluster, x_coord, y_coord, rad);
  std::cout << "x: " << x_coord << ", y: " << y_coord << ", rad: " << rad << std::endl;
  REQUIRE(check);
  REQUIRE(turtlelib::almost_equal(x_coord, 0.4908357, 1e-4));
  REQUIRE(turtlelib::almost_equal(y_coord, -22.15212, 1e-4));
  REQUIRE(turtlelib::almost_equal(rad, 22.17979, 1e-4));
  std::cout << "test 2 pass" << std::endl;
}
