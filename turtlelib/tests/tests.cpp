#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <iostream>


using namespace turtlelib;

TEST_CASE("Transform2D String Stream I/O with labels", "[transform]")   // Ishaan Narain
{
  Transform2D transform = Transform2D();

  std::stringstream sstr;
  sstr << "deg: 45 x: 2 y: 2";
  sstr >> transform;

  REQUIRE(almost_equal(transform.rotation(), deg2rad(45), 0.0001) );
  REQUIRE(almost_equal(transform.translation().x, 2.0, 0.0001) );
  REQUIRE(almost_equal(transform.translation().y, 2.0, 0.0001) );

  std::ostringstream os;
  os << transform;
  REQUIRE(os.str() == "deg: 45 x: 2 y: 2");
}


TEST_CASE("Transform2D String Stream I/O without labels", "[transform]")   // Ishaan Narain
{
  Transform2D transform = Transform2D();

  std::stringstream sstr;
  sstr << "135 -2 2";
  sstr >> transform;

  REQUIRE(almost_equal(transform.rotation(), deg2rad(135), 0.0001) );
  REQUIRE(almost_equal(transform.translation().x, -2.0, 0.0001) );
  REQUIRE(almost_equal(transform.translation().y, 2.0, 0.0001) );

  std::ostringstream os;
  os << transform;
  REQUIRE(os.str() == "deg: 135 x: -2 y: 2");
}


TEST_CASE("Transform2D String Stream I/O without labels - 2", "[transform]")   // Ishaan Narain
{
  Transform2D transform_1 = Transform2D();

  std::stringstream sstr_1;
  sstr_1 << "45 -1 7";
  sstr_1 >> transform_1;

  REQUIRE(almost_equal(transform_1.rotation(), deg2rad(45), 0.0001) );
  REQUIRE(almost_equal(transform_1.translation().x, -1.0, 0.0001) );
  REQUIRE(almost_equal(transform_1.translation().y, 7.0, 0.0001) );

  std::ostringstream os_1;
  os_1 << transform_1;
  REQUIRE(os_1.str() == "deg: 45 x: -1 y: 7");
}


TEST_CASE("Transform2D Inverse", "[transform]")   // Ishaan Narain
{
  Transform2D transform = Transform2D();

  std::stringstream sstr;
  sstr << "deg: 45 x: -2 y: 2";
  sstr >> transform;

  Transform2D transform_inv = transform.inv();

  REQUIRE(almost_equal(transform_inv.rotation(), -0.785398163, 0.1) );
  REQUIRE(almost_equal(transform_inv.translation().x, 0.0, 0.0001) );
  REQUIRE(almost_equal(transform_inv.translation().y, -2.828427124, 0.0001) );

}


TEST_CASE("Transform2D * operator", "[transform]")   // Ishaan Narain
{
  Transform2D tf1 = {{-2, 2}, 45};
  Vector2D v = {1, 1};

  Vector2D v2 = tf1(v);

  REQUIRE(almost_equal(v2.x, -2.32558, 1.0e-4));
  REQUIRE(almost_equal(v2.y, 3.37623, 1.0e-4));
}


TEST_CASE("Test cases for normalizing angles", "[normalize_angle]")   // Ishaan Narain
{
  double case_pi = normalize_angle(PI);
  double case_neg_pi = normalize_angle(-1.0 * PI);
  double case_zero = normalize_angle(0.0 * PI);
  double case_neg_dfour = normalize_angle(-1.0 * (PI / 4.0));
  double case_nthree_dtwo = normalize_angle(3.0 * PI / 2.0);
  double case_neg_nfive_dtwo = normalize_angle(-5.0 * PI / 2.0);

  REQUIRE(almost_equal(case_pi, 3.414213562, 1.0e-6));
  REQUIRE(almost_equal(case_neg_pi, 3.414213562, 1.0e-6));
  REQUIRE(almost_equal(case_zero, 0.0, 1.0e-6));
  REQUIRE(almost_equal(case_neg_dfour, -0.78539816339, 1.0e-6));
  REQUIRE(almost_equal(case_nthree_dtwo, -1.57079632679, 1.0e-6));
  REQUIRE(almost_equal(case_neg_nfive_dtwo, -1.57079632679, 1.0e-6));
}

TEST_CASE("Integrating Twist - pure translation", "[integrate_twist]")   // Ishaan Narain
{
  Twist2D input_tw;
  input_tw.dtheta = 0.0;
  input_tw.dx = 2.0;
  input_tw.dy = 3.0;
  Transform2D Tbb_prime = integrate_twist(input_tw);

  REQUIRE(almost_equal(Tbb_prime.translation().x, 2.0, 1.0e-6));
  REQUIRE(almost_equal(Tbb_prime.translation().y, 3.0, 1.0e-6));
  REQUIRE(almost_equal(Tbb_prime.rotation(), 0.0, 1.0e-6));
}

TEST_CASE("Integrating Twist - pure rotation", "[integrate_twist]")   // Ishaan Narain
{
  Twist2D input_tw;
  input_tw.dtheta = PI / 3;
  input_tw.dx = 0.0;
  input_tw.dy = 0.0;
  Transform2D Tbb_prime = integrate_twist(input_tw);

  REQUIRE(almost_equal(Tbb_prime.translation().x, 0.0, 1.0e-6));
  REQUIRE(almost_equal(Tbb_prime.translation().y, 0.0, 1.0e-6));
  REQUIRE(almost_equal(Tbb_prime.rotation(), 1.0471975512, 1.0e-6));
}

TEST_CASE("Integrating Twist - simultaneous translation and rotation", "[integrate_twist]")   // Ishaan Narain
{
  Twist2D input_tw;
  input_tw.dtheta = PI / 3;
  input_tw.dx = 2.0;
  input_tw.dy = 3.0;
  Transform2D Tbb_prime = integrate_twist(input_tw);

  REQUIRE(almost_equal(Tbb_prime.translation().x, 0.221592, 1.0e-4));
  REQUIRE(almost_equal(Tbb_prime.translation().y, 3.43591, 1.0e-4));
  REQUIRE(almost_equal(Tbb_prime.rotation(), 1.0471975512, 1.0e-4));
}

TEST_CASE("DiffDrive - FK Robot Drives Forward", "[forwardKinematics]")   // Ishaan Narain
{
  DiffDrive diff_drive = DiffDrive();
  WheelAngle new_phi;

  new_phi.left = PI / 4;
  new_phi.right = PI / 4;

  Config output_config = diff_drive.forwardKinematics(new_phi);

  REQUIRE(almost_equal(output_config.theta, 0.0, 1.0e-4));
  REQUIRE(almost_equal(output_config.x, 0.033 * PI / 4, 1.0e-4));
  REQUIRE(almost_equal(output_config.y, 0.0, 1.0e-4));
}

TEST_CASE("DiffDrive - IK Robot Drives Forward", "[inverseKinematics]")   // Ishaan Narain
{
  DiffDrive diff_drive = DiffDrive();
  Twist2D twist;

  twist.dx = 2;
  twist.dy = 0;
  twist.dtheta = 0;

  WheelVelocity output_vel = diff_drive.inverseKinematics(twist);

  REQUIRE(almost_equal(output_vel.left, 60.60606, 1.0e-4));
  REQUIRE(almost_equal(output_vel.right, 60.60606, 1.0e-4));
}

TEST_CASE("DiffDrive - FK Robot Drives rotation", "[forwardKinematics]")   // Ishaan Narain
{
  DiffDrive diff_drive = DiffDrive();
  WheelAngle new_phi;

  new_phi.left = 3.807991;
  new_phi.right = -3.807991;

  Config output_config = diff_drive.forwardKinematics(new_phi);

  REQUIRE(almost_equal(output_config.theta, (-1 * PI / 2.0), 1.0e-4));
  REQUIRE(almost_equal(output_config.x, 0.0, 1.0e-4));
  REQUIRE(almost_equal(output_config.y, 0.0, 1.0e-4));
}

TEST_CASE("DiffDrive - IK Robot Drives rotation", "[inverseKinematics]")   // Ishaan Narain
{
  DiffDrive diff_drive = DiffDrive();
  Twist2D twist;

  twist.dx = 0;
  twist.dy = 0;
  twist.dtheta = PI / 2.0;

  WheelVelocity output_vel = diff_drive.inverseKinematics(twist);

  REQUIRE(almost_equal(output_vel.left, -3.807991, 1.0e-4));
  REQUIRE(almost_equal(output_vel.right, 3.807991, 1.0e-4));
}

TEST_CASE("DiffDrive - FK Robot Drives Arc", "[forwardKinematics]")   // Ishaan Narain
{
  //TODO: FINISH TEST
  DiffDrive diff_drive = DiffDrive();
  WheelAngle new_phi;

  new_phi.left = 19.99195;
  new_phi.right = 27.60793;

  Config output_config = diff_drive.forwardKinematics(new_phi);

  REQUIRE(almost_equal(output_config.theta, PI / 2.0, 1.0e-4));
  REQUIRE(almost_equal(output_config.x, 0.5, 1.0e-4));
  REQUIRE(almost_equal(output_config.y, 0.5, 1.0e-4));
}

TEST_CASE("DiffDrive - IK Robot Drives Arc", "[inverseKinematics]")   // Ishaan Narain
{
  DiffDrive diff_drive = DiffDrive();
  Twist2D twist;

  twist.dx = PI / 4.0;
  twist.dy = 0;
  twist.dtheta = PI / 2.0;

  WheelVelocity output_vel = diff_drive.inverseKinematics(twist);

  REQUIRE(almost_equal(output_vel.left, 19.99195, 1.0e-4));
  REQUIRE(almost_equal(output_vel.right, 27.60793, 1.0e-4));
}

TEST_CASE("Inverse kinematics Impossible to follow test", "[DiffDrive]")
{
  DiffDrive diff_drive = DiffDrive();
  Twist2D twist;

  twist.dx = 1;
  twist.dy = 1;
  twist.dtheta = PI / 4;

  WheelVelocity output_vel;

  CHECK_THROWS(output_vel = diff_drive.inverseKinematics(twist));
}

/////////////////////////////////////////////////////////////////////////////////////////////
///                        All tests below are taken from MSR students                    ///
/////////////////////////////////////////////////////////////////////////////////////////////


TEST_CASE("Rotation and Translation - 0", "[transform]") {   // Hughes, Katie
  float my_x = 2;
  float my_y = 3;
  float my_ang = PI;
  Transform2D Ttest = {Vector2D{my_x, my_y}, my_ang};
  REQUIRE(1 == 1);
  REQUIRE(Ttest.rotation() == my_ang);
  REQUIRE(Ttest.translation().x == my_x);
  REQUIRE(Ttest.translation().y == my_y);
}


TEST_CASE("Inverse 1", "[transform]") {   // Hughes, Katie
  float my_x = 0.;
  float my_y = 1.;
  float my_ang = PI / 2;
  Transform2D Ttest = {Vector2D{my_x, my_y}, my_ang};
  Transform2D Ttest_inv = Ttest.inv();
  REQUIRE( (Ttest.inv()).rotation() == -my_ang);
  REQUIRE(almost_equal(Ttest_inv.translation().x, -1.0, 1.0e-5) );
  REQUIRE(almost_equal(Ttest_inv.translation().y, 0.0, 1.0e-5) );
  // REQUIRE( Ttest_inv.translation().y == 0.0 );
}


TEST_CASE("Operator () for Vector2D", "[transform]") {   // Yin, Hang
  float my_x = 2;
  float my_y = 3;
  float my_ang = PI;
  Transform2D Ttest = {Vector2D{my_x, my_y}, my_ang};
  Vector2D v = {2, 2};
  Vector2D result = Ttest(v);
  REQUIRE(almost_equal(result.x, 0.0, 1.0e-5) );
  REQUIRE(almost_equal(result.y, 1.0, 1.0e-5) );
}


TEST_CASE("Operator () for Twist2D", "[transform]") {   // Yin, Hang
  float my_x = 2;
  float my_y = 3;
  float my_ang = PI;
  Transform2D Ttest = {Vector2D{my_x, my_y}, my_ang};
  Twist2D twist = {2, 2, 2};
  Twist2D result = Ttest(twist);
  REQUIRE(almost_equal(result.dtheta, 2.0, 1.0e-5) );
  REQUIRE(almost_equal(result.dx, 4.0, 1.0e-5) );
  REQUIRE(almost_equal(result.dy, -6.0, 1.0e-5) );
}


TEST_CASE("Rotation() - 1", "[transform]") { // Marno, Nel
  double test_rot = 30.0;
  Transform2D T_test{test_rot};
  REQUIRE(T_test.rotation() == test_rot);
}


TEST_CASE("Translation()", "[transform]") { // Marno, Nel
  double test_x = 4.20;
  double test_y = 6.9;
  Transform2D T_test{{test_x, test_y}};
  REQUIRE(T_test.translation().x == test_x);
  REQUIRE(T_test.translation().y == test_y);
}


TEST_CASE("operator()(Vector2D v)", "[transform]") // Marno, Nel
{
  double test_rot = PI / 2.0;
  double test_x = 0.0;
  double test_y = 1.0;
  Transform2D T_ab{{test_x, test_y}, test_rot};
  Vector2D v_b{1, 1};
  Vector2D v_a = T_ab(v_b);
  REQUIRE(almost_equal(v_a.x, -1.0));
  REQUIRE(almost_equal(v_a.y, 2.0));
}


TEST_CASE("operator()(Twist2D t)", "[transform]") // Marno, Nel
{
  double test_rot = PI / 2.0;
  double test_x = 0.0;
  double test_y = 1.0;
  Transform2D T_ab{{test_x, test_y}, test_rot};
  Twist2D V_b{1, 1, 1};
  Twist2D V_a = T_ab(V_b);
  REQUIRE(almost_equal(V_a.dtheta, 1.0));
  REQUIRE(almost_equal(V_a.dx, 0.0));
  REQUIRE(almost_equal(V_a.dy, 1.0));
}


TEST_CASE("Rotation", "[transform]") {  // Rintaroh, Shima
  double theta = PI / 4;
  Transform2D trans = Transform2D(theta);
  REQUIRE(trans.rotation() == theta);
}


TEST_CASE("Translation", "[transform]") {  // Rintaroh, Shima
  Vector2D v;
  v.x = 5.5;
  v.y = 7.2;
  Transform2D trans = Transform2D(v);
  REQUIRE(trans.translation().x == v.x);
  REQUIRE(trans.translation().y == v.y);
}


TEST_CASE(" Vector2D Operator()", "[transform]") {  // Rintaroh, Shima
  Vector2D v;
  v.x = 5;
  v.y = 6;
  double theta = PI / 4;
  Transform2D trans = Transform2D(v, theta);
  Vector2D vec = {3, 4};
  Vector2D vector = trans(vec);
  REQUIRE(almost_equal(vector.x, 4.29289, 1.0e-4));
  REQUIRE(almost_equal(vector.y, 10.9497, 1.0e-4));
}


TEST_CASE("Twist2D Operator()", "[transform]") { // Rintaroh, Shima
  Vector2D v;
  v.x = 5;
  v.y = 6;
  double theta = PI / 4;
  Transform2D trans = Transform2D(v, theta);
  Twist2D t = {3, 4, 5};
  Twist2D twist = trans(t);
  REQUIRE(almost_equal(twist.dtheta, 3));
  REQUIRE(almost_equal(twist.dx, 17.2929, 1.0e-5));
  REQUIRE(almost_equal(twist.dy, -8.63604, 1.0e-5));
}

TEST_CASE("Stream insertion operator <<", "[transform]")   // Ava, Zahedi
{
  Vector2D vec;
  vec.x = 1.0;
  vec.y = 3.4;
  double phi = 0.0;
  Transform2D tf = Transform2D(vec, phi);
  std::string str = "deg: 0 x: 1 y: 3.4";
  std::stringstream sstr;
  sstr << tf;
  REQUIRE(sstr.str() == str);
}


TEST_CASE("Stream extraction operator >>", "[transform]")   // Ava, Zahedi
{
  Transform2D tf = Transform2D();
  std::stringstream sstr;
  sstr << "deg: 90 x: 1 y: 3.4";
  sstr >> tf;
  REQUIRE(almost_equal(tf.rotation(), deg2rad(90), 0.00001) );
  REQUIRE(almost_equal(tf.translation().x, 1.0, 0.00001) );
  REQUIRE(almost_equal(tf.translation().y, 3.4, 0.00001) );
}


TEST_CASE("operator *=", "[transform]") {    //Megan, Sindelar
  Vector2D trans_ab = {1, 2};
  double rotate_ab = 0;
  Transform2D T_ab_1 = {trans_ab, rotate_ab};       //T_ab's are all the same,
  Transform2D T_ab_2 = {trans_ab, rotate_ab};       //but, need different vars
  Transform2D T_ab_3 = {trans_ab, rotate_ab};       //b/c getting overwritten otherwise
  Vector2D trans_bc = {3, 4};
  double rotate_bc = PI / 2;
  Transform2D T_bc = {trans_bc, rotate_bc};


  REQUIRE(almost_equal((T_ab_1 *= T_bc).translation().x, 4.0));
  REQUIRE(almost_equal((T_ab_2 *= T_bc).translation().y, 6.0));
  REQUIRE(almost_equal((T_ab_3 *= T_bc).rotation(), (PI / 2)));
}


TEST_CASE("Input stream operator >>, [transform]") {     //Megan Sindelar
  std::stringstream ss("deg: 0 x: 4 y: 5");             //create an object of string stream
  std::streambuf * old_cin_streambuf = std::cin.rdbuf();   //pointer that holds the input
  std::cin.rdbuf(ss.rdbuf());                      //redirect input stream to a stringstream
  std::string input1, input2, input3, input4, input5, input6;      // variable to store input
  std::cin >> input1;                               //user input (before whitespace)
  std::cin >> input2;                               //user input (before whitespace)
  std::cin >> input3;                               //user input (before whitespace)
  std::cin >> input4;                               //user input (before whitespace)
  std::cin >> input5;                               //user input (before whitespace)
  std::cin >> input6;                               //user input (before whitespace)
  std::cin.rdbuf(old_cin_streambuf);               //restore the original stream buffer


  std::string input;
  input = input1 + " " + input2 + " " + input3 + " " + input4 + " " + input5 + " " + input6;


  REQUIRE(input == "deg: 0 x: 4 y: 5");
}


TEST_CASE("Rotation and Translation - 1", "[transform]") {   //Megan Sindelar
  Vector2D trans = {1, 2};
  double rotate = 90;
  Transform2D T = {trans, rotate};
  REQUIRE(almost_equal(T.translation().x, trans.x));
  REQUIRE(almost_equal(T.translation().y, trans.y));
  REQUIRE(almost_equal(T.rotation(), rotate));
}


TEST_CASE("operator() for a vector", "[transform]") { // Liz, Metzger
  Vector2D answer = {1.3397643622, 0.910905899};
  Vector2D test_vec = {-1.0, 0.5};
  Vector2D trans_vec = {1, 0};
  Transform2D test_trans = {trans_vec, 30};
  Vector2D result = test_trans(test_vec);
  REQUIRE(almost_equal(result.x, answer.x));
  REQUIRE(almost_equal(result.y, answer.y));
}


TEST_CASE("Operator () for Twist2D - 1", "[transform]") { // Dilan Wijesinghe
  float w = 3.0;
  float x = 4.0;
  float y = 5.0;
  float theta = PI / 2.0;
  Transform2D TfTest = {Vector2D {x, y}, theta};
  Twist2D TwistTest = {w, x, y};
  Twist2D res = TfTest(TwistTest);


  REQUIRE(almost_equal(res.dtheta, 3.0, 1.0e-5) );
  REQUIRE(almost_equal(res.dx, 10.0, 1.0e-5) );
  REQUIRE(almost_equal(res.dy, -8.0, 1.0e-5) );
}


TEST_CASE("Inverse() for Twist2D", "[transform]") { // Dilan Wijesinghe
  float x = 3.0;
  float y = 4.0;
  float theta = PI / 4.0;
  Transform2D TfTest = {Vector2D {x, y}, theta};
  Transform2D res = TfTest.inv();

  REQUIRE(almost_equal(res.rotation(), -theta, 1.0e-5) );
  REQUIRE(almost_equal(res.translation().x, -4.94975, 1.0e-5) );
  REQUIRE(almost_equal(res.translation().y, -0.707107, 1.0e-5) );
}


TEST_CASE("Operator *= for Transform2D - 2", "[transform]") { // Dilan Wijesinghe
  float x = 3.0;
  float y = 4.0;
  float theta = PI / 4.0;
  Transform2D TfTest = {Vector2D {x, y}, theta};
  TfTest *= TfTest;


  REQUIRE(almost_equal(TfTest.rotation(), 1.5708, 1.0e-5) );
  REQUIRE(almost_equal(TfTest.translation().x, 2.29289, 1.0e-5) );
  REQUIRE(almost_equal(TfTest.translation().y, 8.44975, 1.0e-5) );
}


TEST_CASE("Operator>> for Transform2D", "[transform]") { // Dilan Wijesinghe
  std::stringstream input;
  Transform2D TfTest;

  input << "90 2 3";
  input >> TfTest;


  REQUIRE(almost_equal(TfTest.rotation(), deg2rad(90), 1.0e-5) );
  REQUIRE(almost_equal(TfTest.translation().x, 2.0, 1.0e-5) );
  REQUIRE(almost_equal(TfTest.translation().y, 3.0, 1.0e-5) );
}


TEST_CASE("inv() - 1", "[transform]") { // Liz, Metzger
  Vector2D trans_vec = {3, 4};
  Transform2D test_trans = {trans_vec, 0.523599};
  Transform2D test_inv = test_trans.inv();
  REQUIRE(almost_equal(test_inv.translation().x, -4.598080762));
  REQUIRE(almost_equal(test_inv.translation().y, -1.964101615));
  REQUIRE(almost_equal(test_inv.rotation(), -0.523599));
}

TEST_CASE("operator *= for Transform2D - 1", "[transform]") { //Liz Metzger
  Vector2D trans_vec = {3, 4};
  Transform2D test_trans = {trans_vec, 0.523599};
  test_trans *= test_trans;
// std::cout << test_trans.translation().x << " " << test_trans.translation().y << " " << test_trans.rotation();
  REQUIRE(almost_equal(test_trans.translation().x, 3.59808));
  REQUIRE(almost_equal(test_trans.translation().y, 9.26314));
  REQUIRE(almost_equal(test_trans.rotation(), 1.0472));
}


TEST_CASE("operator()", "[Transform2D]") {   // Shantao Cao
  Transform2D trantest;
  Vector2D testvec;
  testvec.x = 0;
  testvec.y = 1;
  double testdeg;
  testdeg = PI / 2;
  trantest = Transform2D(testvec, testdeg);
  Vector2D testvec2 = {1, 1}, testans;
  testans = trantest(testvec2);
  // std::cout<<trantest;
  // std::cout<<testvec2;
  // std::cout<<testans;
  REQUIRE(almost_equal(testans.x, -1, 1.0e-5));
  REQUIRE(almost_equal(testans.y, 2, 1.0e-5));
}


TEST_CASE("inv() - 2", "[Transform2D]") {   // Shantao Cao
  Transform2D trantest, tranans;
  Vector2D testvec;
  testvec.x = 0;
  testvec.y = 1;
  double testdeg;
  testdeg = PI / 2;
  trantest = Transform2D(testvec, testdeg);
  tranans = trantest.inv();
  REQUIRE(almost_equal(tranans.rotation(), -testdeg, 1.0e-5));
  REQUIRE(almost_equal(tranans.translation().x, -1, 1.0e-5));
  REQUIRE(almost_equal(tranans.translation().y, 0, 1.0e-5));
}


TEST_CASE("operator*=", "[Transform2D]") {   // Shantao Cao
  Transform2D trantest1, trantest2, tranans;
  Vector2D testvec1, testvec2;
  testvec1.x = 0;
  testvec1.y = 1;
  testvec2.x = 1;
  testvec2.y = 0;
  double testdeg;
  testdeg = PI / 2;
  trantest1 = Transform2D(testvec1, testdeg);
  trantest2 = Transform2D(testvec2, testdeg);
  tranans = trantest1 * trantest2;
  REQUIRE(almost_equal(tranans.rotation(), 2 * testdeg, 1.0e-5));
  REQUIRE(almost_equal(tranans.translation().x, 0, 1.0e-5));
  REQUIRE(almost_equal(tranans.translation().y, 2, 1.0e-5));
}


TEST_CASE("translation and rotation", "[Transform2D]") {   // Shantao Cao
  Transform2D trantest1;
  Vector2D testvec1;
  testvec1.x = 0;
  testvec1.y = 1;
  double testdeg;
  testdeg = PI / 2;
  trantest1 = Transform2D(testvec1, testdeg);
  REQUIRE(almost_equal(trantest1.rotation(), testdeg, 1.0e-5));
  REQUIRE(almost_equal(trantest1.translation().x, testvec1.x, 1.0e-5));
  REQUIRE(almost_equal(trantest1.translation().y, testvec1.y, 1.0e-5));
}


TEST_CASE("Rotation() - 2", "[transform]") { //Ghosh, Ritika
  double test_ang = PI / 4;
  Vector2D test_vec = {3.5, 2.5};
  Transform2D Ttest1, Ttest2(test_ang), Ttest3(test_vec), Ttest4(test_vec, test_ang);
  REQUIRE(Ttest1.rotation() == 0.0);
  REQUIRE(Ttest2.rotation() == test_ang);
  REQUIRE(Ttest3.rotation() == 0.0);
  REQUIRE(Ttest4.rotation() == test_ang);
}


TEST_CASE("twistconversion()", "[transform]") { //Ghosh, Ritika
  Transform2D Tab = {Vector2D{0, 1}, PI / 2};
  Transform2D Tbc = {Vector2D{1, 0}, PI / 2};
  Twist2D Vb = {1, 1, 1};
  Twist2D Va = Tab(Vb);
  Twist2D Vc = (Tbc.inv())(Vb);
  REQUIRE(almost_equal(Va.dtheta, 1.0, 1.0e-5));
  REQUIRE(almost_equal(Va.dx, 0.0, 1.0e-5));
  REQUIRE(almost_equal(Va.dy, 1.0, 1.0e-5));
  REQUIRE(almost_equal(Vc.dtheta, 1.0, 1.0e-5));
  REQUIRE(almost_equal(Vc.dx, 2.0, 1.0e-5));
  REQUIRE(almost_equal(Vc.dy, -1.0, 1.0e-5));
}


TEST_CASE("operator <<", "[transform]") { //Ghosh, Ritika
  Transform2D Ttest = {Vector2D{0, 1}, PI / 2};
  std::ostringstream os;
  os << Ttest;
  REQUIRE(os.str() == "deg: 90 x: 0 y: 1");
}


TEST_CASE("operator >>", "[transform]") { //Ghosh, Ritika
  Transform2D Ttest1, Ttest2;
  std::istringstream is1("deg: 90 x: 0 y:1"), is2("45 1 0");
  is1 >> Ttest1;
  is2 >> Ttest2;
  REQUIRE(almost_equal(Ttest1.rotation(), PI / 2));
  REQUIRE(almost_equal(Ttest2.rotation(), PI / 4));
}


TEST_CASE("inverse 2", "[transform]")   // James Oubre
{
  Transform2D tf = {{0, 1}, 90};
  Transform2D ground_truth = {{-1, 0}, -90};
  Transform2D inv_tf = tf.inv();


  Vector2D inv_tran = inv_tf.translation();
  double inv_rot = inv_tf.rotation();


  Vector2D out_tran = ground_truth.translation();
  double out_rot = ground_truth.rotation();


  REQUIRE(almost_equal(inv_tran.x, out_tran.x, 1.0e-6));
  REQUIRE(almost_equal(inv_tran.y, out_tran.y, 1.0e-6));
  REQUIRE(almost_equal(inv_rot, out_rot, 1.0e-6));
}
