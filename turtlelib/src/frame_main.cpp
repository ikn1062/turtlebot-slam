#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <string>
#include <climits>


int main()
{
  // Clear CIN buffer
  std::cin.clear();
  std::cin.ignore(INT_MAX, '\n');

  // Transform Outputs
  turtlelib::Transform2D trans_ab = turtlelib::Transform2D();
  std::cout << "Enter transform T_{a,b}:" << std::endl;
  std::cin >> trans_ab;

  turtlelib::Transform2D trans_bc = turtlelib::Transform2D();
  std::cout << "Enter transform T_{b,c}:" << std::endl;
  std::cin >> trans_bc;

  std::cout << "T_{a,b}: " << trans_ab << std::endl;
  turtlelib::Transform2D trans_ab_inv = trans_ab.inv();
  std::cout << "T_{b,a}: " << trans_ab_inv << std::endl;

  std::cout << "T_{b,c}: " << trans_bc << std::endl;
  turtlelib::Transform2D trans_bc_inv = trans_bc.inv();
  std::cout << "T_{c,b}: " << trans_bc_inv << std::endl;

  turtlelib::Transform2D trans_ac = turtlelib::Transform2D(
    trans_ab.translation(),
    trans_ab.rotation());
  trans_ac *= trans_bc;
  std::cout << "T_{a,c}: " << trans_ac << std::endl;
  std::cout << "T_{c,a}: " << trans_ac.inv() << std::endl;


  // Vector Outputs
  turtlelib::Vector2D vb = turtlelib::Vector2D();
  std::cout << "Enter vector v_b:" << std::endl;
  std::cin >> vb;


  turtlelib::Vector2D vb_copy = turtlelib::Vector2D(vb.x, vb.y);
  vb_copy.normalise();
  std::cout << "v_bhat: " << vb_copy << std::endl;
  std::cout << "v_a: " << trans_ab(vb) << std::endl;
  std::cout << "v_b: " << vb << std::endl;
  std::cout << "v_c: " << trans_bc_inv(vb) << std::endl;


  turtlelib::Twist2D tw_b = turtlelib::Twist2D();
  std::cout << "Enter twist V_b:" << std::endl;
  std::cin >> tw_b;

  std::cout << "V_a " << trans_ab(tw_b) << std::endl;
  std::cout << "tw_b " << tw_b << std::endl;
  std::cout << "V_a " << trans_bc_inv(tw_b) << std::endl;

  return 0;
}
