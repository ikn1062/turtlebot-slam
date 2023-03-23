#include "nuslam/circle_library.hpp"

namespace ekf_slam
{

landmarkCircle::landmarkCircle()
: dist_threshold(0.5),
  classify_std(0.15),
  classify_mean_min(90),
  classify_mean_max(135)
{

}

landmarkCircle::landmarkCircle(double cluster_thresh, double std, double mean_min, double mean_max)
: dist_threshold(cluster_thresh),
  classify_std(std),
  classify_mean_min(mean_min),
  classify_mean_max(mean_max)
{

}

std::vector<std::vector<turtlelib::Vector2D>> landmarkCircle::landmarkCluster(
  std::vector<float> laser_ranges, double angle_min, double angle_inc, double min_range,
  double max_range)
{
  std::vector<std::vector<turtlelib::Vector2D>> cluster_list;
  std::vector<turtlelib::Vector2D> cluster;

  double angle;
  double current_range, next_range;
  turtlelib::Vector2D laser_point;
  int range_size = laser_ranges.size();

  for (int i = 0; i < range_size; i++) {
    // Ignore all the laser scans out of the min and max range
    if (laser_ranges[i] < min_range || laser_ranges[i] > max_range) {
      cluster_list.push_back(cluster);
      cluster.clear();
      continue;
    }

    angle = angle_min + (angle_inc * i);
    current_range = laser_ranges[i];
    next_range = (i + 1 == range_size) ? 0 : laser_ranges[i + 1];

    // Get the location of the current point in the world frame
    laser_point.x = laser_ranges[i] * cos(angle);
    laser_point.y = laser_ranges[i] * sin(angle);

    // If the 2 points are close together, we cluster them
    if (fabs(current_range - next_range) < dist_threshold) {
      cluster.push_back(laser_point);
      if (i + 1 == range_size) {
        // add all points in the current cluster to the first cluster
        cluster_list[0].insert(cluster_list[0].end(), cluster.begin(), cluster.end());
        // for (unsigned int j = 0; i < cluster.size(); j++) {
        //    cluster_list[0].push_back(cluster[j]);
        // }
      }
    } else {
      // Add current point to the list (close to previous point)
      cluster.push_back(laser_point);

      // Add current cluster to the list of clusters, and clear current list
      cluster_list.push_back(cluster);
      cluster.clear();
    }
  }

  // remove any clusters smaller than 3
  auto it = cluster_list.begin();
  while (it != cluster_list.end()) {
    if (it->size() < 3) {
      it = cluster_list.erase(it);
    } else {
      ++it;
    }
  }

  return cluster_list;
}


bool landmarkCircle::circleFit(
  std::vector<turtlelib::Vector2D> cluster, double & x_coord,
  double & y_coord, double & rad)
{
  double cluster_size = cluster.size();

  // Get the Centroid
  turtlelib::Vector2D centroid(0, 0);
  for (unsigned int i = 0; i < cluster_size; i++) {
    //std::cout << cluster[i] << std::endl;
    centroid.x += cluster[i].x;
    centroid.y += cluster[i].y;
  }
  centroid.x = centroid.x / cluster_size;
  centroid.y = centroid.y / cluster_size;

  // Shift coordinates so centroid is at the origin
  for (unsigned int i = 0; i < cluster_size; i++) {
    cluster[i].x = cluster[i].x - centroid.x;
    cluster[i].y = cluster[i].y - centroid.y;
  }

  // Data Matrix and mean of z
  double z = 0;
  double z_mean = 0;
  arma::mat Z_mat(cluster.size(), 4, arma::fill::ones);
  for (unsigned int i = 0; i < cluster_size; i++) {
    z = (pow(cluster[i].x, 2) + pow(cluster[i].y, 2));
    z_mean += z;

    Z_mat(i, 0) = z;
    Z_mat(i, 1) = cluster[i].x;
    Z_mat(i, 2) = cluster[i].y;
  }
  z_mean = z_mean / cluster.size();
  // Form the moment matrix
  //arma::mat M_mat = (Z_mat.t() * Z_mat) % (1/cluster_size);

  // Constraint Matrix
  arma::mat H_mat(4, 4, arma::fill::eye);
  H_mat(0, 0) = 2.0 * z_mean;
  H_mat(0, 3) = 2.0;
  H_mat(3, 0) = 2.0;
  H_mat(3, 3) = 0.0;

  // Inv Constraint Matrix
  arma::mat Hinv_mat(4, 4, arma::fill::eye);
  Hinv_mat(0, 0) = 0.0;
  Hinv_mat(0, 3) = 0.5;
  Hinv_mat(3, 0) = 0.5;
  Hinv_mat(3, 3) = -2.0 * z_mean;

  // Singular Value Decomposition of Z -> func taken from armadillo website
  arma::mat U_mat;
  arma::vec s_vec;
  arma::mat V_mat;
  arma::svd(U_mat, s_vec, V_mat, Z_mat);

  // SVD failed
  if (s_vec.size() < 4) {
    return false;
  }

  // Check for smallest singular value
  arma::vec A_vec;
  if (s_vec(3) < 1e-12) {
    A_vec = V_mat.col(3);
  } else {
    arma::mat Y_mat = V_mat * arma::diagmat(s_vec) * V_mat.t();
    arma::mat Q_mat = Y_mat * Hinv_mat * Y_mat;

    // Find Eigen Vectors and Values of Q_mat
    arma::cx_vec eigval;
    arma::cx_mat eigvec;
    arma::eig_gen(eigval, eigvec, Q_mat);

    // arma::mat eigvec;
    // arma::eig_sym(eigval, eigvec, Q_mat);

    // Find A* - eigenvector corresponding to smallest positive eigenvalue of Q
    int idx = 0;
    double eig_min = 2e31;

    for (unsigned int i = 0; i < eigval.size(); i++) {
      if (eigval(i).real() > 0 && eigval(i).real() < eig_min) {
        eig_min = eigval(i).real();
        idx = i;
        //Astar_vec = eigvec.col(i);
      }
    }
    arma::vec Astar_vec{eigvec(0, idx).real(), eigvec(1, idx).real(), eigvec(2, idx).real(), eigvec(
        3, idx).real()};
    A_vec = arma::solve(Y_mat, Astar_vec);
  }

  // solve equation of the circle
  double a = -A_vec(1) / (2 * A_vec(0));
  double b = -A_vec(2) / (2 * A_vec(0));
  double R2 = (pow(A_vec(1), 2) + pow(A_vec(2), 2) - 4 * A_vec(0) * A_vec(3)) / (4 * pow(
      A_vec(
        0), 2));

  // Get x, y, and r and return True
  x_coord = a + centroid.x;
  y_coord = b + centroid.y;
  rad = sqrt(R2);
  return true;
}

bool landmarkCircle::classifyCircle(std::vector<turtlelib::Vector2D> cluster)
{
  turtlelib::Vector2D point, p1, p2;
  double mean, std_dev;
  std::vector<double> angle_list;
  unsigned int cluster_size = cluster.size();

  // Get points p1 and p2
  p1 = cluster[0];
  p2 = cluster[cluster_size - 1];

  double arc_num, arc_dem, angle;
  for (unsigned int i = 1; i < cluster_size - 1; i++) {
    point = cluster[i];

    // Equations taken from paper
    arc_num = p1.y * (point.x - p2.x) + point.y * (p2.x - p1.x) + p2.y * (p1.x - point.x);
    arc_dem = (p1.x - point.x) * (point.x - p2.x) + (p1.y - point.y) * (point.y - p2.y);
    angle = turtlelib::rad2deg(atan2(arc_num, arc_dem));

    mean += angle / (cluster_size - 2);

    // Add to list of angles -> used later for std
    angle_list.push_back(angle);
  }

  for (unsigned int i = 0; i < angle_list.size(); i++) {
    std_dev += pow(angle_list[i] - mean, 2);
  }
  std_dev = sqrt(std_dev / angle_list.size());

  if (std_dev <= classify_std && mean >= classify_mean_min && mean <= classify_mean_max) {
    return true;
  }
  return false;
}

}
