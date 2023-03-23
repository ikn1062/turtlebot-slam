#ifndef CIRCLE_LIB_INCLUDE_GUARD_HPP
#define CIRCLE_LIB_INCLUDE_GUARD_HPP

/// \file
/// \brief Library for Laser Scan Clustering, Circular Fitting, and Circular Classification

#include <turtlelib/rigid2d.hpp>
#include <armadillo>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>


namespace ekf_slam
{
/// \brief A class for unsupervised and unsupervised cirlce clustering and classification algorithms
class landmarkCircle
{
public:
  /// \brief Initializes the Landmark Circle Object
  landmarkCircle();

  /// \brief Initializes the Landmark Circle Object with thresholds
  /// \param cluster_thresh Distance threshold between two points when clustering
  /// \param std STD threshold for classifiying a circle
  /// \param mean_min Mean min threshold for classifiying a circle
  /// \param mean_max Mean max threshold for classifiying a circle
  landmarkCircle(double cluster_thresh, double std, double mean_min, double mean_max);

  /// \brief Initializes the Extended Kalman Filter Object with number of objects, Q, and R
  /// \param laser_ranges Laser range scan
  /// \param angle_min Minimum Laser angle
  /// \param angle_max Maximum laser angle
  /// \param angle_inc Laser Increment Angle
  /// \param min_range Minimum range of laser
  /// \param max_range Maximum range of laser
  /// \return List of Clusters
  std::vector<std::vector<turtlelib::Vector2D>> landmarkCluster(
    std::vector<float> laser_ranges,
    double angle_min, double angle_inc,
    double min_range, double max_range);

  /// \brief Initializes the Extended Kalman Filter Object with number of objects, Q, and R
  /// \param cluster List of laser points in a cluster
  /// \param x_coord Reference to X coordinate
  /// \param y_coord Reference to Y coordinate
  /// \param rad Reference to radius
  /// \return True or false if able to return a radius
  bool circleFit(
    std::vector<turtlelib::Vector2D> cluster, double & x_coord, double & y_coord,
    double & rad);

  /// \brief Determined if a cluster of points is a circle or not circle - avoids false postives
  ///        Uses Algorithm: J. Xavier et. al., Fast line, arc/circle and leg detection from laser scan data
  /// \param cluster List of laser points in a cluster
  /// \return Bool for classification - True if it is a circle
  bool classifyCircle(std::vector<turtlelib::Vector2D> cluster);

private:
  /// \brief Distance threshold between two points when clustering
  double dist_threshold;

  /// \brief STD threshold for classifiying a circle
  double classify_std;

  /// \brief Mean min threshold for classifiying a circle
  double classify_mean_min;

  /// \brief Mean max threshold for classifiying a circle
  double classify_mean_max;
};
}

#endif
