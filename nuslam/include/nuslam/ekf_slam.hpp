#ifndef EKF_SLAM_INCLUDE_GUARD_HPP
#define EKF_SLAM_INCLUDE_GUARD_HPP

/// \file
/// \brief Extended Kalman Filter SLAM Library

#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <armadillo>
#include <iostream>
#include <cmath>

const double MAX = 2e31;

namespace ekf_slam
{
/// \brief A class for computing stats and covariance matrix for an Extended Kalman Filter
class ExtendedKalmanFilter
{
public:
  /// \brief Initializes the Extended Kalman Filter Object
  ExtendedKalmanFilter();

  /// \brief Initializes the Extended Kalman Filter Object with number of objects, Q, and R
  ExtendedKalmanFilter(int n_obs, double Q_val, double R_val);

  /// \brief Initializes the Extended Kalman Filter Object with robot state, number of objects, Q, and R
  ExtendedKalmanFilter(turtlelib::Config robot_pose, int n_obs, double Q_val, double R_val);

  /// \brief Calculates the A matrix from the robot twist
  /// \param tw Input Twist / controls
  /// \return Matrix of size (3+2n x 3+2n) representing the transition matrix A
  arma::mat calculate_A(turtlelib::Twist2D tw);

  /// \brief Calculates the next state of robot given input twist
  /// \param tw Input Twist / controls
  /// \return Updated Matrix containing robot and landmark state variables
  arma::mat calculate_next_state(turtlelib::Twist2D tw);

  /// \brief Computes the predict step of EKF SLAM - updates state and covariance matrix
  /// \param tw Input Twist / controls
  /// \return Updated Matrix containing robot and landmark state variables
  void predict_step(turtlelib::Twist2D tw);

  /// \brief Initializes a landmark in the state matrix
  /// \param obs_id Input id of landmark - used for indexing
  /// \param obs_coords Input coordinate of landmark
  void init_landmark(int obs_id, turtlelib::Vector2D obs_coords);

  /// \brief Caclutes and returns a range-bearing vector
  /// \param obs_id Input id of landmark - used for indexing
  /// \return h_id range-bearing vector
  arma::mat calculate_h_id(int obs_id, arma::mat input_mat);

  /// \brief Caclutes and returns the H matrix for obstacle id
  /// \param obs_id Input id of landmark - used for indexing
  /// \param obs_coords Input coordinate of landmark
  /// \return H matrix
  arma::mat calculate_H_id(int obs_id, arma::mat input_mat);

  /// \brief Computes the update step of EKF SLAM - updates state and covariance matrix
  /// \param obs_id Input id of landmark - used for indexing
  /// \param obs_coords Input coordinate of landmark
  /// \return Updated Matrix containing robot and landmark state variables
  arma::mat update_step(int obs_id, turtlelib::Vector2D obs_coords);

  /// \brief Associates landmarks with obstacles in a given space
  /// \param obs_coords Input Coordinates of obstacle landmarks
  /// \return Landmark index for obstacle
  int dataAssociation(turtlelib::Vector2D obs_coords);

private:
  /// \brief Number of obstacles
  int n_obstacles;

  /// \brief Len of fulle matrix = 2*n_obstacles + 3
  int len;

  /// \brief State matrix
  arma::mat state_mat;

  /// \brief Covariance matrix
  arma::mat covariance_mat;

  /// \brief Process Noise matrix
  arma::mat Q_mat;

  /// \brief Sensor Noise matrix
  arma::mat R_mat;

  /// \brief Current length of known slandmarks for data associations
  int landmarks_idx;

  /// \brief Threshold for data association distance
  double man_thresh;


};
}

#endif
