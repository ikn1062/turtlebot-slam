#include "nuslam/ekf_slam.hpp"

namespace ekf_slam
{
ExtendedKalmanFilter::ExtendedKalmanFilter()
{}

ExtendedKalmanFilter::ExtendedKalmanFilter(int n_obs, double Q_val, double R_val)
: n_obstacles(n_obs),
  len(3 + 2 * n_obs),
  state_mat(arma::mat(len, 1, arma::fill::zeros)),
  covariance_mat(arma::mat(len, len, arma::fill::zeros)),
  Q_mat(arma::mat(len, len, arma::fill::zeros)),
  R_mat(R_val * arma::mat(2, 2, arma::fill::eye)),
  landmarks_idx(0),
  man_thresh(0.1)
{
  arma::mat Q_upper(Q_val * arma::mat(3, 3, arma::fill::eye));
  Q_mat.submat(0, 0, 2, 2) = Q_upper;
  arma::mat covariance_lower(MAX * arma::mat(2 * n_obs, 2 * n_obs, arma::fill::eye));
  covariance_mat.submat(3, 3, len - 1, len - 1) = covariance_lower;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(
  turtlelib::Config robot_pose, int n_obs, double Q_val,
  double R_val)
: n_obstacles(n_obs),
  len(3 + 2 * n_obs),
  state_mat(arma::mat(len, 1, arma::fill::zeros)),
  covariance_mat(arma::mat(len, len, arma::fill::zeros)),
  Q_mat(arma::mat(len, len, arma::fill::zeros)),
  R_mat(R_val * arma::mat(2, 2, arma::fill::eye)),
  landmarks_idx(0),
  man_thresh(0.1)
{
  arma::mat Q_upper(Q_val * arma::mat(3, 3, arma::fill::eye));
  Q_mat.submat(0, 0, 2, 2) = Q_upper;
  arma::mat covariance_lower(MAX * arma::mat(2 * n_obs, 2 * n_obs, arma::fill::eye));
  covariance_mat.submat(3, 3, len - 1, len - 1) = covariance_lower;
  state_mat(0, 0) = robot_pose.theta;
  state_mat(1, 0) = robot_pose.x;
  state_mat(2, 0) = robot_pose.y;
}

arma::mat ExtendedKalmanFilter::calculate_A(turtlelib::Twist2D tw)
{
  arma::mat G_mat = arma::mat(len, len, arma::fill::zeros);
  arma::mat I = arma::mat(len, len, arma::fill::eye);
  arma::mat A_mat;
  double theta = state_mat(0, 0);
  double theta_norm = turtlelib::normalize_angle(theta);

  if (turtlelib::almost_equal(tw.dtheta, 0.0, 0.01)) {
    G_mat(1, 0) = -tw.dx * sin(theta);
    G_mat(2, 0) = tw.dx * cos(theta);
  } else {
    G_mat(1, 0) = -(tw.dx / tw.dtheta) * cos(theta_norm) + (tw.dx / tw.dtheta) * cos(
      turtlelib::normalize_angle(
        theta_norm + tw.dtheta));
    G_mat(2, 0) = -(tw.dx / tw.dtheta) * sin(theta_norm) + (tw.dx / tw.dtheta) * sin(
      turtlelib::normalize_angle(
        theta_norm + tw.dtheta));
  }

  A_mat = I + G_mat;
  return A_mat;
}

arma::mat ExtendedKalmanFilter::calculate_next_state(turtlelib::Twist2D tw)
{
  double dtheta, dx, dy;
  double theta = state_mat(0, 0);
  double theta_norm = turtlelib::normalize_angle(theta);

  if (turtlelib::almost_equal(tw.dtheta, 0.0, 0.01)) {
    dtheta = 0.0;
    dx = tw.dx * cos(theta);
    dy = tw.dx * sin(theta);
  } else {
    dtheta = tw.dtheta;
    dx = -(tw.dx / tw.dtheta) * sin(theta_norm) + (tw.dx / tw.dtheta) * sin(theta_norm + tw.dtheta);
    dy = (tw.dx / tw.dtheta) * cos(theta_norm) - (tw.dx / tw.dtheta) * cos(theta_norm + tw.dtheta);
  }

  state_mat(0, 0) = turtlelib::normalize_angle(theta + dtheta);
  state_mat(1, 0) += dx;
  state_mat(2, 0) += dy;

  return state_mat;
}

void ExtendedKalmanFilter::predict_step(turtlelib::Twist2D tw)
{
  arma::mat A_mat = calculate_A(tw);
  state_mat = calculate_next_state(tw);
  covariance_mat = A_mat * covariance_mat * (A_mat.t()) + Q_mat;
}

void ExtendedKalmanFilter::init_landmark(int obs_id, turtlelib::Vector2D obs_coords)
{
  double r = sqrt(pow(obs_coords.x, 2) + (pow(obs_coords.y, 2)));
  double phi = turtlelib::normalize_angle(atan2(obs_coords.y, obs_coords.x));
  state_mat(
    3 + 2 * obs_id,
    0) = state_mat(1, 0) + r * cos(turtlelib::normalize_angle(phi + state_mat(0, 0)));
  state_mat(
    4 + 2 * obs_id,
    0) = state_mat(2, 0) + r * sin(turtlelib::normalize_angle(phi + state_mat(0, 0)));
}

arma::mat ExtendedKalmanFilter::calculate_h_id(int obs_id, arma::mat input_mat)
{
  arma::mat h_id(arma::mat(2, 1, arma::fill::zeros));
  double curr_theta = input_mat(0, 0);
  double landmark_mx = input_mat(3 + 2 * obs_id, 0) - input_mat(1, 0);
  double landmark_my = input_mat(4 + 2 * obs_id, 0) - input_mat(2, 0);

  h_id(0, 0) = sqrt(pow(landmark_mx, 2) + pow(landmark_my, 2));
  h_id(1, 0) = turtlelib::normalize_angle(atan2(landmark_my, landmark_mx) - curr_theta);

  return h_id;
}

arma::mat ExtendedKalmanFilter::calculate_H_id(int obs_id, arma::mat input_mat)
{
  arma::mat H_mat(arma::mat(2, len, arma::fill::zeros));

  double dx = input_mat(3 + 2 * obs_id, 0) - input_mat(1, 0);
  double dy = input_mat(4 + 2 * obs_id, 0) - input_mat(2, 0);
  double disc = pow(dx, 2) + pow(dy, 2);

  H_mat(0, 0) = 0.0;
  H_mat(0, 1) = -dx / sqrt(disc);
  H_mat(0, 2) = -dy / sqrt(disc);
  H_mat(1, 0) = -1.0;
  H_mat(1, 1) = dy / disc;
  H_mat(1, 2) = -dx / disc;

  H_mat(0, 3 + 2 * obs_id) = dx / sqrt(disc);
  H_mat(0, 4 + 2 * obs_id) = dy / sqrt(disc);
  H_mat(1, 3 + 2 * obs_id) = -dy / disc;
  H_mat(1, 4 + 2 * obs_id) = dx / disc;

  return H_mat;
}

arma::mat ExtendedKalmanFilter::update_step(int obs_id, turtlelib::Vector2D obs_coords)
{
  arma::mat I;
  arma::mat h_id;
  arma::mat H_id;
  arma::mat z_id;
  arma::mat z_id_delta;
  arma::mat K_id;

  I = arma::mat(len, len, arma::fill::eye);

  h_id = calculate_h_id(obs_id, state_mat);
  H_id = calculate_H_id(obs_id, state_mat);

  z_id = arma::mat(2, 1, arma::fill::zeros);
  z_id(0, 0) = sqrt(pow(obs_coords.x, 2) + (pow(obs_coords.y, 2)));
  z_id(1, 0) = turtlelib::normalize_angle(atan2(obs_coords.y, obs_coords.x));

  z_id_delta = z_id - h_id;
  z_id_delta(1, 0) = turtlelib::normalize_angle(z_id_delta(1, 0));

  K_id = covariance_mat * H_id.t() * arma::inv(H_id * covariance_mat * H_id.t() + R_mat);

  state_mat = state_mat + K_id * (z_id_delta);
  state_mat(0, 0) = turtlelib::normalize_angle(state_mat(0, 0));

  covariance_mat = (I - K_id * H_id) * covariance_mat;

  return state_mat;
}


int ExtendedKalmanFilter::dataAssociation(turtlelib::Vector2D obs_coords)
{
  arma::mat Hk_mat, Psi_mat, zk_hat, zk_diff, z_new, dk;
  arma::mat temp = state_mat;

  if (landmarks_idx == 0) {
    init_landmark(landmarks_idx, obs_coords);
    landmarks_idx++;
    return 0;
  }

  z_new = arma::mat(2, 1, arma::fill::zeros);
  z_new(0, 0) = sqrt(pow(obs_coords.x, 2) + (pow(obs_coords.y, 2)));
  z_new(1, 0) = turtlelib::normalize_angle(atan2(obs_coords.y, obs_coords.x));
  //std::cout << "z_new: " << z_new << std::endl;

  temp(
    3 + 2 * landmarks_idx,
    0) = temp(1, 0) + z_new(0, 0) * cos(turtlelib::normalize_angle(z_new(1, 0) + temp(0, 0)));
  temp(
    4 + 2 * landmarks_idx,
    0) = temp(2, 0) + z_new(0, 0) * sin(turtlelib::normalize_angle(z_new(1, 0) + temp(0, 0)));

  for (int k = 0; k < landmarks_idx + 1; k++) {
    Hk_mat = calculate_H_id(k, temp);
    Psi_mat = Hk_mat * covariance_mat * Hk_mat.t() + R_mat;
    //std::cout << "Psi_mat: " << Psi_mat << std::endl;
    zk_hat = calculate_h_id(k, temp);
    //std::cout << "zk_hat: " << zk_hat << std::endl;
    double curr_man_dist;
    if (k == landmarks_idx) {
      break;
    } else {
      zk_diff = z_new - zk_hat;
      //std::cout << "zk_diff: " << zk_diff << std::endl;
      dk = zk_diff.t() * Psi_mat.i() * zk_diff;
      curr_man_dist = fabs(dk(0, 0));
      std::cout << "dk: " << fabs(dk(0, 0)) << std::endl;
    }

    if (curr_man_dist < man_thresh) {
      //std::cout << "man dist: " << curr_man_dist << std::endl;
      return k;
    }

  }
  init_landmark(landmarks_idx, obs_coords);
  int res = landmarks_idx;
  landmarks_idx++;
  return res;
}

}
