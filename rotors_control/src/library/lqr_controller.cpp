/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/lqr_controller.h"

namespace rotors_control {

LqrController::LqrController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

LqrController::~LqrController() {}

void LqrController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

  control_input_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseudo-inverse
  control_input_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
  * (controller_parameters_.allocation_matrix_ * controller_parameters_.allocation_matrix_.transpose()).inverse();

  control_gain <<1.5,   0,   0, 0.2,   0,    0,   0, -0.1, 0,    0, -0.05, 0,
                   0, 1.5,   0,   0, 0.2,    0, 0.1,    0, 0, 0.05,     0, 0,
                   0,   0, 0.4,   0,   0, 0.12,   0,    0, 0,    0,     0, 0,
                   0,   0,   0,   0,   0,    0,   0,    0, 7,    0,     0, 4;
  
  initialized_params_ = true;
}

void LqrController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::VectorXd error_signal(12);
  ComputeError(&error_signal);

  Eigen::Vector4d control_input;
  control_input = control_gain * error_signal;
  control_input(3) += vehicle_parameters_.mass_ * 9.81;
  *rotor_velocities = control_input_to_rotor_velocities_ * control_input;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LqrController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LqrController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LqrController::ComputeError(Eigen::VectorXd* error_signal) const {
  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();

  Eigen::Vector3d command_trajectory_euler;
  Eigen::Vector3d odometry_euler;
  quaternionToEuler(command_trajectory_.orientation_W_B, &command_trajectory_euler);
  quaternionToEuler(odometry_.orientation, &odometry_euler);
  Eigen::Vector3d angle_error = command_trajectory_euler - odometry_euler;
  error_signal->block<3,1>(0,0) = angle_error;

  Eigen::Vector3d angular_velocity_error = R_W_I.transpose() * (command_trajectory_.angular_velocity_W
                   - odometry_.angular_velocity);
  error_signal->block<3,1>(3,0) = angular_velocity_error;

  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = command_trajectory_.velocity_W - velocity_W;
  error_signal->block<3,1>(6,0) = velocity_error;

  Eigen::Matrix3d yaw_rotation;
  yaw_rotation << cos(odometry_euler.z()), sin(odometry_euler.z()), 0,
                 -sin(odometry_euler.z()), cos(odometry_euler.z()), 0,
                                     0,                          0, 1;
  Eigen::Vector3d position_error;
  position_error = yaw_rotation * (command_trajectory_.position_W - odometry_.position);
  error_signal->block<3,1>(9,0) = position_error;


}
}
