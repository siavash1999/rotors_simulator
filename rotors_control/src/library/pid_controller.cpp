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

#include "rotors_control/pid_controller.h"

namespace rotors_control {

PidController::PidController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

PidController::~PidController() {}

void PidController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

  control_input_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseudo-inverse
  control_input_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
  * (controller_parameters_.allocation_matrix_ * controller_parameters_.allocation_matrix_.transpose()).inverse();
  
  initialized_params_ = true;
}

void PidController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, Eigen::Vector3d* angle_error) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector4d control_input;
  CalculateThrustMoments(&control_input, angle_error);

  *rotor_velocities = control_input_to_rotor_velocities_ * control_input;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void PidController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void PidController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void PidController::CalculateRollPitchThrust(Eigen::Vector3d* roll_pitch_thrust_ptr, Eigen::Vector3d* euler_angles_ptr) const {
  // TODO: Make parameters for gains, store them in parameter server, make them dynamically reconfigurable.
  Eigen::Vector3d position_gain(0.05, -0.05, 4);
  Eigen::Vector3d velocity_gain( 0.1,  -0.1, 7);
  Eigen::Matrix3d yaw_rotation;

  quaternionToEuler(odometry_.orientation, euler_angles_ptr);

  yaw_rotation << cos(euler_angles_ptr->z()), -sin(euler_angles_ptr->z()), 0,
                  sin(euler_angles_ptr->z()),  cos(euler_angles_ptr->z()), 0,
                                           0,                           0, 1;

  *roll_pitch_thrust_ptr = yaw_rotation * 
                           (position_gain.cwiseProduct(command_trajectory_.position_W - odometry_.position) + 
                           velocity_gain.cwiseProduct(command_trajectory_.velocity_W - odometry_.velocity));
  double temp = (*roll_pitch_thrust_ptr).x();
  (*roll_pitch_thrust_ptr).x() = (*roll_pitch_thrust_ptr).y();
  (*roll_pitch_thrust_ptr).y() = temp;
  (*roll_pitch_thrust_ptr).z() += vehicle_parameters_.mass_ * 9.81;
}

void PidController::CalculateThrustMoments(Eigen::Vector4d* control_input_ptr, Eigen::Vector3d* angle_error) const {
  // TODO: do something for direct command for attitude. Same procedure for gains.
  Eigen::Vector3d attitude_gain(1.5, 1.5, 0.004);
  Eigen::Vector3d angular_velocity_gain(0.2, 0.2, 0.012);
  Eigen::Vector3d roll_pitch_thrust;
  Eigen::Vector3d euler_angles;
  CalculateRollPitchThrust(&roll_pitch_thrust, &euler_angles);

  Eigen::Vector3d desired_attitude;
  quaternionToEuler(command_trajectory_.orientation_W_B, &desired_attitude);
  desired_attitude.segment<2>(0) = roll_pitch_thrust.segment<2>(0);

  control_input_ptr->segment<3>(0) = attitude_gain.cwiseProduct(desired_attitude - euler_angles) +
                                     angular_velocity_gain.cwiseProduct(-odometry_.angular_velocity);
  control_input_ptr->w() = roll_pitch_thrust.z();
  *angle_error = desired_attitude - euler_angles;
}
}
