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

#ifndef ROTORS_CONTROL_LQR_CONTROLLER_H
#define ROTORS_CONTROL_LQR_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <control_toolbox/pid.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

class LqrControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LqrControllerParameters() {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  RotorConfiguration rotor_configuration_;
};

class LqrController {
 public:
  LqrController();
  ~LqrController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, Eigen::Vector3d* angle_error) const;

  void SetOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  LqrControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;
  Eigen::Matrix<double,4,12> control_gain;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::MatrixX4d control_input_to_rotor_velocities_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;

  void ComputeError(Eigen::VectorXd* error_signal, Eigen::Vector3d* angle_error) const;
};
}

#endif // ROTORS_CONTROL_LQR_CONTROLLER_H
