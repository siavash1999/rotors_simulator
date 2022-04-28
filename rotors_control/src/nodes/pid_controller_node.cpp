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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include "std_msgs/Float32.h"
#include "pid_controller_node.h"
#include "rotors_control/parameters_ros.h"

namespace rotors_control {

PidControllerNode::PidControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &PidControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PidControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &PidControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  roll_angle_pub_ = nh_.advertise<std_msgs::Float32>("/pelican/angles/roll", 1);
  pitch_angle_pub_ = nh_.advertise<std_msgs::Float32>("/pelican/angles/pitch", 1);
  yaw_angle_pub_ = nh_.advertise<std_msgs::Float32>("/pelican/angles/yaw", 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &PidControllerNode::TimedCommandCallback, this,
                                  true, false);
}

PidControllerNode::~PidControllerNode() { }

void PidControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  /* GetRosParameter(private_nh_, "velocity_gain/x/p",
                  idas_controller_.controller_parameters_.velocity_gain_.p.x(),
                  &idas_controller_.controller_parameters_.velocity_gain_.p.x());
  GetRosParameter(private_nh_, "velocity_gain/x/i",
                  idas_controller_.controller_parameters_.velocity_gain_.i.x(),
                  &idas_controller_.controller_parameters_.velocity_gain_.i.x());
  GetRosParameter(private_nh_, "velocity_gain/x/d",
                  idas_controller_.controller_parameters_.velocity_gain_.d.x(),
                  &idas_controller_.controller_parameters_.velocity_gain_.d.x());
  GetRosParameter(private_nh_, "velocity_gain/y/p",
                  idas_controller_.controller_parameters_.velocity_gain_.p.y(),
                  &idas_controller_.controller_parameters_.velocity_gain_.p.y());
  GetRosParameter(private_nh_, "velocity_gain/y/i",
                  idas_controller_.controller_parameters_.velocity_gain_.i.y(),
                  &idas_controller_.controller_parameters_.velocity_gain_.i.y());
  GetRosParameter(private_nh_, "velocity_gain/y/d",
                  idas_controller_.controller_parameters_.velocity_gain_.d.y(),
                  &idas_controller_.controller_parameters_.velocity_gain_.d.y());
  GetRosParameter(private_nh_, "velocity_gain/z/p",
                  idas_controller_.controller_parameters_.velocity_gain_.p.z(),
                  &idas_controller_.controller_parameters_.velocity_gain_.p.z());
  GetRosParameter(private_nh_, "velocity_gain/z/i",
                  idas_controller_.controller_parameters_.velocity_gain_.i.z(),
                  &idas_controller_.controller_parameters_.velocity_gain_.i.z());
  GetRosParameter(private_nh_, "velocity_gain/z/d",
                  idas_controller_.controller_parameters_.velocity_gain_.d.z(),
                  &idas_controller_.controller_parameters_.velocity_gain_.d.z());
  GetRosParameter(private_nh_, "angular_rate_gain/roll/p",
                  idas_controller_.controller_parameters_.angular_rate_gain_.p.x(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.p.x());
  GetRosParameter(private_nh_, "angular_rate_gain/roll/i",
                  idas_controller_.controller_parameters_.angular_rate_gain_.i.x(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.i.x());
  GetRosParameter(private_nh_, "angular_rate_gain/roll/d",
                  idas_controller_.controller_parameters_.angular_rate_gain_.d.x(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.d.x());
  GetRosParameter(private_nh_, "angular_rate_gain/pitch/p",
                  idas_controller_.controller_parameters_.angular_rate_gain_.p.y(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.p.y());
  GetRosParameter(private_nh_, "angular_rate_gain/pitch/i",
                  idas_controller_.controller_parameters_.angular_rate_gain_.i.y(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.i.y());
  GetRosParameter(private_nh_, "angular_rate_gain/pitch/d",
                  idas_controller_.controller_parameters_.angular_rate_gain_.d.y(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.d.y());
  GetRosParameter(private_nh_, "angular_rate_gain/yaw/p",
                  idas_controller_.controller_parameters_.angular_rate_gain_.p.z(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.p.z());
  GetRosParameter(private_nh_, "angular_rate_gain/yaw/i",
                  idas_controller_.controller_parameters_.angular_rate_gain_.i.z(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.i.z());
  GetRosParameter(private_nh_, "angular_rate_gain/yaw/d",
                  idas_controller_.controller_parameters_.angular_rate_gain_.d.z(),
                  &idas_controller_.controller_parameters_.angular_rate_gain_.d.z());
  GetRosParameter(private_nh_, "angular_rate_gain/altitude_gain/p",
                  idas_controller_.controller_parameters_.altitude_gain_.x(),
                  &idas_controller_.controller_parameters_.altitude_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/altitude_gain/i",
                  idas_controller_.controller_parameters_.altitude_gain_.y(),
                  &idas_controller_.controller_parameters_.altitude_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/altitude_gain/d",
                  idas_controller_.controller_parameters_.altitude_gain_.z(),
                  &idas_controller_.controller_parameters_.altitude_gain_.z()); */
  GetVehicleParameters(private_nh_, &pid_controller_.vehicle_parameters_);
  pid_controller_.InitializeParameters();
}

void PidControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void PidControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PidControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PidControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("PID Controller got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  pid_controller_.SetOdometry(odometry);

  Eigen::Vector3d angle_euler;
  Eigen::VectorXd ref_rotor_velocities;
  pid_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &angle_euler);

  std_msgs::Float32 roll_angle;
  roll_angle.data = angle_euler.x();
  std_msgs::Float32 pitch_angle;
  pitch_angle.data = angle_euler.y();
  std_msgs::Float32 yaw_angle;
  yaw_angle.data = angle_euler.z();

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
  roll_angle_pub_.publish(roll_angle);
  pitch_angle_pub_.publish(pitch_angle);
  yaw_angle_pub_.publish(yaw_angle);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::PidControllerNode pid_controller_node(nh, private_nh);
  ros::spin();

  return 0;
}
