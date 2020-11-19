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

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

#include <assert.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values.
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandMotorSpeedTopic =
    mav_msgs::default_topics::COMMAND_ACTUATORS; // "command/motor_speed";
static const std::string kDefaultCommandMultiDofJointTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_TRAJECTORY; // "command/trajectory"
static const std::string kDefaultCommandRollPitchYawrateThrustTopic =
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST;
    // "command/roll_pitch_yawrate_thrust"
static const std::string kDefaultImuTopic =
    mav_msgs::default_topics::IMU; // "imu
static const std::string kDefaultOdometryTopic =
    mav_msgs::default_topics::ODOMETRY; // "odometry"

struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}


inline void calculateAllocation(const RotorConfiguration& rotor_configuration,
                                Eigen::MatrixXd* allocation_matrix){
    assert(allocation_matrix != nullptr);
    allocation_matrix->resize(6,10);



    double front_arm_length = rotor_configuration.rotors[0].arm_length; //TODO does this work??
    double back_arm_length = rotor_configuration.rotors[3].arm_length;
    double rotor_force_constant =  8.54858e-6;



    allocation_matrix->setZero();
    (*allocation_matrix)(1,0) = 8/5;
    (*allocation_matrix)(3,0) = -(8*front_arm_length/5);

    (*allocation_matrix)(0,1) = 8/5;
    (*allocation_matrix)(4,1) = 8*front_arm_length/5;

    (*allocation_matrix)(1,2) = -(4/5);
    (*allocation_matrix)(2,2) = (4*pow(3,0.5)/5);
    (*allocation_matrix)(3,2) = -(8*front_arm_length/5);

    (*allocation_matrix)(0,3) = 8/5;
    (*allocation_matrix)(4,3) = -(4*front_arm_length)/5;
    (*allocation_matrix)(5,3) = (4*pow(3,0.5)*front_arm_length)/5;

    (*allocation_matrix)(1,4) = -4/5;
    (*allocation_matrix)(2,4) = -(4*pow(3,0.5)/5);
    (*allocation_matrix)(3,4) = -(8*front_arm_length/5);

    (*allocation_matrix)(0,5) = 8/5;
    (*allocation_matrix)(4,5) = -(4*front_arm_length)/5;
    (*allocation_matrix)(5,5) = -(4*pow(3,0.5)*front_arm_length/5);

    (*allocation_matrix)(1,6) = 8/5;
    (*allocation_matrix)(3,6) = 0; //almost zero
    (*allocation_matrix)(5,6) = -(8*back_arm_length/5);

    (*allocation_matrix)(0,7) = 0; //almost zero
    (*allocation_matrix)(2,7) = 8/5;
    (*allocation_matrix)(4,7) = 1.6*back_arm_length;


    (*allocation_matrix) *= rotor_force_constant;
}

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}
}

#endif /* INCLUDE_ROTORS_CONTROL_COMMON_H_ */
