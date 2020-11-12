//
// Created by griffin on 10.11.20.
//

#ifndef ROTORS_CONTROL_GRIFFIN_CONTROLLER_H
#define ROTORS_CONTROL_GRIFFIN_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"


namespace rotors_control {

    class GriffinController {
    public:
        GriffinController();
        ~GriffinController();

        void SetOdometry(const EigenOdometry& odometry);
        void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;



        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:

        EigenOdometry odometry_;

        void CalculateForces(Eigen::Vector3d* forces) const;
        void CalculateTorques(Eigen::Vector3d* torques) const;

    };
}


#endif //SRC_GRIFFIN_CONTROLLER_H
