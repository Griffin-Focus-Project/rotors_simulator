//
// Created by griffin on 10.11.20.
//

#include "rotors_control/griffin_controller.h"

namespace rotors_control{

    void GriffinController::SetOdometry(const EigenOdometry& odometry) {
        odometry_ = odometry;
    }

    void GriffinController::CalculateRotorVelocities(Eigen::VectorXd *rotor_velocities) const {


    }

    void GriffinController::CalculateForces(Eigen::Vector3d* forces) {

        Eigen::Vector3d position_error;
        

    }

    void GriffinController::CalculateTorques(Eigen::Vector3d* torques) const {}
};


