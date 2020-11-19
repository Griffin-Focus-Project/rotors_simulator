#ifndef ROTORS_CONTROL_GRIFFIN_CONTROLLER_H
#define ROTORS_CONTROL_GRIFFIN_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {


    static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
    static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
    static const Eigen::Vector3d kDefaultOrientationGain = Eigen::Vector3d(3, 3, 0.035);
    static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

    class GriffinControllerParameters {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GriffinControllerParameters()
                : position_gain_(kDefaultPositionGain),
                  velocity_gain_(kDefaultVelocityGain),
                  orientation_gain_(kDefaultOrientationGain),
                  angular_rate_gain_(kDefaultAngularRateGain) {
            calculateAllocation(rotor_configuration_, &allocation_matrix_);
        }
        Eigen::MatrixXd allocation_matrix_;
        Eigen::Vector3d position_gain_;
        Eigen::Vector3d velocity_gain_;
        Eigen::Vector3d orientation_gain_;
        Eigen::Vector3d angular_rate_gain_;
        RotorConfiguration rotor_configuration_;
    };

    class GriffinController {
    public:
        GriffinController();
        ~GriffinController();
        void InitializeParameters();
        void CalculateRotorOutputs(Eigen::VectorXd* rotor_outputs) const;

        void SetOdometry(const EigenOdometry& odometry);
        void SetTrajectoryPoint(
                const mav_msgs::EigenTrajectoryPoint& command_trajectory);

        GriffinControllerParameters controller_parameters_;
        VehicleParameters vehicle_parameters_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool controller_active_;
        bool initialized_params_;

        Eigen::MatrixXd Allocation_Matrix_PseudoInverse;
        Eigen::Matrix3d Inertia_Matrix;

        mav_msgs::EigenTrajectoryPoint command_trajectory_;
        EigenOdometry odometry_;

        void ComputeDesiredTorques(const Eigen::Vector3d& forces,
                                      Eigen::Vector3d* torques) const;
        void ComputeDesiredForces(Eigen::Vector3d* forces) const;
    };
}

#endif
