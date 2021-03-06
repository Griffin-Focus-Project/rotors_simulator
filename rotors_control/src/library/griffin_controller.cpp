#include "rotors_control/griffin_controller.h"

namespace rotors_control {

    GriffinController::GriffinController()
            : controller_active_(false),
            initialized_params_(false) {
        InitializeParameters();
    }

    GriffinController::~GriffinController() {}

    void GriffinController::InitializeParameters() {

        calculateAllocation(&(controller_parameters_.allocation_matrix_));
        Allocation_Matrix_PseudoInverse = controller_parameters_.allocation_matrix_.transpose()
                                                          * ((controller_parameters_.allocation_matrix_ * controller_parameters_.allocation_matrix_.transpose()).inverse()); // A^{ \dagger} = A^T*(A*A^T)^{-1}

        Inertia_Matrix.setZero();
        Inertia_Matrix = vehicle_parameters_.inertia_;


        initialized_params_ = true;

    }


    void GriffinController::CalculateRotorOutputs(Eigen::VectorXd* rotor_outputs) const {
        assert(rotor_outputs);
        assert(initialized_params_);

        // Return 0 velocities on all rotors, until the first command is received.
        if (!controller_active_) {
            *rotor_outputs = Eigen::VectorXd::Zero(rotor_outputs->rows());
            return;
        }

        Eigen::Vector3d forces;
        ComputeDesiredForces(&forces);

        Eigen::Vector3d torques;
        ComputeDesiredTorques(forces, &torques);


        Eigen::VectorXd forces_torques(forces.size() + torques.size());
        forces_torques << forces, torques;
        *rotor_outputs = Allocation_Matrix_PseudoInverse * forces_torques;
    }

    void GriffinController::SetOdometry(const EigenOdometry& odometry) {
        odometry_ = odometry;
    }

    void GriffinController::SetTrajectoryPoint(
            const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
        command_trajectory_ = command_trajectory;
        controller_active_ = true;
    }

    void GriffinController::ComputeDesiredForces(Eigen::Vector3d* forces) const {
        assert(forces);

        Eigen::Vector3d position_error;
        position_error = odometry_.position - command_trajectory_.position_W;

        // Transform velocity to world frame.
        const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix(); //Alles in world frame berechnen!
        Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
        Eigen::Vector3d velocity_error;
        velocity_error = velocity_W - command_trajectory_.velocity_W;

        Eigen::Vector3d gravity_vec(Eigen::Vector3d::UnitZ()); //


        *forces = vehicle_parameters_.mass_*(R_W_I.transpose()*(( -position_error.cwiseProduct(controller_parameters_.position_gain_))
                - velocity_error.cwiseProduct(controller_parameters_.velocity_gain_) + command_trajectory_.acceleration_W - gravity_vec * vehicle_parameters_.gravity_)
                        + odometry_.angular_velocity.cross(R_W_I.transpose()*odometry_.velocity));

    }


    void GriffinController::ComputeDesiredTorques(const Eigen::Vector3d& forces,
                                                         Eigen::Vector3d* torques) const {
        assert(torques);


        Eigen::Vector3d x_com(0 , 0 , 0);  //TODO x_com (center of mass offset)


        Eigen::Matrix3d R_SP = command_trajectory_.orientation_W_B.toRotationMatrix();     //Orientation Matrix setpoint
        Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
        Eigen::Matrix3d orientation_error_matrix = 0.5 * (R_SP.transpose() * R - R.transpose() * R_SP);
        Eigen::Vector3d orientation_error;
        vectorFromSkewMatrix(orientation_error_matrix, &orientation_error);

        Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R.transpose() * (R_SP * command_trajectory_.angular_acceleration_W); //e_omega

        *torques = Inertia_Matrix * ( - orientation_error.cwiseProduct(controller_parameters_.orientation_gain_) - angular_rate_error.cwiseProduct(controller_parameters_.angular_rate_gain_))
                    + odometry_.angular_velocity.cross(Inertia_Matrix * odometry_.angular_velocity) + x_com.cross(forces);
    }
}
