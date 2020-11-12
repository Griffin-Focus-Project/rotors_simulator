#include "rotors_control/griffin_controller.h"

namespace rotors_control {

    GriffinController::GriffinController()
            : controller_active_(false) {
    }

    GriffinController::~GriffinController() {}


    void GriffinController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
        assert(rotor_velocities);

        // Return 0 velocities on all rotors, until the first command is received.
        if (!controller_active_) {
            *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
            return;
        }

        Eigen::Vector3d forces;
        ComputeDesiredForces(&forces);

        Eigen::Vector3d torques;
        ComputeDesiredTorques(forces, &torques);

        Eigen::MatrixXd Allocation_Matrix;
        Allocation_Matrix.resize(6,10); //TODO which size and initialize



        Eigen::MatrixXd Allocation_Matrix_PseudoInverse = Allocation_Matrix.transpose() * ((Allocation_Matrix * Allocation_Matrix.transpose()).inverse()); // A^{ \dagger} = A^T*(A*A^T)^{-1}
        Eigen::VectorXd forces_torques(forces.size() + torques.size());
        forces_torques << forces, torques;
        *rotor_velocities = Allocation_Matrix_PseudoInverse * forces_torques;
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


        *forces = vehicle_parameters_.mass_*(R_W_I.transpose()*(- position_error.cwiseProduct(controller_parameters_.position_gain_)
                - velocity_error.cwiseProduct(controller_parameters_.velocity_gain_) + command_trajectory_.acceleration_W
                - vehicle_parameters_.gravity_) + odometry_.angular_velocity.cross(R_W_I.transpose()*odometry_.velocity));

    }


    void GriffinController::ComputeDesiredTorques(const Eigen::Vector3d& forces,
                                                         Eigen::Vector3d* torques) const {
        assert(torques);

        Eigen::Matrix3d Inertia_Matrix = Eigen::Matrix3d::Zero();
        Inertia_Matrix(0,0) = kDefaultInertiaXx;
        Inertia_Matrix(1,1) = kDefaultInertiaYy;
        Inertia_Matrix(2,2) = kDefaultInertiaZz;
        Eigen::Vector3d x_com(0 , 0 , 0);  //TODO x_com (center of mass offset)


        Eigen::Matrix3d R_SP = command_trajectory_.orientation_W_B.toRotationMatrix();     //Orientation Matrix setpoint
        Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
        Eigen::Vector3d orientation_error = 0.5 * (R_SP.transpose() * R - R.transpose() * R_SP);   //e_R

        Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R.transpose() * R_SP * command_trajectory_.angular_acceleration_W //e_omega

        *torques = Inertia_Matrix * ( - controller_parameters_.orientation_gain_ * orientation_error - controller_parameters_.angular_rate_gain_ * angular_rate_error)
                    + odometry_.angular_velocity.cross(Inertia_Matrix * odometry_.angular_velocity) + x_com.cross(forces);
    }
}
