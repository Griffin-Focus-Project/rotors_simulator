#include "rotors_control/griffin_controller.h"

namespace rotors_control {

    GriffinController::GriffinController()
<<<<<<< HEAD
            : controller_active_(false),
            initialized_params_(false) {
        InitializeParameters();
=======
            : controller_active_(false) {
>>>>>>> feature/griffin_multimotor
    }

    GriffinController::~GriffinController() {}

<<<<<<< HEAD
    void GriffinController::InitializeParameters() {

        calculateAllocation(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
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

        rotor_outputs->resize(12,1);
        rotor_outputs->setZero();

=======

    void GriffinController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
        assert(rotor_velocities);

        // Return 0 velocities on all rotors, until the first command is received.
        if (!controller_active_) {
            *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
            return;
        }

>>>>>>> feature/griffin_multimotor
        Eigen::Vector3d forces;
        ComputeDesiredForces(&forces);

        Eigen::Vector3d torques;
        ComputeDesiredTorques(forces, &torques);

<<<<<<< HEAD

        Eigen::VectorXd forces_torques(forces.size() + torques.size());
        forces_torques << forces, torques;
        Eigen::VectorXd combined_output;
        combined_output.resize(12,1);
        combined_output = Allocation_Matrix_PseudoInverse * forces_torques;


        for (int i = 0; i < 3; i++) {
            (*rotor_outputs)(i+8,0) = atan2(combined_output(2*i,0), combined_output(2*i+1,0)); //angles from 8-11
            (*rotor_outputs)(i,0) = sin((*rotor_outputs)(i+8,0))*combined_output(2*i,0) + cos((*rotor_outputs)(i+8,0))*combined_output(2*i+1,0); //velocities from 0-7
            (*rotor_outputs)(i,0) = sqrt((*rotor_outputs)(i,0));
            (*rotor_outputs)(i+4,0) = (*rotor_outputs)(i,0);
        }

=======
        Eigen::MatrixXd Allocation_Matrix;
        Allocation_Matrix.resize(6,10);//TODO which size and initialize
        Allocation_Matrix.setOnes();



        Eigen::MatrixXd Allocation_Matrix_PseudoInverse = Allocation_Matrix.transpose() * ((Allocation_Matrix * Allocation_Matrix.transpose()).inverse()); // A^{ \dagger} = A^T*(A*A^T)^{-1}
        Eigen::VectorXd forces_torques(forces.size() + torques.size());
        forces_torques << forces, torques;
        *rotor_velocities = Allocation_Matrix_PseudoInverse * forces_torques;
>>>>>>> feature/griffin_multimotor
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

<<<<<<< HEAD
        Eigen::Vector3d gravity_vec(Eigen::Vector3d::UnitZ()); //
=======
        Eigen::Vector3d gravity_vec(0,0,1); //TODO -1 oder 1??
>>>>>>> feature/griffin_multimotor


        *forces = vehicle_parameters_.mass_*(R_W_I.transpose()*(( -position_error.cwiseProduct(controller_parameters_.position_gain_))
                - velocity_error.cwiseProduct(controller_parameters_.velocity_gain_) + command_trajectory_.acceleration_W - gravity_vec * vehicle_parameters_.gravity_)
                        + odometry_.angular_velocity.cross(R_W_I.transpose()*odometry_.velocity));

    }


    void GriffinController::ComputeDesiredTorques(const Eigen::Vector3d& forces,
                                                         Eigen::Vector3d* torques) const {
        assert(torques);

<<<<<<< HEAD

=======
        Eigen::Matrix3d Inertia_Matrix = Eigen::Matrix3d::Zero();
        Inertia_Matrix(0,0) = kDefaultInertiaXx;
        Inertia_Matrix(1,1) = kDefaultInertiaYy;
        Inertia_Matrix(2,2) = kDefaultInertiaZz;
>>>>>>> feature/griffin_multimotor
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
