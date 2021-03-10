#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_H_

namespace rotors_control {


static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLengthFront = 0.4;
static constexpr double kDefaultArmLengthBack = 0.6;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;


// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;



class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        rotor_force_constant_(kDefaultRotorForceConstant),
        front_arm_length_(kDefaultArmLengthFront),
        back_arm_length_(kDefaultArmLengthBack),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()){}
  double mass_;
  const double gravity_;
  double rotor_force_constant_;
  double front_arm_length_;
  double back_arm_length_;
  Eigen::Matrix3d inertia_;
};

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_H_ */
