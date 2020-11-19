#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_H_

namespace rotors_control {
// Default values for the Griffin rotor configuration.
static constexpr double kDefaultRotor0Gamma = -1.0471975512;
static constexpr double kDefaultRotor1Gamma = 1.0471975512;
static constexpr double kDefaultRotor2Gamma = 3.14159265359;
static constexpr double kDefaultRotor3Gamma = 0.0;

static constexpr double kDefaultRotor0Phi = 0.0;
static constexpr double kDefaultRotor1Phi = 0.0;
static constexpr double kDefaultRotor2Phi = 0.0;
static constexpr double kDefaultRotor3Phi = -1.57079632679;


// Default vehicle parameters for Asctec Firefly.
static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLengthFront = 0.4;
static constexpr double kDefaultArmLengthBack = 0.6;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;


// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;

struct Rotor {
  Rotor()
      : gamma(0.0),
        phi(0.0),
        arm_length(kDefaultArmLengthFront),
        rotor_force_constant(kDefaultRotorForceConstant)
        {}
  Rotor(double _gamma, double _phi, double _arm_length,
        double _rotor_force_constant)
      : gamma(_gamma),
        phi(_phi),
        arm_length(_arm_length),
        rotor_force_constant(_rotor_force_constant)
        {}
  double gamma;
  double phi;
  double arm_length;
  double rotor_force_constant;
};

struct RotorConfiguration {
  RotorConfiguration() {
    // Rotor configuration of Asctec Firefly.
    rotors.push_back(
      Rotor(kDefaultRotor0Gamma, kDefaultRotor0Phi, kDefaultArmLengthFront, kDefaultRotorForceConstant));
    rotors.push_back(
      Rotor(kDefaultRotor1Gamma, kDefaultRotor1Phi, kDefaultArmLengthFront, kDefaultRotorForceConstant));
    rotors.push_back(
      Rotor(kDefaultRotor2Gamma, kDefaultRotor2Phi, kDefaultArmLengthFront, kDefaultRotorForceConstant));
    rotors.push_back(
      Rotor(kDefaultRotor3Gamma, kDefaultRotor3Phi, kDefaultArmLengthBack, kDefaultRotorForceConstant));
  }
  std::vector<Rotor> rotors;
};

class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()) {}
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
};

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_H_ */
