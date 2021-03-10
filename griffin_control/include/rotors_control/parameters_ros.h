#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_

#include <ros/ros.h>

#include "rotors_control/parameters.h"

namespace rotors_control {

    template<typename T>
    inline void GetRosParameter(const ros::NodeHandle &nh,
                                const std::string &key,
                                const T &default_value,
                                T *value) {
        ROS_ASSERT(value != nullptr);
        bool have_parameter = nh.getParam(key, *value);
        if (!have_parameter) {
            ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                                                                    << "/" << key << ", setting to default: "
                                                                    << default_value);
            *value = default_value;
        }
    }


    inline void GetVehicleParameters(const ros::NodeHandle &nh, VehicleParameters *vehicle_parameters) {
        GetRosParameter(nh, "mass",
                        vehicle_parameters->mass_,
                        &vehicle_parameters->mass_);
        GetRosParameter(nh, "rotor_force_constant",
                        vehicle_parameters->rotor_force_constant_,
                        &vehicle_parameters->rotor_force_constant_);
        GetRosParameter(nh, "front_arm_length",
                        vehicle_parameters->front_arm_length_,
                        &vehicle_parameters->front_arm_length_);
        GetRosParameter(nh, "back_arm_length",
                        vehicle_parameters->back_arm_length_,
                        &vehicle_parameters->back_arm_length_);
        GetRosParameter(nh, "inertia/xx",
                        vehicle_parameters->inertia_(0, 0),
                        &vehicle_parameters->inertia_(0, 0));
        GetRosParameter(nh, "inertia/xy",
                        vehicle_parameters->inertia_(0, 1),
                        &vehicle_parameters->inertia_(0, 1));
        vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
        GetRosParameter(nh, "inertia/xz",
                        vehicle_parameters->inertia_(0, 2),
                        &vehicle_parameters->inertia_(0, 2));
        vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
        GetRosParameter(nh, "inertia/yy",
                        vehicle_parameters->inertia_(1, 1),
                        &vehicle_parameters->inertia_(1, 1));
        GetRosParameter(nh, "inertia/yz",
                        vehicle_parameters->inertia_(1, 2),
                        &vehicle_parameters->inertia_(1, 2));
        vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
        GetRosParameter(nh, "inertia/zz",
                        vehicle_parameters->inertia_(2, 2),
                        &vehicle_parameters->inertia_(2, 2));
    }
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_ */
