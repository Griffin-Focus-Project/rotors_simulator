//
// Created by griffin on 10.11.20.
//

#ifndef ROTORS_CONTROL_GRIFFIN_CONTROLLER_NODE_H
#define ROTORS_CONTROL_GRIFFIN_CONTROLLER_NODE_H

#include <boost/bind.hpp>

#include <Eigen/Eigen>
#include <stdio.h>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include "rotors_control/common.h"

namespace rotors_control {
    class GriffinControllerNode {
    public:
        GriffinControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
        ~GriffinControllerNode();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber odometry_sub_;

        void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    };
}


#endif //ROTORS_CONTROL_GRIFFIN_CONTROLLER_NODE_H
