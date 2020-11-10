//
// Created by griffin on 09.11.20.
//

#ifndef GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H
#define GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <stdio.h>

#include "griffin_control/position_controller.h"
#include "griffin_control/common.h"


namespace griffin_control {

    class PositionControllerNode {
    public:
        PositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

        ~PositionControllerNode();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber odometry_sub_;

        void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);


    };
}

#endif //GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H
