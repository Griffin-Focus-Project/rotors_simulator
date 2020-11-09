//
// Created by griffin on 09.11.20.
//

#ifndef GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H
#define GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <mav_msgs/Actuators.h>


class PositionControllerNode {
    public:
        LeePositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~LeePositionControllerNode();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber odometry_sub_;

        ros::Publisher motor_velocity_reference_pub_;
};


#endif //GRIFFIN_CONTROL_POSITION_CONTROLLER_NODE_H
