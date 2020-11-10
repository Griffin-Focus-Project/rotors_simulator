//
// Created by griffin on 09.11.20.
//

#include "position_controller_node.h"
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>

namespace griffin_control {
    PositionControllerNode::PositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh) {

        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                      &LeePositionControllerNode::OdometryCallback, this);
    }

    PositionControllerNode::~PositionControllerNode() { }


    void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

        ROS_INFO_ONCE("PositionController got first odometry message.");

        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);
        /*lee_position_controller_.SetOdometry(odometry);

        Eigen::Vector6d ref_rotor_velocities;
        position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        actuator_msg->angular_velocities.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = odometry_msg->header.stamp;

        motor_velocity_reference_pub_.publish(actuator_msg);
        */
    }





int main(int argc, char** argv) {
    ros::init(argc, argv, "position_controller_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    griffin_control::PositionControllerNode position_controller_node(nh, private_nh);

    ros::spin();

    return 0;
}