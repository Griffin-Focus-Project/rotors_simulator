//
// Created by griffin on 10.11.20.
//


#include "griffin_controller_node.h"
#include <ros/ros.h>
#include "griffin_controller_node.h"

namespace rotors_control{

    GriffinControllerNode::GriffinControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) :
    nh_(nh), private_nh_(private_nh){

        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                      &GriffinControllerNode::OdometryCallback, this);

    }


    GriffinControllerNode::~GriffinControllerNode() { }

    void GriffinControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

        ROS_INFO_ONCE("GriffinController got first odometry message.");

        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);
        /*
     lee_position_controller_.SetOdometry(odometry);


     Eigen::VectorXd ref_rotor_velocities;
     lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

     // Todo(ffurrer): Do this in the conversions header.
     mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

     actuator_msg->angular_velocities.clear();
     for (int i = 0; i < ref_rotor_velocities.size(); i++)
         actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
     actuator_msg->header.stamp = odometry_msg->header.stamp;

     motor_velocity_reference_pub_.publish(actuator_msg);
      */
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "griffin_controller_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    rotors_control::GriffinControllerNode griffin_controller_node(nh, private_nh);

    ros::spin();

    return 0;
}
