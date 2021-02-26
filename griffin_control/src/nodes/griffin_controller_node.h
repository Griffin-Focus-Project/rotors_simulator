#ifndef ROTORS_CONTROL_GRIFFIN_CONTROLLER_NODE_H
#define ROTORS_CONTROL_GRIFFIN_CONTROLLER_NODE_H

#include <boost/bind.hpp>

#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "griffin_control/common.h"
#include "griffin_contol/griffin_controller.h"

namespace rotors_control {

    class GriffinControllerNode {
    public:
        GriffinControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~GriffinControllerNode();

        void InitializeParams();
        void Publish();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        GriffinController griffin_controller_;

        std::string namespace_;

        // subscribers
        ros::Subscriber cmd_trajectory_sub_;
        ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
        ros::Subscriber cmd_pose_sub_;
        ros::Subscriber odometry_sub_;

        ros::Publisher motor_velocity_reference_pub_;

        mav_msgs::EigenTrajectoryPointDeque commands_;
        std::deque<ros::Duration> command_waiting_times_;
        ros::Timer command_timer_;

        void TimedCommandCallback(const ros::TimerEvent& e);

        void MultiDofJointTrajectoryCallback(
                const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

        void CommandPoseCallback(
                const geometry_msgs::PoseStampedConstPtr& pose_msg);

        void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    };
}

#endif
