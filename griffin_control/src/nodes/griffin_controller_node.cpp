#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "griffin_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

    GriffinControllerNode::GriffinControllerNode(
            const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
            :nh_(nh),
             private_nh_(private_nh){
        InitializeParams();

        cmd_pose_sub_ = nh_.subscribe(
                mav_msgs::default_topics::COMMAND_POSE, 1,
                &GriffinControllerNode::CommandPoseCallback, this);

        cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                &GriffinControllerNode::MultiDofJointTrajectoryCallback, this);

        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                      &GriffinControllerNode::OdometryCallback, this);

        motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
                mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

        command_timer_ = nh_.createTimer(ros::Duration(0), &GriffinControllerNode::TimedCommandCallback, this,
                                         true, false);
    }

    GriffinControllerNode::~GriffinControllerNode() { }

    void GriffinControllerNode::InitializeParams() {

        // Read parameters from rosparam.
        GetRosParameter(private_nh_, "position_gain/x",
                        griffin_controller_.controller_parameters_.position_gain_.x(),
                        &griffin_controller_.controller_parameters_.position_gain_.x());
        GetRosParameter(private_nh_, "position_gain/y",
                        griffin_controller_.controller_parameters_.position_gain_.y(),
                        &griffin_controller_.controller_parameters_.position_gain_.y());
        GetRosParameter(private_nh_, "position_gain/z",
                        griffin_controller_.controller_parameters_.position_gain_.z(),
                        &griffin_controller_.controller_parameters_.position_gain_.z());
        GetRosParameter(private_nh_, "velocity_gain/x",
                        griffin_controller_.controller_parameters_.velocity_gain_.x(),
                        &griffin_controller_.controller_parameters_.velocity_gain_.x());
        GetRosParameter(private_nh_, "velocity_gain/y",
                        griffin_controller_.controller_parameters_.velocity_gain_.y(),
                        &griffin_controller_.controller_parameters_.velocity_gain_.y());
        GetRosParameter(private_nh_, "velocity_gain/z",
                        griffin_controller_.controller_parameters_.velocity_gain_.z(),
                        &griffin_controller_.controller_parameters_.velocity_gain_.z());
        GetRosParameter(private_nh_, "orientation_gain/x",
                        griffin_controller_.controller_parameters_.orientation_gain_.x(),
                        &griffin_controller_.controller_parameters_.orientation_gain_.x());
        GetRosParameter(private_nh_, "orientation_gain/y",
                        griffin_controller_.controller_parameters_.orientation_gain_.y(),
                        &griffin_controller_.controller_parameters_.orientation_gain_.y());
        GetRosParameter(private_nh_, "orientation_gain/z",
                        griffin_controller_.controller_parameters_.orientation_gain_.z(),
                        &griffin_controller_.controller_parameters_.orientation_gain_.z());
        GetRosParameter(private_nh_, "angular_rate_gain/x",
                        griffin_controller_.controller_parameters_.angular_rate_gain_.x(),
                        &griffin_controller_.controller_parameters_.angular_rate_gain_.x());

        GetRosParameter(private_nh_, "angular_rate_gain/y",
                        griffin_controller_.controller_parameters_.angular_rate_gain_.y(),
                        &griffin_controller_.controller_parameters_.angular_rate_gain_.y());
        GetRosParameter(private_nh_, "angular_rate_gain/z",
                        griffin_controller_.controller_parameters_.angular_rate_gain_.z(),
                        &griffin_controller_.controller_parameters_.angular_rate_gain_.z());
        GetVehicleParameters(private_nh_, &griffin_controller_.vehicle_parameters_);
    }
    void GriffinControllerNode::Publish() {
    }

    void GriffinControllerNode::CommandPoseCallback(  //Listens for a single pose (no derivatives) and controller tries to continuously achieve this pose.
            const geometry_msgs::PoseStampedConstPtr& pose_msg) {
        // Clear all pending commands.
        command_timer_.stop();
        commands_.clear();
        command_waiting_times_.clear();

        mav_msgs::EigenTrajectoryPoint eigen_reference;
        mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
        commands_.push_front(eigen_reference);

        griffin_controller_.SetTrajectoryPoint(commands_.front());
        commands_.pop_front();
    }

    void GriffinControllerNode::MultiDofJointTrajectoryCallback(
            const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) { //listens for trajectory with poses, twists, acceleraeions from a tajectory generator

        // Clear all pending commands.
        command_timer_.stop();
        commands_.clear();
        command_waiting_times_.clear();

        const size_t n_commands = msg->points.size();

        if(n_commands < 1){
            ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
            return;
        }

        mav_msgs::EigenTrajectoryPoint eigen_reference;
        mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
        commands_.push_front(eigen_reference);

        for (size_t i = 1; i < n_commands; ++i) {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
            const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

            mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

            commands_.push_back(eigen_reference);
            command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
        }

        // We can trigger the first command immediately.
        griffin_controller_.SetTrajectoryPoint(commands_.front());
        commands_.pop_front();

        if (n_commands > 1) {
            command_timer_.setPeriod(command_waiting_times_.front());
            command_waiting_times_.pop_front();
            command_timer_.start();
        }
    }

    void GriffinControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

        if(commands_.empty()){
            ROS_WARN("Commands empty, this should not happen here");
            return;
        }

        const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
        griffin_controller_.SetTrajectoryPoint(commands_.front());
        commands_.pop_front();
        command_timer_.stop();
        if(!command_waiting_times_.empty()){
            command_timer_.setPeriod(command_waiting_times_.front());
            command_waiting_times_.pop_front();
            command_timer_.start();
        }
    }

    void GriffinControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

        ROS_INFO_ONCE("GriffinController got first odometry message.");

        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);
        griffin_controller_.SetOdometry(odometry);

        Eigen::VectorXd ref_rotor_outputs;
        griffin_controller_.CalculateRotorOutputs(&ref_rotor_outputs);


        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        actuator_msg->angular_velocities.clear(); //TODO motors from 0-7,servos 8-11
        for (int i = 0; i < 3; i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_outputs[i]);
        for (int i = 0; i < 3; i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_outputs[i]);
        ROS_INFO_ONCE("First rotor output" + ref_rotor_outputs[0]);


        for(int i= 0; i < 7; i++)
            actuator_msg->angles.push_back(0);
        for (int i = 7; i < 11; i++)
            actuator_msg->angles.push_back(ref_rotor_outputs[i]);
        actuator_msg->header.stamp = odometry_msg->header.stamp;


        motor_velocity_reference_pub_.publish(actuator_msg);
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
