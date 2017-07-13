/*************************************************************************
	> File Name: robot_states_publisher.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 12 Jul 2017 10:52:51 AM PDT
 ************************************************************************/

#include<iostream>
#include <algorithm>
#include "barrett_hw/robot_states_publisher.h"

namespace robot_states_publisher 
{
    bool RobotStatesPublisher::init(barrett_model::RobotStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set!");
            return false;
        }

        if (!controller_nh.getParam("tactile", biotac_sensors_exist_))
        {
            ROS_ERROR("Parameter 'tactile' not set!");
            return false;
        }

        const std::vector<std::string>& robot_names = hw->getNames();
        num_robots_ = robot_names.size();
        for (int i = 0; i < num_robots_; i++)
        {
            ROS_INFO("Get a robot with name >>%s<<.", robot_names[i].c_str());
            robot_state_handle_.push_back(hw->getHandle(robot_names[i]));
        }
        const std::vector<std::string>& joint_names = (hw->getHandle(robot_names[0])).getJointStateInterface().getNames();
        num_joints_ = joint_names.size();
        for (int i = 0; i < num_joints_; i++)
        {
            ROS_INFO("The robot has a joint with name >>%s<<.", joint_names[i].c_str());
            joint_state_handles_.push_back((hw->getHandle(robot_names[0])).getJointStateInterface().getHandle(joint_names[i]));
        }

        const std::vector<std::string>& cartesian_pose_names = (hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getNames();
        num_cartesian_pose_ = cartesian_pose_names.size();
        for (int i = 0; i < num_cartesian_pose_; i++)
        {
            ROS_INFO("The robot has a base frame with name >>%s<<.", (hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getHandle(cartesian_pose_names[i]).getBaseFrame().c_str());
            arm_cartesian_state_handles_.push_back((hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getHandle(cartesian_pose_names[i]));
        }



        if (biotac_sensors_exist_)
        {
            realtime_pub_tactile_.reset(new realtime_tools::RealtimePublisher<barrett_hw::robot_states_tactile>(root_nh, "robot_states_tactile", 10));
            realtime_pub_tactile_->msg_.base_frame =  arm_cartesian_state_handles_[0].getBaseFrame(); 
            for (int i = 0; i < num_joints_; i++)
            {
                realtime_pub_tactile_->msg_.joint_names.push_back(joint_names[i]);
                realtime_pub_tactile_->msg_.joint_positions.push_back(0.0);
                realtime_pub_tactile_->msg_.joint_velocities.push_back(0.0);
                realtime_pub_tactile_->msg_.joint_efforts.push_back(0.0);                
            }
            biotac_hand_.reset(new biotac::BioTacHandClass("left_hand_biotacs"));
            biotac_hand_->initBioTacSensors();
        }
        else 
        {
            realtime_pub_no_tactile_.reset(new realtime_tools::RealtimePublisher<barrett_hw::robot_states_no_tactile>(root_nh, "robot_states_no_tactile", 10));
            realtime_pub_no_tactile_->msg_.base_frame =  arm_cartesian_state_handles_[0].getBaseFrame(); 
            for (int i = 0; i < num_joints_; i++)
            {
                realtime_pub_no_tactile_->msg_.joint_names.push_back(joint_names[i]);
                realtime_pub_no_tactile_->msg_.joint_positions.push_back(0.0);
                realtime_pub_no_tactile_->msg_.joint_velocities.push_back(0.0);
                realtime_pub_no_tactile_->msg_.joint_efforts.push_back(0.0);
            }
            
        }


        return true;
    }

    void RobotStatesPublisher::starting(const ros::Time& time)
    {
        last_publish_time_ = time;
        controller_start_time_ = time;
    }

    void RobotStatesPublisher::update(const ros::Time& time, const ros::Duration& /*period*/)
    {
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
            if (!biotac_sensors_exist_)
            {
                if (realtime_pub_no_tactile_->trylock())
                {
                    ros::Time duration((time - controller_start_time_).toSec());
                    realtime_pub_no_tactile_->msg_.header.stamp = duration;
                    tf::poseKDLToMsg(arm_cartesian_state_handles_[0].getPose(),  realtime_pub_no_tactile_->msg_.Pose);
                    tf::twistKDLToMsg(arm_cartesian_state_handles_[0].getTwist(), realtime_pub_no_tactile_->msg_.Twist);
                    for (int i = 0; i < num_joints_; i++)
                    {
                        realtime_pub_no_tactile_->msg_.joint_positions[i] = joint_state_handles_[i].getPosition();
                        realtime_pub_no_tactile_->msg_.joint_velocities[i] = joint_state_handles_[i].getVelocity();
                        realtime_pub_no_tactile_->msg_.joint_efforts[i] = joint_state_handles_[i].getEffort();
                    }
                    realtime_pub_no_tactile_->unlockAndPublish();
                }
            }
            else 
            {
                if (realtime_pub_tactile_->trylock())
                {
                    ros::Time duration((time - controller_start_time_).toSec());
                    realtime_pub_tactile_->msg_.header.stamp = duration;
                    tf::poseKDLToMsg(arm_cartesian_state_handles_[0].getPose(),  realtime_pub_tactile_->msg_.Pose);
                    tf::twistKDLToMsg(arm_cartesian_state_handles_[0].getTwist(), realtime_pub_tactile_->msg_.Twist);
                    for (int i = 0; i < num_joints_; i++)
                    {
                        realtime_pub_tactile_->msg_.joint_positions[i] = joint_state_handles_[i].getPosition();
                        realtime_pub_tactile_->msg_.joint_velocities[i] = joint_state_handles_[i].getVelocity();
                        realtime_pub_tactile_->msg_.joint_efforts[i] = joint_state_handles_[i].getEffort();
                    }
                    realtime_pub_tactile_->msg_.biotac_hand = biotac_hand_->collectBatch();
                    realtime_pub_tactile_->msg_.biotac_hand.header.stamp = ros::Time((realtime_pub_tactile_->msg_.biotac_hand.header.stamp - controller_start_time_).toSec());
                    realtime_pub_tactile_->msg_.biotac_hand.bt_time.frame_start_time = ros::Time((realtime_pub_tactile_->msg_.biotac_hand.bt_time.frame_start_time - controller_start_time_).toSec());
                    realtime_pub_tactile_->msg_.biotac_hand.bt_time.frame_end_time = ros::Time((realtime_pub_tactile_->msg_.biotac_hand.bt_time.frame_end_time - controller_start_time_).toSec());                                   
                    realtime_pub_tactile_->unlockAndPublish();
                }                 
            }
        }
    }

    void RobotStatesPublisher::stopping(const ros::Time& /*time*/)
    {}
}
PLUGINLIB_EXPORT_CLASS(robot_states_publisher::RobotStatesPublisher, controller_interface::ControllerBase)

