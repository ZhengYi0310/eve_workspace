/*************************************************************************
	> File Name: robot_state_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 11 Jul 2017 09:28:02 PM PDT
 ************************************************************************/

#ifndef _ROBOT_STATE_INTERFACE_H
#define _ROBOT_STATE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <cassert>
#include <string>

#include <hardware_interface/joint_state_interface.h>
#include "arm_pose_interface.h"

namespace barrett_model 
{
    /** A handle to read the joint + cartesian + biotac(optional) state of the arm.. */
    class RobotStateHandle
    {
        public:
            RobotStateHandle() : name_(""), arm_joint_state_interface_(NULL), arm_cartesian_state_interface_(NULL)
            {};

        	RobotStateHandle(const std::string& name, const hardware_interface::JointStateInterface* arm_joint_state_interface, const barrett_model::ArmPoseStatesInterface* arm_cartesian_state_interface) : name_(name) 
        	{
            	if (!arm_joint_state_interface)
            	{
                	throw hardware_interface::HardwareInterfaceException("Cannot create handle for robot'" + name + "'. joint_state_interface is empty.");                 
            	}

            	if (!arm_cartesian_state_interface)
            	{
                	throw hardware_interface::HardwareInterfaceException("Cannot create handle for robot'" + name + "'. arm_pose_states_interface is empty.");                                 
            	}

            	arm_joint_state_interface_ = arm_joint_state_interface;
            	arm_cartesian_state_interface_ = arm_cartesian_state_interface;
        	}

        	std::string getName() const {return name_;}
        	hardware_interface::JointStateInterface getJointStateInterface() const {assert(arm_joint_state_interface_); return *arm_joint_state_interface_;}
        	barrett_model::ArmPoseStatesInterface getArmPoseStatesInterface() const {assert(arm_cartesian_state_interface_); return *arm_cartesian_state_interface_;}
        	
		
		private:
			std::string name_;
			const hardware_interface::JointStateInterface* arm_joint_state_interface_;
			const barrett_model::ArmPoseStatesInterface* arm_cartesian_state_interface_;
    };
	class RobotStateInterface : public hardware_interface::HardwareResourceManager<RobotStateHandle> {};
}

#endif