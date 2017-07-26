/*************************************************************************
	> File Name: dmp_joint_position_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 06 Jun 2017 02:00:25 PM PDT
 ************************************************************************/

// system includes 
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <robot_info/robot_info.h>
#include <usc_utilities/assert.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes 
#include <wam_dmp_controller/dmp_joint_position_controller.h>
#include <wam_dmp_controller/dmp_controller.h>
#include <wam_dmp_controller/dmp_controller_implementation.h>

using namespace Eigen;

namespace wam_dmp_controller
{
    bool DMPJointPositionController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
    {
        ROS_INFO("Initializing DMP joint position controller.");

        // initialize joint position controllers
        joint_position_controllers_.clear();
        std::vector<std::string> joint_names;
        std::vector<std::string> controlled_joint_names;
        ROS_VERIFY(usc_utilities::read(node_handle, "joint_names", joint_names));
        for (int i = 0; i < (int)joint_names.size(); ++i)
        {
            controlled_joint_names.push_back(joint_names[i]);
            ros::NodeHandle joint_node_handle(node_handle, joint_names[i]);
            boost::shared_ptr<JointPositionController> joint_position_controller(boost::make_shared<JointPositionController>());
            //JointPositionController joint_position_controller;
            if (!joint_position_controller->init(hw, joint_node_handle))
            {
                ROS_ERROR("Could not initialize joint controller for joint >%s<.", joint_names[i].c_str());
                return false;
            }
            
            joint_position_controllers_.push_back(joint_position_controller);
        }

        std::string dmp_implementation;
        ROS_VERIFY(usc_utilities::read(node_handle, "dmp_implementation", dmp_implementation));
        if (dmp_implementation == "NC2010DMPControllerImplementation")
        {
            dmp_controller_.reset(new DMPControllerImplementation<dmp::NC2010DMP>());
        }
        else 
        {
            ROS_ERROR("Could not figure what which DMPController implementation to use.");
            return false;
        }

        ROS_VERIFY(dmp_controller_->initialize(node_handle.getNamespace(), controlled_joint_names));

        // initialize member 
        num_joints_ = controlled_joint_names.size();
        desired_positions_ = Eigen::VectorXd::Zero(num_joints_);
        desired_velocities_ = Eigen::VectorXd::Zero(num_joints_);
        desired_accelerations_ = Eigen::VectorXd::Zero(num_joints_);

        ROS_INFO(">%s< initialized with >%i< joints.", dmp_implementation.c_str(), num_joints_);
        for (int i = 0; i < num_joints_; i++)
        {
            ROS_INFO(">%s<", controlled_joint_names[i].c_str());
        }

        return true;
    }

    // REAL-TIME REQUIREMENTS
    void DMPJointPositionController::starting(const ros::Time &time)
    {
        ROS_DEBUG("Starting...");
        for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); i++)
        {
            joint_position_controllers_[i]->starting(time);
            //joint_position_controllers_[i].starting(time);
        }
        holdPositions();
    }

    // REAL-TIME REQUIREMENTS
    void DMPJointPositionController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (dmp_controller_->newDMPReady())
        {
            // set start of DMP to current desited position 
            ROS_VERIFY(dmp_controller_->changeDMPStart(desired_positions_));
        }
        if (dmp_controller_->isRunning(desired_positions_, desired_velocities_, desired_accelerations_))
        {
            setDesiredState();
        }
        else
        {
            holdPositions();
        }

        for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); i++)
        {
            joint_position_controllers_[i]->update(time, period);
            //joint_position_controllers_[i].update(time, period);
        }
    }

    // REAL-TIME REQUIREMENTS
    void DMPJointPositionController::setDesiredState()
    {
        for (int i = 0; i < num_joints_; i++)
        {
            joint_position_controllers_[i]->setCommand(desired_positions_(i)); //desired_velocities_(i) ????
            //joint_position_controllers_[i].setCommand(desired_positions_(i));
        }
    }
    
    void DMPJointPositionController::getDesiredPosition()
    {
        for (int i = 0; i < num_joints_; i++)
        {
            desired_positions_(i) = joint_position_controllers_[i]->getJointPosition();
            //desired_positions_(i) = joint_position_controllers_[i].getJointPosition();
        }
    }
    
    // REAL-TIME REQUIREMENTS
    void DMPJointPositionController::holdPositions()
    {
        getDesiredPosition();
        desired_velocities_.setZero(num_joints_);
        desired_accelerations_.setZero(num_joints_);
        setDesiredState();
    }
}
PLUGINLIB_DECLARE_CLASS(wam_dmp_controller, DMPJointPositionController,
    wam_dmp_controller::DMPJointPositionController, controller_interface::ControllerBase)