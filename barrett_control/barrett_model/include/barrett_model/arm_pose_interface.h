/*************************************************************************
	> File Name: arm_pose_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 30 Jun 2017 04:43:03 PM PDT
 ************************************************************************/

#ifndef _ARM_POSE_INTERFACE_H
#define _ARM_POSE_INTERFACE_H

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <cassert>
#include <string>

namespace barrett_model
{
    /** A handle to read the cartisian state of an arm.. */
    class ArmPoseStatesHandle
    {
        public:
            ArmPoseStatesHandle()
            {
                position_->reserve(3);
                orientation_->reserve(4);
                linear_velocity_->reserve(3);
                angular_velocity_->reserve(4);
            }

            ArmPoseStatesHandle(const std::string& name, const std::string* base_frame, const std::vector<double>* position, const std::vector<double>* orientation, const std::vector<double>* linear_velocity, const std::vector<double>* angular_velocity) : name_(name), base_frame_(base_frame)
            {
                if (!base_frame)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Base frame pointer is null.");
                }

                if (!position)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
                }

                if (!orientation)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Orientation data pointer is null.");
                }

                if (!linear_velocity)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Linear velocity data pointer is null.");
                }

                if (!angular_velocity)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Angular velocity data pointer is null.");
                }

                assert((position->size() == 3) && (orientation->size() == 4) && (linear_velocity->size() == 3) && (angular_velocity->size() == 4));

                position_ = position;
                orientation_ = orientation;
                linear_velocity_ = linear_velocity;
                angular_velocity_ = angular_velocity;
            }

            std::string getName() const {return name_;}
            std::string getBaseFrame() const {assert(base_frame_); return *base_frame_;}

            std::vector<double> getPosition() const {assert(position_); return *position_;}
            std::vector<double> getOrientation() const {assert(orientation_); return *orientation_;}
            std::vector<double> getLinearVelocity() const {assert(linear_velocity_); return *linear_velocity_}
            std::vector<double> getAngularVelocity() const {assert(angular_velocity_); return *angular_velocity_;}

        private:
            std::string name_;

            const std::string* base_frame_;
            const std::vector<double>* position_;
            const std::vector<double>* orientation_;
            const std::vector<double>* linear_velocity_;
            const std::vector<double>* angular_velocity_;
    };

    class ArmPoseStatesInterface : public hardware_interface::HardwareResourceManager<ArmPoseStatesHandle> {};

}
#endif
