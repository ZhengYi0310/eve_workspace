/*************************************************************************
	> File Name: biotac_state_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 06 Jul 2017 10:58:27 AM PDT
 ************************************************************************/

#include <algorithm>
#include <cstddef>

#include "barrett_hw/biotac_state_controller.h"

namespace biotac_state_controller 
{
    bool BioTacStateController::init(barrett_model::BiotacFingerStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // get all serial numbes from the hardware interface 
        const std::vector<std::string>& serial_numbers = hw->getNames();
        num_biotac_fingers_ = serial_numbers.size();
        for (size_t i = 0; i < num_biotac_fingers_; i++)
        {
            ROS_INFO("Got biotac sensor %s", serial_numbers[i].c_str());
        }

        // get the publish period 
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set!");
            return false;
        }

        // realtime publisher 
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand>(root_nh, "biotac_states", 4));

        // get biotac finger and allocate message 
        biotac_sensors::BioTacHand bt_hand_msg;
        biotac_sensors::BioTacData bt_data_msg;
        biotac_sensors::BioTacTime bt_time_msg;
        for (size_t i = 0; i < num_biotac_fingers_; i++)
        {
            biotac_state_.push_back(hw->getHandle(serial_numbers[i]));
            realtime_pub_->msg_.bt_data.push_back(bt_data_msg);
        }
        realtime_pub_->msg_.header = bt_hand_msg.header;
        realtime_pub_->msg_.hand_id = bt_hand_msg.hand_id;
        realtime_pub_->msg_.bt_time = bt_time_msg;

        return true;
    }

    void BioTacStateController::starting(const ros::Time& time)
    {
        //initialize time 
        last_publish_time_ = time;
    }

    void BioTacStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
    {
        // limit the rate of publishing 
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            // try to publish 
            if (realtime_pub_->trylock())
            {
                // increment time since we're actually publishing 
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
                // populate the biotac fingers present in BioTacFingerStateInterface, i.e. indices [0, num_biotac_fingers)
                realtime_pub_->msg_.header.stamp = time;
                for (size_t i = 0; i < num_biotac_fingers_; i++)
                {
                    // assign the bitac data first 
                    biotac_sensors::BioTacData bt_data;
                    bt_data.bt_serial = biotac_state_[i].getBtSerial();
                    bt_data.bt_position = biotac_state_[i].getBtPosition();
                    bt_data.tdc_data = biotac_state_[i].getTDCData();
                    bt_data.tac_data = biotac_state_[i].getTACData();
                    bt_data.pdc_data = biotac_state_[i].getPDCData();
                    ROS_INFO("filling pac and electrode data...");
                    
                    for (size_t j = 0; j < 22; j++)
                    {
                        bt_data.pac_data[j] = (biotac_state_[i].getPACData())[j];
                    }

                    for (size_t j = 0; j < 19; j++)
                    {
                        bt_data.electrode_data[j] = (biotac_state_[i].getElectrodeData())[j];
                    }
                    
                }
                realtime_pub_->unlockAndPublish();
            }
        }
    }

    void BioTacStateController::stopping(const ros::Time& /*time*/)
    {}
}
PLUGINLIB_EXPORT_CLASS(biotac_state_controller::BioTacStateController, controller_interface::ControllerBase)

