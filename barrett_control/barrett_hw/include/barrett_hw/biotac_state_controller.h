/*************************************************************************
	> File Name: biotac_state_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 06 Jul 2017 10:36:27 AM PDT
 ************************************************************************/

#ifndef _BIOTAC_STATE_CONTROLLER_H
#define _BIOTAC_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <barrett_model/biotac_finger_interface.h>

#include <biotac_sensors/BioTacData.h>
#include <biotac_sensors/BioTacHand.h>
#include <biotac_sensors/BioTacTime.h>

namespace biotac_state_controller 
{
    /**
     * \brief Controller that publishes the state of all biotac devices on a robot.
     *
     * This controller publishes the state of all resources registered to a \c barrett_model::biotac_finger_interface to a
     * topic of type \c biotac_sensors/BioTacHand. The following is a basic configuration of the controller.
     *
     * \code
     * biotac_state_controller:
     *   type: barrett_model/BioTacStateController
     *   publish_rate: 50
     * \endcode
     */
    class BioTacStateController : public controller_interface::Controller<barrett_model::BiotacFingerStateInterface>
    {
        public:
            BioTacStateController() : publish_rate_(0) {};
            virtual bool init(barrett_model::BiotacFingerStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
            virtual void stopping(const ros::Time& /*time*/);

        private:
            std::vector<barrett_model::BiotacFingerStateHandle> biotac_state_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand> > realtime_pub_;
            ros::Time last_publish_time_;
            double publish_rate_;
            unsigned int num_biotac_fingers_; /// Number of biotac fingers present in the BiotacFingerStateInterface
    };
}


#endif
