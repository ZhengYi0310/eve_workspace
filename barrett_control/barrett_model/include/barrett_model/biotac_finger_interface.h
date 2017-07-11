/*************************************************************************
	> File Name: biotac_finger_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 30 Jun 2017 10:19:14 AM PDT
 ************************************************************************/

#ifndef _BIOTAC_HAND_INTERFACE_H
#define _BIOTAC_HAND_INTERFACE_H
//#include <cstdint>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <biotac_sensors/BioTacHand.h>
#include <biotac_sensors/BioTacData.h>
#include <biotac_sensors/BioTacTime.h>

#include <cassert>
#include <string>

#define NUM_PAC_DATA 22
#define NUM_ELECTRODE_DATA 19

namespace barrett_model
{
    /** A handle used to read the state of a biotac sensor or a hand ??. */
    class BiotacFingerStateHandle 
    {
        public:
            BiotacFingerStateHandle() : bt_serial_(""), bt_data_(NULL)
            {}; //TODO:: think about if add biotac time info 

            
            BiotacFingerStateHandle(const std::string& bt_serial, const biotac_sensors::BioTacData* bt_data) : bt_serial_(bt_serial)
            {
                if (!bt_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bt_serial + "'. Biotac data pointer is null.");
                }

                // check the sizes are equal 
                assert((bt_data->pac_data).size() == NUM_PAC_DATA);
                assert((bt_data->electrode_data).size() == NUM_ELECTRODE_DATA);

                bt_data_ = bt_data;
            }

            std::string getName() const {return bt_serial_;}
			
			biotac_sensors::BioTacData getCompleteData() const {assert(bt_data_); return *bt_data_;}
			std::string getBtSerial() const {assert(bt_data_); assert(bt_serial_ == bt_data_->bt_serial); return bt_data_->bt_serial;}
            uint16_t getBtPosition() const {assert(bt_data_); return bt_data_->bt_position;}
            uint16_t getTDCData() const {assert(bt_data_); return bt_data_->tdc_data;}
            uint16_t getTACData() const {assert(bt_data_); return bt_data_->tac_data;}
            uint16_t getPDCData() const {assert(bt_data_); return bt_data_->pdc_data;}
			uint16_t getPACDataWithIndex(size_t index) {assert(bt_data_); assert((index >= 0) && (index < 22)); return (bt_data_->pac_data)[index];}
            uint16_t getElectrodeDataWithIndex(size_t index) {assert(bt_data_); assert((index >= 0) && (index < 19)); return (bt_data_->electrode_data)[index];}

            //std::vector<uint16_t> getPACData() const {assert(bt_data_); return bt_data_->pac_data;}
            //std::vector<uint16_t> getElectrodeData() const {assert(bt_data_); return bt_data_->electrode_data;}

        private:
            std::string bt_serial_;
			const biotac_sensors::BioTacData* bt_data_;
    };

    /** \brief Hardware interface to support reading the state of an array of joints
     *
     * This \ref HardwareInterface supports reading the state of an array of named
     * joints, each of which has some position, velocity, and effort (force or
     * torque).
     *
     */
     class BiotacFingerStateInterface : public hardware_interface::HardwareResourceManager<BiotacFingerStateHandle> {};
}
#endif
