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
//#include <biotac_sensors/BioTacHand.h>

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
            BiotacFingerStateHandle() : bt_serial_(""), bt_position_(0), tdc_data_(0), tac_data_(0), pdc_data_(0), pac_data_(NULL), electrode_data_(NULL)
            {}; //TODO:: think about if add biotac time info 

            
            BiotacFingerStateHandle(const std::string& bt_serial, const unsigned int* bt_position, const unsigned int* tdc_data, const unsigned int* tac_data, const unsigned int* pdc_data, const std::vector<size_t>* pac_data, const std::vector<size_t>* electrode_data) : bt_serial_(bt_serial), bt_position_(bt_position), tdc_data_(tdc_data), tac_data_(tac_data), pdc_data_(pdc_data)
            {
                if (!bt_position)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bt_serial + "'. Biotac Position data pointer is null.");
                }

                if (!tdc_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot crate handle '" + bt_serial + "'. Biotac TDC data pointer is null.");
                }

                if (!tac_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot crate handle '" + bt_serial + "'. Biotac TAC data pointer is null.");                
                }

                if (!pdc_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot crate handle '" + bt_serial + "'. Biotac PDC data pointer is null");                                 
                }

                if (!pac_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot crate handle '" + bt_serial + "'. Biotac PAC data vector pointer is null.");                                
                }

                if (!electrode_data)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot crate handle '" + bt_serial + "'. Biotac Electrode data vector pointer is null.");                                
                }

                // check the sizes are equal 
                assert(pac_data->size() == NUM_PAC_DATA);
                assert(electrode_data->size() == NUM_ELECTRODE_DATA);

                pac_data_ = pac_data;
                electrode_data_ = electrode_data;
            }

            std::string getBtSerial() const {return bt_serial_;}
        
            unsigned int getBtPosition() const {assert(bt_position_); return *bt_position_;}
            unsigned int getTDCData() const {assert(tdc_data_); return *tdc_data_;}
            unsigned int getTACData() const {assert(tac_data_); return *tac_data_;}
            unsigned int getPDCData() const {assert(pdc_data_); return *pdc_data_;}

            std::vector<size_t> getPACData() const {assert(pac_data_); return *pac_data_;}
            std::vector<size_t> getElectrodeData() const {assert(electrode_data_); return *electrode_data_;}

        private:
            std::string bt_serial_;

            const unsigned int* bt_position_;
            const unsigned int* tdc_data_;
            const unsigned int* tac_data_;
            const unsigned int* pdc_data_;

            const std::vector<size_t>* pac_data_;
            const std::vector<size_t>* electrode_data_;
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
