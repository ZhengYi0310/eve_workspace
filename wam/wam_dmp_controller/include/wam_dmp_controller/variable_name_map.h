/*************************************************************************
	> File Name: variable_name_map.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 08 Jun 2017 04:35:54 PM PDT
 ************************************************************************/

#ifndef _VARIABLE_NAME_MAP_H
#define _VARIABLE_NAME_MAP_H  

// system includes 
#include <string>
#include <vector>
#include <tr1/unordered_map>

// local includes 
namespace wam_dmp_controller 
{
    class VariableNameMap 
    {
        public:

            VariableNameMap() : initialized_(false), start_index_(0) {};
            virtual ~VariableNameMap() {};

            /*
             * @param supported_variable_names 
             * @used_variable_names 
             * @return True on success, otherwise False 
             */
            bool initialize(const std::vector<std::string>& supported_variable_names,
                            const std::vector<std::string>& used_variable_names,
                            const int start_index = 0);

            /*!
             * Uses the joint_names provided as supported_variable_names
             * @param supported_variable_names
             * return True on success, otherwise False 
             */
            bool initialize(const std::vector<std::string>& supported_variable_names,
                            const int start_index = 0);

            /*!
             * REAL-TIME REQUIREMENT
             */
            void reset();

            /*!
             * @param used_variable_names
             * @param index 
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENT
             */
            bool set(const std::string& used_variable_names, const int index);

            /*!
             * @param used_index 
             * @param supported_index 
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENT
             */
            bool getSupportedVariableIndex(const int used_index, int& supported_index) const;

            /*!
             * @param supported_index
             * @param used_index
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENT
             */
            bool getUsedVariableIndex(const int supported_index, int& used_index) const;
            
            /*!
             * Gets the index to the variable name 
             * @param name 
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENT
             */
            bool getSupportedVariableIndex(const std::string& name, int& index) const;

        private:
            bool initialized_;
            int start_index_;

            std::r1::unordered_map<std::string, int> supported_name_to_index_map_;
            std::vector<int> used_to_supported_map_;
            std::vector<int> supported_to_used_map_;
    };
}
#endif
