/*************************************************************************
	> File Name: task_recorder_utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:57:50 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_UTILITIES_H
#define _TASK_RECORDER_UTILITIES_H

// system includes 
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// ros includes 
namespace task_recorder 
{
    const std::string TRIAL_FILE_NAME_APPENDIX(".txt");
    const std::string TRIAL_FILE_NAME("_trial_counter" + TRIAL_FILE_NAME_APPENDIX);

    const std::string FILE_NAME_ID_SEPERATPR("_");
    const std::string FILE_NAME_DATA_TRUNK("data" + FILE_NAME_ID_SEPERATPR);
    const std::string FILE_NAME_STATISTICS_TRUNK("stat" + FILE_NAME_ID_SEPERATPR);

    const std::string BAG_FILE_APPENDIX(".bag");

    inline std::string getString(const int number)
    {
        std::stringstream ss;
        ss << number;
        return ss.str();
    }

    inline bool getTopicName(std::string& topic_name)
    {
        size_t topic_name_pos = topic_name.find_first_of("/");
        if (topic_name_pos == std::string::npos)
        {
            return false;
        }

        size_t start = topic_name_pos + 1;
        size_t length = topic_name.length() - start;
        topic_name = topic_name.substr(start, length);

        size_t slash_pos = topic_name.find_first_of("/");
        while (slash_pos != std::string::npos)
        {
            topic_name = topic_name.replace(slash_pos, 1, std::string("_"));
            slash_pos = topic_name.find_first_of("/");
        }
        return true;
    }
}
#endif
