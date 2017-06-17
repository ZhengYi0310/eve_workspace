/*************************************************************************
	> File Name: task_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:24:43 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_H
#define _TASK_RECORDER_H

// system includes 
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ros includes 
#include <ros/ros.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>

// local includes 
#include <task_recorder/task_recorder_io.h>
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/accumulator.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder 
{
    template <class MessageType, class StopServiceType>
    class TaskRecorder 
    {
        public:
            
            typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
            typedef boost::shared_ptr<StopServiceType const> StopServiceTypeConstPtr;

            TaskRecorder() : initialized_(false), is_recording_(false) {};
            virtual ~TaskRecorder() {};

            /*!
             * @param node_handle 
             * @param topic_name 
             * return True on success, otherwise False 
             */
            bool initializeBase(ros::NodeHandle& node_handle,
                                const std::string& topic_name);

            void startRecording();

            void stopRecording();

            /*!
             * @param request 
             * @param response 
             * @return True on success, otherwise False 
             */
            bool startRecording(task_recorder::StartRecording::Request& request,
                                task_recorder::StartRecording::Response& response);

            /*!
             * @param request 
             * @param response 
             * @return True on success, otherwise False 
             */
            bool stopRecording(typename StopServiceType::Request& request,
                               typename StopServiceType::Response& response);

        protected:

            bool initialized_;
            bool is_recording_;
            
            TaskRecorderIO<MessageType> recorder_io_;

            /*!
             * @param start_time
             * @param end_time 
             * @param num_samples
             * @param filter_and_cropped_messages
             * @param message_names
             * @param times
             * @param data 
             * @return True on success, otherwise False 
             */
            virtual bool filterAndCrop(const ros::Time start_time,
                                       const ros::Time end_time,
                                       int num_samples,
                                       std::vector<MessageType>& filter_and_cropped_messages,
                                       std::vector<std::string>& message_names,
                                       std::vector<ros::Time> times,
                                       std::vector<double>& data);

            /*!
             * @param message 
             * @return True on success, otherwise False 
             */
            virtual bool transformMessage(MessageType& message);

            /*!
             * @pram vector_of_accumulated_statistics
             * @return True on success, otherwise False 
             */
            virtual bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_statistics_vector);

            /*!
             * @param accumulated_trial_statistics 
             */
            virtual void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics) = 0;

            /*!
             * @param signal_index
             * @param singal_name 
             */
            virtual void getSignalNames(const int signal_index,
                                        std::string& signal_name) = 0;
            virtual void getVariables(const MessageType& message, std::vector<double>& variables) = 0;
            virtual void getVariables(MessageType& message, const std::vector<double>& variables) = 0;
            virtual int getNumVariables() = 0;

        private:

            ros::ServiceServer start_recording_service_server_;
            ros::ServiceServer stop_recording_service_server_;

            ros::Subscriber subscriber_;

            boost::mutex mutex_;
            bool logging_;

            task_recorder::Accumulator accumulator_;

            void recordMessageCallback(const MessageTypeConstPtr message);

            bool resample(std::vector<MessageType>& messages,
                          const ros::Time& start_time,
                          const ros::Time& end_time,
                          const int num_samples,
                          std::vector<MessageType>& resampled_messages,
                          bool use_bspline=false);
    };
}
#endif
