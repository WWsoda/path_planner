//
// Created by wwsoda
//

#ifndef PATH_PLANNER_INCLUDE_TOOLS_TIME_RECORDER_H_
#define PATH_PLANNER_INCLUDE_TOOLS_TIME_RECORDER_H_
#include <vector>
#include <string>
#include <ctime>
#include "glog/logging.h"

namespace PathPlannerNS
{
    class TimeRecorder
    {
    public:
        TimeRecorder(_const std::string &title) : title_(title) {}
        void recordTime(const std::string &name);
        void printTime() const;
        void clear();

    private:
        std::string title_;
        std::vector<std::string> names_;
        std::vector<clock_t> time_stamps_;
    };
} // namespace PathPlannerNS

#endif // PATH_PLANNER_INCLUDE_TOOLS_TIME_RECORDER_H_