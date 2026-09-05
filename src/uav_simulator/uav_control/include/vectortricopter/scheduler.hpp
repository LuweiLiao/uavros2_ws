#pragma once

#include <stdint.h>
#include <functional>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

#define FAST_TASK_CLASS(clsInstancePtr, clsName, funcName) \
    ::tilt::tricopter::Scheduler::Task { \
        .updateRateHZ = 0, \
        .hasScheduled = false, \
        .lastScheduledTime = 0., \
        .func = std::bind(&clsName::funcName, clsInstancePtr) \
    }

#define SCHED_TASK_CLASS(clsInstancePtr, clsName, funcName, updateRate) \
    ::tilt::tricopter::Scheduler::Task { \
        .updateRateHZ = updateRate, \
        .hasScheduled = false, \
        .lastScheduledTime = 0., \
        .func = std::bind(&clsName::funcName, clsInstancePtr) \
    }

namespace tilt {
namespace tricopter {

class Scheduler {
public:
    struct Task {
        uint64_t updateRateHZ{0};
        bool hasScheduled{false};
        double lastScheduledTime{0.};
        std::function<void ()> func;
    };

    static Scheduler& GetInstance();

    ~Scheduler();

    void Init(const std::vector<Task>& tasks);

    // Stop and join the scheduler before rclcpp or the shared node is
    // destroyed.  ROS 1 terminated with the process; ROS 2 needs this
    // explicit lifecycle boundary for a clean SIGINT shutdown.
    void Shutdown();

private:
    void ScheduleTasks();

    void MainLoop();

private:
    // The global scheduler instacne
    static Scheduler *globalInstance;

    // The thread running the main loop
    std::thread _main_loop;

    // The threads to be scheduled
    std::vector<Task> _tasks;

    // Whether or not the 
    bool _initialized{false};

    std::atomic_bool _stop_requested{false};

    std::mutex _mutex;

    Scheduler() = default;

    Scheduler(const Scheduler&) = delete;
    Scheduler(Scheduler&&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    Scheduler& operator=(Scheduler&&) = delete;
    
};

}
}
