#include <vectortricopter/scheduler.hpp>
#include <vectortricopter/world.hpp>

#include <cassert>
#include <chrono>

namespace tilt {
namespace tricopter {

Scheduler *Scheduler::globalInstance = nullptr;

Scheduler& Scheduler::GetInstance() {
    if (globalInstance == nullptr) {
        globalInstance = new Scheduler();
    }

    assert(globalInstance != nullptr);

    return *globalInstance;
}

void Scheduler::Init(const std::vector<Task>& tasks) {
    std::unique_lock<std::mutex> lock(_mutex);
    assert(!_initialized);
    _tasks = tasks;
    _stop_requested.store(false);
    ScheduleTasks();
    _initialized = true;
}

Scheduler::~Scheduler() {
    Shutdown();
}

void Scheduler::Shutdown() {
    _stop_requested.store(true);
    if (_main_loop.joinable()) {
        _main_loop.join();
    }
}

void Scheduler::ScheduleTasks() {
    _main_loop = std::thread(
        std::bind(&Scheduler::MainLoop, this)
    );
}

void Scheduler::MainLoop() {
    while (!_stop_requested.load() && rclcpp::ok()) {
        const size_t ntasks = _tasks.size();

        const double schedule_start_time = GetCurrentTimeSeconds();
        double now = schedule_start_time;
        for (size_t n = 0; n < ntasks; ++n) {
            Task& task = _tasks[n];
            
            bool should_schedule = !task.hasScheduled || task.updateRateHZ == 0;

            now = GetCurrentTimeSeconds();

            if (!should_schedule) {
                const double dt = now - task.lastScheduledTime;
                should_schedule = dt >= 1. / (double) task.updateRateHZ;
            }

            if (should_schedule) {
                task.hasScheduled = true;
                task.lastScheduledTime = now;
                task.func();
            }
        }

        // Keep the original high-rate scheduler semantics while yielding to
        // ROS 2 executor and Gazebo threads instead of busy-spinning a core.
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

}
}
