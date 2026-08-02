#include "runtime_safety.hpp"

#include <pthread.h>
#include <sched.h>

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_can_failure(const vlai_l1::CanHealthSnapshot& baseline,
                        const std::function<void(vlai_l1::CanHealthSnapshot&)>& mutate) {
    auto current = baseline;
    mutate(current);
    try {
        vlai_l1::require_can_health("can-test", baseline, current);
    } catch (const std::runtime_error&) {
        return;
    }
    throw std::runtime_error("unhealthy CAN transition was accepted");
}

vlai_l1::JointSafetyLimits joint_limits() {
    vlai_l1::JointSafetyLimits limits;
    limits.min_deg.fill(-90.0);
    limits.max_deg.fill(90.0);
    limits.max_following_error_deg.fill(10.0);
    limits.following_error_timeout_ns = 100'000'000;
    return limits;
}

double radians(double degrees) { return degrees * 3.14159265358979323846 / 180.0; }

}  // namespace

int main() {
    try {
        require(vlai_l1::bounded_fifo_priority(50, 20) == 20,
                "FIFO requests above the tracked value must be capped");
        require(vlai_l1::bounded_fifo_priority(10, 20) == 10,
                "FIFO requests below the tracked value must be preserved");
        require(vlai_l1::bounded_fifo_priority(50, 0) == 50,
                "a disabled cap must preserve the request");
        vlai_l1::configure_fifo_priority_cap(20);
        const sched_param normal_priority{0};
        require(pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal_priority) == 0,
                "pthread scheduling interposition must resolve the host implementation");

        vlai_l1::CanHealthSnapshot baseline;
        baseline.state = CAN_STATE_ERROR_ACTIVE;
        vlai_l1::require_can_health("can-test", baseline, baseline);

        std::vector<std::function<void(vlai_l1::CanHealthSnapshot&)>> faults{
            [](auto& value) { value.state = CAN_STATE_ERROR_WARNING; },
            [](auto& value) { value.live_errors.txerr = 1; },
            [](auto& value) { value.live_errors.rxerr = 1; },
            [](auto& value) { value.cumulative.bus_error = 1; },
            [](auto& value) { value.cumulative.error_warning = 1; },
            [](auto& value) { value.cumulative.error_passive = 1; },
            [](auto& value) { value.cumulative.bus_off = 1; },
            [](auto& value) { value.cumulative.arbitration_lost = 1; },
            [](auto& value) { value.cumulative.restarts = 1; },
        };
        for (const auto& fault : faults) {
            expect_can_failure(baseline, fault);
        }

        auto unhealthy_baseline = baseline;
        unhealthy_baseline.live_errors.txerr = 1;
        expect_can_failure(unhealthy_baseline, [](auto&) {});

        std::array<double, vlai_l1::kArmJointCount> leader{};
        std::array<double, vlai_l1::kArmJointCount> follower{};
        vlai_l1::JointSafetyMonitor normal("right", joint_limits());
        require(!normal.observe(1, leader, follower).has_value(),
                "in-range matching joints must pass");

        auto outside = leader;
        outside[3] = radians(91.0);
        vlai_l1::JointSafetyMonitor bound_monitor("right", joint_limits());
        const auto bound_fault = bound_monitor.observe(1, outside, follower);
        require(bound_fault.has_value() && bound_fault->fatal &&
                    bound_fault->detail.find("right joint_4 leader position") !=
                        std::string::npos,
                "an out-of-range joint must fail with side, joint, and role detail");

        leader[2] = radians(20.0);
        vlai_l1::JointSafetyMonitor following_monitor("right", joint_limits());
        require(!following_monitor.observe(1, leader, follower).has_value(),
                "a new following error must start its bounded grace period");
        require(!following_monitor.observe(99'000'001, leader, follower).has_value(),
                "a following error shorter than the timeout must not fail");
        const auto following_fault =
            following_monitor.observe(100'000'001, leader, follower);
        require(following_fault.has_value() && following_fault->fatal &&
                    following_fault->detail.find("right joint_3 following error") !=
                        std::string::npos,
                "a sustained following error must identify the affected joint");

        auto warning_limits = joint_limits();
        warning_limits.stop_on_following_error = false;
        vlai_l1::JointSafetyMonitor warning_monitor("right", warning_limits);
        require(!warning_monitor.observe(1, leader, follower).has_value(),
                "warning-only following error must retain its grace period");
        const auto warning = warning_monitor.observe(100'000'001, leader, follower);
        require(warning.has_value() && !warning->fatal &&
                    warning->detail.find("right joint_3 following error") !=
                        std::string::npos,
                "warning-only following error must emit one nonfatal event");
        require(!warning_monitor.observe(120'000'001, leader, follower).has_value(),
                "warning-only following error must not flood repeated events");

        vlai_l1::JointSafetyMonitor cleared_monitor("left", joint_limits());
        require(!cleared_monitor.observe(1, leader, follower).has_value(),
                "a following-error timer must start");
        leader[2] = radians(5.0);
        require(!cleared_monitor.observe(50'000'001, leader, follower).has_value(),
                "a recovered joint must clear its following-error timer");
        leader[2] = radians(20.0);
        require(!cleared_monitor.observe(150'000'001, leader, follower).has_value(),
                "a later following error must receive a fresh grace period");
        require(!cleared_monitor.observe(249'000'001, leader, follower).has_value(),
                "a fresh following error must use its own start time");

        const auto timestamp_fault =
            cleared_monitor.observe(249'000'001, leader, follower);
        require(timestamp_fault.has_value() && timestamp_fault->fatal &&
                    timestamp_fault->detail.find("timestamp did not increase") !=
                        std::string::npos,
                "non-increasing joint safety timestamps must fail closed");
        cleared_monitor.reset();
        require(!cleared_monitor.observe(1, follower, follower).has_value(),
                "reset must clear safety timing state after AdjustPosition");
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS runtime safety contracts\n";
    return 0;
}
