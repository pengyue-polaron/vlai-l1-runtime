#include "runtime_safety.hpp"

#include <pthread.h>
#include <sched.h>

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
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS runtime safety contracts\n";
    return 0;
}
