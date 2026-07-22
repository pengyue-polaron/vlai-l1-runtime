#pragma once

#include <linux/can/netlink.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace vlai_l1 {

struct CanHealthSnapshot {
    can_state state = CAN_STATE_MAX;
    can_berr_counter live_errors{};
    can_device_stats cumulative{};
};

int bounded_fifo_priority(int requested, int configured_cap) noexcept;
void configure_fifo_priority_cap(int priority);
void set_and_require_all_threads_fifo(int priority, std::size_t minimum_thread_count);
void require_all_threads_fifo(int priority, std::size_t minimum_thread_count);

void require_can_health(const std::string& interface, const CanHealthSnapshot& baseline,
                        const CanHealthSnapshot& current);

class CanHealthMonitor {
public:
    explicit CanHealthMonitor(std::vector<std::string> interfaces);
    CanHealthMonitor(const CanHealthMonitor&) = delete;
    CanHealthMonitor& operator=(const CanHealthMonitor&) = delete;
    ~CanHealthMonitor();

    void check();

private:
    CanHealthSnapshot read(const std::string& interface);

    int socket_ = -1;
    std::uint32_t sequence_ = 0;
    std::vector<std::string> interfaces_;
    std::unordered_map<std::string, CanHealthSnapshot> baselines_;
};

}  // namespace vlai_l1
