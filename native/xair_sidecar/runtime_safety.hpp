#pragma once

#include <linux/can/netlink.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace vlai_l1 {

constexpr std::size_t kArmJointCount = 7;

struct JointSafetyLimits {
    std::array<double, kArmJointCount> min_deg{};
    std::array<double, kArmJointCount> max_deg{};
    std::array<double, kArmJointCount> max_following_error_deg{};
    std::uint64_t following_error_timeout_ns = 0;
    bool stop_on_following_error = true;
};

struct JointSafetyEvent {
    bool fatal = true;
    std::string detail;
};

class JointSafetyMonitor {
public:
    JointSafetyMonitor(std::string side, JointSafetyLimits limits);

    std::optional<JointSafetyEvent> observe(
        std::uint64_t monotonic_ns,
        const std::array<double, kArmJointCount>& leader_radians,
        const std::array<double, kArmJointCount>& follower_radians);
    void reset() noexcept;

private:
    std::string side_;
    JointSafetyLimits limits_;
    std::array<std::optional<std::uint64_t>, kArmJointCount> error_started_ns_{};
    std::array<bool, kArmJointCount> following_warning_emitted_{};
    std::optional<std::uint64_t> last_monotonic_ns_;
};

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
