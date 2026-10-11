#pragma once

#include "feedback_log_buffer.hpp"

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <thread>
#include <mutex>

namespace vlai_l1 {
// Optional diagnostic in the SDK-owning process. Never sends a CAN frame.
// Buffered raw data is written only after the SDK handle has been released.
class PassiveFeedbackLog {
public:
    PassiveFeedbackLog(const std::filesystem::path& path,
                       std::array<std::uint32_t, 8> ids, std::size_t capacity,
                       std::uint64_t timeout_ns, int poll_ms, int owner_uid, int owner_gid,
                       bool retain_latest = false);
    ~PassiveFeedbackLog();
    void start(const std::string& interface);
    bool faulted() const { return failed_.load(std::memory_order_acquire); }
    void stop_capture() noexcept { stop(); }
    void finish(bool runtime_succeeded);
    std::array<FeedbackLogRecord, 8> latest_records();
private:
    void receive_loop() noexcept;
    void stop() noexcept;
    FeedbackLogBuffer buffer_;
    std::array<std::uint32_t, 8> ids_;
    int poll_ms_;
    int socket_ = -1;
    FILE* output_ = nullptr;
    std::thread thread_;
    std::atomic<bool> stopped_{false};
    std::atomic<bool> failed_{false};
    std::string error_;
    std::string interface_;
    bool retain_latest_;
    std::mutex latest_mutex_;
    std::array<FeedbackLogRecord, 8> latest_{};
};
}  // namespace vlai_l1
