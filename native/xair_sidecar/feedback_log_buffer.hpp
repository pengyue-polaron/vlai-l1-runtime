#pragma once

// Pure bounded diagnostic storage. No device, clock, file or SDK access.
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace vlai_l1 {
struct FeedbackLogRecord {
    std::uint64_t receive_monotonic_ns;
    std::uint64_t receive_realtime_ns;
    std::uint64_t kernel_realtime_ns;
    std::uint32_t can_id;
    bool can_fd;
    std::array<std::uint8_t, 8> payload;
    std::uint32_t overflow_count;
};

class FeedbackLogBuffer {
public:
    FeedbackLogBuffer(std::array<std::uint32_t, 8> ids, std::size_t capacity,
                      std::uint64_t timeout_ns, bool retain_latest = false);
    void start(std::uint64_t now_ns);
    void ingest(const FeedbackLogRecord& record);
    void check(std::uint64_t now_ns);
    const std::vector<FeedbackLogRecord>& records() const { return records_; }
    const FeedbackLogRecord& chronological(std::size_t index) const;
    std::uint64_t evicted() const { return evicted_; }
    const std::string& fault() const { return fault_; }
private:
    [[noreturn]] void fail(const std::string& reason);
    void require_healthy() const;
    std::array<std::uint32_t, 8> ids_;
    std::array<std::uint64_t, 8> last_{};
    std::vector<FeedbackLogRecord> records_;
    std::size_t capacity_;
    std::uint64_t timeout_ns_;
    std::uint64_t started_ns_ = 0;
    std::uint64_t last_check_ns_ = 0;
    std::string fault_;
    bool retain_latest_;
    std::size_t oldest_ = 0;
    std::uint64_t evicted_ = 0;
    const FeedbackLogRecord& latest() const;
};
}  // namespace vlai_l1
