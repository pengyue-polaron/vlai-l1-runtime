#include "feedback_log_buffer.hpp"

#include <algorithm>
#include <stdexcept>
#include <limits>

namespace vlai_l1 {
FeedbackLogBuffer::FeedbackLogBuffer(std::array<std::uint32_t, 8> ids,
                                     std::size_t capacity, std::uint64_t timeout_ns,
                                     bool retain_latest)
    : ids_(ids), capacity_(capacity), timeout_ns_(timeout_ns), retain_latest_(retain_latest) {
    auto sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    if (sorted.front() == 0 || sorted.back() > 0x7ff ||
        std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end() ||
        capacity < ids.size() || capacity > 2'000'000 || timeout_ns == 0) {
        throw std::invalid_argument("invalid passive feedback storage specification");
    }
    records_.reserve(capacity_);
}
const FeedbackLogRecord& FeedbackLogBuffer::chronological(std::size_t index) const {
    if (index >= records_.size()) throw std::out_of_range("feedback record index");
    return records_[(oldest_ + index) % records_.size()];
}
const FeedbackLogRecord& FeedbackLogBuffer::latest() const {
    return chronological(records_.size() - 1);
}
void FeedbackLogBuffer::require_healthy() const {
    if (!fault_.empty()) throw std::runtime_error(fault_);
}
[[noreturn]] void FeedbackLogBuffer::fail(const std::string& reason) {
    fault_ = reason;
    throw std::runtime_error(reason);
}
void FeedbackLogBuffer::start(std::uint64_t now_ns) {
    require_healthy();
    if (started_ns_ != 0 || now_ns == 0) fail("invalid feedback capture start");
    started_ns_ = last_check_ns_ = now_ns;
}
void FeedbackLogBuffer::ingest(const FeedbackLogRecord& r) {
    require_healthy();
    if (started_ns_ == 0) fail("feedback capture not started");
    const auto found = std::find(ids_.begin(), ids_.end(), r.can_id);
    if (found == ids_.end()) fail("unexpected or flagged CAN response id");
    if (r.overflow_count != 0) fail("passive feedback socket dropped frames");
    if (r.kernel_realtime_ns == 0 || r.kernel_realtime_ns > r.receive_realtime_ns ||
        r.receive_realtime_ns-r.kernel_realtime_ns > timeout_ns_) {
        fail("missing, future or stale kernel feedback timestamp");
    }
    if (r.receive_monotonic_ns < started_ns_ ||
        (!records_.empty() && r.receive_monotonic_ns <= latest().receive_monotonic_ns)) {
        fail("feedback receive monotonic timestamp regressed");
    }
    if (!records_.empty()) {
        const auto& prior = latest();
        if (r.kernel_realtime_ns < prior.kernel_realtime_ns ||
            r.receive_realtime_ns < prior.receive_realtime_ns) fail("feedback realtime clock regressed");
        const auto real_delta = r.receive_realtime_ns-prior.receive_realtime_ns;
        const auto mono_delta = r.receive_monotonic_ns-prior.receive_monotonic_ns;
        if ((real_delta > mono_delta ? real_delta-mono_delta : mono_delta-real_delta) > timeout_ns_) {
            fail("feedback clock discontinuity");
        }
    }
    if (records_.size() >= capacity_ && !retain_latest_) fail("passive feedback capture capacity exhausted");
    last_[static_cast<std::size_t>(found-ids_.begin())] = r.receive_monotonic_ns;
    if (records_.size() < capacity_) {
        records_.push_back(r);
    } else {
        if (evicted_ == std::numeric_limits<std::uint64_t>::max()) fail("feedback eviction counter exhausted");
        records_[oldest_] = r;
        oldest_ = (oldest_ + 1) % capacity_;
        ++evicted_;
    }
}
void FeedbackLogBuffer::check(std::uint64_t now_ns) {
    require_healthy();
    if (started_ns_ == 0 || now_ns < last_check_ns_ ||
        (!records_.empty() && now_ns < latest().receive_monotonic_ns)) {
        fail("invalid feedback watchdog timestamp");
    }
    last_check_ns_ = now_ns;
    for (std::size_t i = 0; i < last_.size(); ++i) {
        const auto last = last_[i] == 0 ? started_ns_ : last_[i];
        if (now_ns-last > timeout_ns_) fail("missing/stale motor response " + std::to_string(ids_[i]));
    }
}
}  // namespace vlai_l1
