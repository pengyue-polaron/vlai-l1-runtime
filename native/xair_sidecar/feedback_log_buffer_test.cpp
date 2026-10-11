#include "feedback_log_buffer.hpp"

#include <functional>
#include <iostream>
#include <stdexcept>

namespace {
const std::array<std::uint32_t, 8> ids{17,18,19,20,21,22,23,24};
void require(bool ok) { if (!ok) throw std::runtime_error("feedback log test failed"); }
vlai_l1::FeedbackLogRecord record(std::uint64_t now, std::uint32_t id = 17) {
    return {now, 10'000+now, 9'999+now, id, true, {}, 0};
}
void rejects(const std::function<void(vlai_l1::FeedbackLogRecord&)>& mutate) {
    vlai_l1::FeedbackLogBuffer buffer(ids, 16, 100);
    buffer.start(1000);
    buffer.ingest(record(1001));
    auto bad = record(1002);
    mutate(bad);
    bool caught = false;
    try { buffer.ingest(bad); } catch (const std::runtime_error&) { caught = true; }
    require(caught && !buffer.fault().empty() && buffer.records().size() == 1);
    caught = false;
    try { buffer.ingest(record(1003)); } catch (const std::runtime_error&) { caught = true; }
    require(caught); // Never resume a failed session.
}
}
int main() {
    try {
        vlai_l1::FeedbackLogBuffer buffer(ids, 16, 100);
        buffer.start(1000);
        for (std::size_t i = 0; i < 8; ++i) buffer.ingest(record(1001+i, ids[i]));
        buffer.check(1101); // Exact timeout boundary is still fresh.
        bool caught = false;
        try { buffer.check(1102); } catch (const std::runtime_error&) { caught = true; }
        require(caught && buffer.fault().find("17") != std::string::npos);
        rejects([](auto& r) { r.can_id |= 0x80000000; });
        rejects([](auto& r) { r.can_id = 25; });
        rejects([](auto& r) { r.overflow_count = 1; });
        rejects([](auto& r) { r.kernel_realtime_ns = 0; });
        rejects([](auto& r) { r.kernel_realtime_ns = r.receive_realtime_ns+1; });
        rejects([](auto& r) { r.kernel_realtime_ns = r.receive_realtime_ns-101; });
        rejects([](auto& r) { r.receive_monotonic_ns = 1001; });
        rejects([](auto& r) { r.receive_realtime_ns += 1000; r.kernel_realtime_ns += 1000; });
        rejects([](auto& r) { r.receive_realtime_ns -= 1; r.kernel_realtime_ns -= 2; });
        vlai_l1::FeedbackLogBuffer missing(ids, 8, 100);
        missing.start(1000);
        caught = false;
        try { missing.check(1101); } catch (const std::runtime_error&) { caught = true; }
        require(caught);
        vlai_l1::FeedbackLogBuffer full(ids, 8, 100);
        full.start(1000);
        for (std::size_t i = 0; i < 8; ++i) full.ingest(record(1001+i, ids[i]));
        caught = false;
        try { full.ingest(record(1009)); } catch (const std::runtime_error&) { caught = true; }
        require(caught && full.records().size() == 8);
        vlai_l1::FeedbackLogBuffer ring(ids, 16, 100, true);
        ring.start(1000);
        for (std::size_t i = 0; i < 57; ++i) {
            ring.ingest(record(1001+i, ids[i % 8]));
            ring.check(1001+i);
        }
        require(ring.records().size() == 16 && ring.evicted() == 41);
        for (std::size_t i = 0; i < 16; ++i) {
            require(ring.chronological(i).receive_monotonic_ns == 1042+i);
            require(ring.chronological(i).can_id == ids[(41+i) % 8]);
        }
        caught = false;
        try { ring.ingest(record(1057)); } catch (const std::runtime_error&) { caught = true; }
        require(caught); // Regression still fails after wrap, against newest, not storage.back().
        vlai_l1::FeedbackLogBuffer ring_overflow(ids, 8, 100, true);
        ring_overflow.start(1000);
        for (std::size_t i = 0; i < 24; ++i) ring_overflow.ingest(record(1001+i, ids[i % 8]));
        auto lost = record(1025); lost.overflow_count = 1;
        caught = false;
        try { ring_overflow.ingest(lost); } catch (const std::runtime_error&) { caught = true; }
        require(caught && ring_overflow.evicted() == 16);
        vlai_l1::FeedbackLogBuffer ring_stale(ids, 8, 100, true);
        ring_stale.start(1000);
        for (std::size_t i = 0; i < 24; ++i) ring_stale.ingest(record(1001+i, ids[i % 8]));
        caught = false;
        try { ring_stale.check(1118); } catch (const std::runtime_error&) { caught = true; }
        require(caught); // A ring never bypasses the per-motor watchdog.
        auto duplicate = ids; duplicate[7] = 17;
        caught = false;
        try { vlai_l1::FeedbackLogBuffer invalid(duplicate, 8, 100); }
        catch (const std::invalid_argument&) { caught = true; }
        require(caught);
        std::cout << "PASS pure passive feedback buffer\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
