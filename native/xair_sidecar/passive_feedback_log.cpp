#include "passive_feedback_log.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace vlai_l1 {
namespace {
std::uint64_t clock_ns(clockid_t clock) {
    timespec value{};
    if (clock_gettime(clock, &value) != 0) throw std::runtime_error("feedback clock read failed");
    return static_cast<std::uint64_t>(value.tv_sec)*1'000'000'000 + value.tv_nsec;
}
void require(int status, const char* operation) {
    if (status < 0) throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}
PassiveFeedbackLog::PassiveFeedbackLog(const std::filesystem::path& path,
    std::array<std::uint32_t, 8> ids, std::size_t capacity, std::uint64_t timeout_ns, int poll_ms,
    int owner_uid, int owner_gid, bool retain_latest)
    : buffer_(ids, capacity, timeout_ns, retain_latest), ids_(ids), poll_ms_(poll_ms),
      retain_latest_(retain_latest) {
    if (poll_ms <= 0 || static_cast<std::uint64_t>(poll_ms)*1'000'000 > timeout_ns) {
        throw std::invalid_argument("invalid passive feedback polling period");
    }
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
    require(fd, "create passive feedback log");
    if (owner_uid < 0 || owner_gid < 0 || ::fchown(fd, owner_uid, owner_gid) != 0) {
        ::close(fd); throw std::runtime_error("feedback output ownership failed");
    }
    output_ = ::fdopen(fd, "w");
    if (output_ == nullptr) { ::close(fd); throw std::runtime_error("open feedback output stream failed"); }
}
PassiveFeedbackLog::~PassiveFeedbackLog() {
    stop();
    if (output_ != nullptr) ::fclose(output_);
}
void PassiveFeedbackLog::stop() noexcept {
    stopped_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
    if (socket_ >= 0) { ::close(socket_); socket_ = -1; }
}
void PassiveFeedbackLog::start(const std::string& interface) {
    if (socket_ >= 0 || thread_.joinable() || stopped_.load()) {
        throw std::runtime_error("passive feedback capture cannot be restarted");
    }
    interface_ = interface;
    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    require(socket_, "open passive feedback socket");
    const int enabled = 1;
    require(::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)), "CAN FD reception");
    require(::setsockopt(socket_, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)), "kernel receive timestamp");
    require(::setsockopt(socket_, SOL_SOCKET, SO_RXQ_OVFL, &enabled, sizeof(enabled)), "socket overflow reporting");
    std::array<can_filter, 8> filters{};
    for (std::size_t i = 0; i < ids_.size(); ++i) {
        filters[i] = {ids_[i], CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
    }
    require(::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)), "feedback filters");
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(::if_nametoindex(interface.c_str()));
    if (address.can_ifindex == 0) throw std::runtime_error("unknown feedback interface");
    require(::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)), "bind passive feedback socket");
    buffer_.start(clock_ns(CLOCK_MONOTONIC));
    thread_ = std::thread([this]() { receive_loop(); });
}
void PassiveFeedbackLog::receive_loop() noexcept {
    try {
        while (!stopped_.load(std::memory_order_relaxed)) {
            pollfd ready{socket_, POLLIN, 0};
            const int polled = ::poll(&ready, 1, poll_ms_);
            if (polled < 0 && errno == EINTR) continue;
            require(polled, "passive feedback poll");
            if (ready.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("feedback socket poll fault");
            // Bounded work per wake; no filesystem or SDK calls on this thread.
            for (std::size_t i = 0; i < 256 && !stopped_.load(); ++i) {
                canfd_frame frame{};
                iovec data{&frame, sizeof(frame)};
                alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))+CMSG_SPACE(sizeof(std::uint32_t))> control{};
                msghdr message{};
                message.msg_iov = &data; message.msg_iovlen = 1;
                message.msg_control = control.data(); message.msg_controllen = control.size();
                const auto count = ::recvmsg(socket_, &message, MSG_DONTWAIT);
                if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                if (count < 0 && errno == EINTR) continue;
                require(static_cast<int>(count), "passive feedback receive");
                if ((count != CAN_MTU && count != CANFD_MTU) || frame.len != 8 ||
                    (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC))) {
                    throw std::runtime_error("truncated/malformed passive feedback frame");
                }
                FeedbackLogRecord record{};
                record.receive_monotonic_ns = clock_ns(CLOCK_MONOTONIC);
                record.receive_realtime_ns = clock_ns(CLOCK_REALTIME);
                record.can_id = frame.can_id;
                record.can_fd = count == CANFD_MTU;
                std::memcpy(record.payload.data(), frame.data, 8);
                for (auto* c = CMSG_FIRSTHDR(&message); c != nullptr; c = CMSG_NXTHDR(&message, c)) {
                    if (c->cmsg_level != SOL_SOCKET) continue;
                    if (c->cmsg_type == SO_TIMESTAMPNS && c->cmsg_len == CMSG_LEN(sizeof(timespec))) {
                        timespec stamp{}; std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
                        if (stamp.tv_sec <= 0 || stamp.tv_nsec < 0 || stamp.tv_nsec >= 1'000'000'000) {
                            throw std::runtime_error("invalid kernel feedback timestamp");
                        }
                        record.kernel_realtime_ns = static_cast<std::uint64_t>(stamp.tv_sec)*1'000'000'000+stamp.tv_nsec;
                    } else if (c->cmsg_type == SO_RXQ_OVFL && c->cmsg_len == CMSG_LEN(sizeof(std::uint32_t))) {
                        std::memcpy(&record.overflow_count, CMSG_DATA(c), sizeof(record.overflow_count));
                    }
                }
                buffer_.ingest(record);
                {
                    std::lock_guard<std::mutex> guard(latest_mutex_);
                    for (std::size_t j = 0; j < ids_.size(); ++j) {
                        if (record.can_id == ids_[j]) latest_[j] = record;
                    }
                }
            }
            buffer_.check(clock_ns(CLOCK_MONOTONIC));
        }
    } catch (const std::exception& error) {
        error_ = error.what();
        failed_.store(true, std::memory_order_release);
    }
}
std::array<FeedbackLogRecord, 8> PassiveFeedbackLog::latest_records() {
    std::lock_guard<std::mutex> guard(latest_mutex_);
    return latest_;
}
void PassiveFeedbackLog::finish(bool runtime_succeeded) {
    stop();
    if (output_ == nullptr) throw std::runtime_error("feedback log already closed");
    const bool passed = runtime_succeeded && !faulted() && !buffer_.records().empty();
    std::fprintf(output_, "# interface=%s\n# status=%s\n# diagnostic_only=true\n", interface_.c_str(), passed ? "passed" : "failed");
    std::fprintf(output_, "# error=%s\n", error_.c_str());
    if (retain_latest_) {
        std::fprintf(output_, "# retained_latest_evicted=%llu\n",
            static_cast<unsigned long long>(buffer_.evicted()));
    }
    std::fprintf(output_, "sequence,host_receive_monotonic_ns,host_receive_realtime_ns,kernel_receive_realtime_ns,can_id,can_fd,payload_hex,rx_overflow_count\n");
    std::size_t sequence = 0;
    for (std::size_t index = 0; index < buffer_.records().size(); ++index) {
        const auto& r = buffer_.chronological(index);
        std::fprintf(output_, "%zu,%llu,%llu,%llu,%u,%u,", sequence++,
            static_cast<unsigned long long>(r.receive_monotonic_ns),
            static_cast<unsigned long long>(r.receive_realtime_ns),
            static_cast<unsigned long long>(r.kernel_realtime_ns), r.can_id, r.can_fd ? 1 : 0);
        for (const auto byte : r.payload) std::fprintf(output_, "%02x", byte);
        std::fprintf(output_, ",%u\n", r.overflow_count);
    }
    const bool write_error = std::ferror(output_) != 0;
    const int close_status = std::fclose(output_); output_ = nullptr;
    if (write_error || close_status != 0) throw std::runtime_error("passive feedback log write failed");
    if (!passed) throw std::runtime_error("passive feedback capture failed: " + error_);
}
}  // namespace vlai_l1
