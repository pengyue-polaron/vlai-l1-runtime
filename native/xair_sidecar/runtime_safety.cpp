#include "runtime_safety.hpp"

#include <dlfcn.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace {

using PthreadSetSchedParam = int (*)(pthread_t, int, const sched_param*);

std::atomic<int> g_fifo_priority_cap{0};
std::atomic<PthreadSetSchedParam> g_real_pthread_setschedparam{nullptr};

PthreadSetSchedParam resolve_pthread_setschedparam() noexcept {
    auto real = g_real_pthread_setschedparam.load(std::memory_order_acquire);
    if (real != nullptr) {
        return real;
    }
    dlerror();
    const auto symbol = dlsym(RTLD_NEXT, "pthread_setschedparam");
    if (symbol == nullptr || dlerror() != nullptr) {
        return nullptr;
    }
    real = reinterpret_cast<PthreadSetSchedParam>(symbol);
    g_real_pthread_setschedparam.store(real, std::memory_order_release);
    return real;
}

template <typename Value>
Value attribute_value(const rtattr* attribute, const char* label) {
    if (RTA_PAYLOAD(attribute) != sizeof(Value)) {
        throw std::runtime_error(std::string("invalid rtnetlink ") + label + " size");
    }
    Value value;
    std::memcpy(&value, RTA_DATA(attribute), sizeof(value));
    return value;
}

std::vector<pid_t> thread_ids() {
    std::vector<pid_t> result;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
        const auto name = entry.path().filename().string();
        std::size_t consumed = 0;
        try {
            const long value = std::stol(name, &consumed);
            if (consumed == name.size() && value > 0) {
                result.push_back(static_cast<pid_t>(value));
            }
        } catch (const std::exception&) {
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::string can_health_detail(const vlai_l1::CanHealthSnapshot& snapshot) {
    std::ostringstream output;
    output << "state=" << static_cast<int>(snapshot.state)
           << " txerr=" << snapshot.live_errors.txerr
           << " rxerr=" << snapshot.live_errors.rxerr
           << " bus_error=" << snapshot.cumulative.bus_error
           << " warning=" << snapshot.cumulative.error_warning
           << " passive=" << snapshot.cumulative.error_passive
           << " bus_off=" << snapshot.cumulative.bus_off
           << " arbitration_lost=" << snapshot.cumulative.arbitration_lost
           << " restarts=" << snapshot.cumulative.restarts;
    return output.str();
}

}  // namespace

extern "C" int pthread_setschedparam(pthread_t thread, int policy,
                                     const struct sched_param* parameter) {
    const auto real = resolve_pthread_setschedparam();
    if (real == nullptr) {
        return ENOSYS;
    }
    sched_param bounded = *parameter;
    if (policy == SCHED_FIFO) {
        bounded.sched_priority = vlai_l1::bounded_fifo_priority(
            bounded.sched_priority, g_fifo_priority_cap.load(std::memory_order_relaxed));
    }
    return real(thread, policy, &bounded);
}

namespace vlai_l1 {

int bounded_fifo_priority(int requested, int configured_cap) noexcept {
    if (configured_cap <= 0) {
        return requested;
    }
    return std::min(requested, configured_cap);
}

void configure_fifo_priority_cap(int priority) {
    if (priority < sched_get_priority_min(SCHED_FIFO) ||
        priority > sched_get_priority_max(SCHED_FIFO)) {
        throw std::invalid_argument("configured FIFO priority is outside the host range");
    }
    if (resolve_pthread_setschedparam() == nullptr) {
        throw std::runtime_error("cannot resolve pthread_setschedparam");
    }
    g_fifo_priority_cap.store(priority, std::memory_order_release);
}

void require_all_threads_fifo(int priority, std::size_t minimum_thread_count) {
    const auto threads = thread_ids();
    if (threads.size() < minimum_thread_count) {
        throw std::runtime_error("x_air process has fewer threads than required");
    }
    for (const pid_t thread : threads) {
        sched_param parameter{};
        const int policy = sched_getscheduler(thread);
        if (policy < 0 || sched_getparam(thread, &parameter) != 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "cannot inspect x_air thread scheduling");
        }
        if (policy != SCHED_FIFO || parameter.sched_priority != priority) {
            throw std::runtime_error("x_air thread scheduling differs from tracked FIFO policy");
        }
    }
}

void set_and_require_all_threads_fifo(int priority, std::size_t minimum_thread_count) {
    const auto threads = thread_ids();
    if (threads.size() < minimum_thread_count) {
        throw std::runtime_error("x_air process has fewer threads than required");
    }
    const sched_param parameter{priority};
    for (const pid_t thread : threads) {
        if (sched_setscheduler(thread, SCHED_FIFO, &parameter) != 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "cannot apply tracked x_air FIFO policy");
        }
    }
    require_all_threads_fifo(priority, minimum_thread_count);
}

void require_can_health(const std::string& interface, const CanHealthSnapshot& baseline,
                        const CanHealthSnapshot& current) {
    if (baseline.state != CAN_STATE_ERROR_ACTIVE || baseline.live_errors.txerr != 0 ||
        baseline.live_errors.rxerr != 0) {
        throw std::runtime_error(interface + " CAN baseline is unhealthy: " +
                                 can_health_detail(baseline));
    }
    if (current.state != CAN_STATE_ERROR_ACTIVE || current.live_errors.txerr != 0 ||
        current.live_errors.rxerr != 0) {
        throw std::runtime_error(interface + " CAN live health failed: " +
                                 can_health_detail(current));
    }
    const auto& before = baseline.cumulative;
    const auto& after = current.cumulative;
    if (before.bus_error != after.bus_error ||
        before.error_warning != after.error_warning ||
        before.error_passive != after.error_passive || before.bus_off != after.bus_off ||
        before.arbitration_lost != after.arbitration_lost || before.restarts != after.restarts) {
        throw std::runtime_error(interface + " CAN cumulative health changed: baseline " +
                                 can_health_detail(baseline) + ", current " +
                                 can_health_detail(current));
    }
}

CanHealthMonitor::CanHealthMonitor(std::vector<std::string> interfaces)
    : interfaces_(std::move(interfaces)) {
    if (interfaces_.empty()) {
        throw std::invalid_argument("CAN health monitor requires at least one interface");
    }
    socket_ = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (socket_ < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "cannot create CAN rtnetlink socket");
    }
    sockaddr_nl local{};
    local.nl_family = AF_NETLINK;
    if (::bind(socket_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        const int error = errno;
        ::close(socket_);
        socket_ = -1;
        throw std::system_error(error, std::generic_category(),
                                "cannot bind CAN rtnetlink socket");
    }
    const timeval receive_timeout{0, 20'000};
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
                   sizeof(receive_timeout)) != 0) {
        const int error = errno;
        ::close(socket_);
        socket_ = -1;
        throw std::system_error(error, std::generic_category(),
                                "cannot configure CAN rtnetlink timeout");
    }
    try {
        for (const auto& interface : interfaces_) {
            const auto snapshot = read(interface);
            require_can_health(interface, snapshot, snapshot);
            baselines_.emplace(interface, snapshot);
        }
    } catch (...) {
        ::close(socket_);
        socket_ = -1;
        throw;
    }
}

CanHealthMonitor::~CanHealthMonitor() {
    if (socket_ >= 0) {
        ::close(socket_);
    }
}

void CanHealthMonitor::check() {
    for (const auto& interface : interfaces_) {
        require_can_health(interface, baselines_.at(interface), read(interface));
    }
}

CanHealthSnapshot CanHealthMonitor::read(const std::string& interface) {
    const unsigned int index = if_nametoindex(interface.c_str());
    if (index == 0) {
        throw std::system_error(errno, std::generic_category(),
                                "cannot resolve CAN interface " + interface);
    }
    struct Request {
        nlmsghdr header;
        ifinfomsg link;
    } request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST;
    request.header.nlmsg_seq = ++sequence_;
    request.link.ifi_family = AF_UNSPEC;
    request.link.ifi_index = static_cast<int>(index);
    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;
    if (sendto(socket_, &request, request.header.nlmsg_len, 0,
               reinterpret_cast<const sockaddr*>(&kernel), sizeof(kernel)) < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "cannot request CAN rtnetlink health");
    }

    std::array<std::byte, 16 * 1024> buffer{};
    const auto received = recv(socket_, buffer.data(), buffer.size(), 0);
    if (received < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "cannot receive CAN rtnetlink health");
    }
    CanHealthSnapshot snapshot;
    bool found_state = false;
    bool found_live = false;
    bool found_cumulative = false;
    int remaining = static_cast<int>(received);
    for (auto* message = reinterpret_cast<nlmsghdr*>(buffer.data()); NLMSG_OK(message, remaining);
         message = NLMSG_NEXT(message, remaining)) {
        if (message->nlmsg_seq != sequence_) {
            continue;
        }
        if (message->nlmsg_type == NLMSG_ERROR) {
            const auto* netlink_error = static_cast<const nlmsgerr*>(NLMSG_DATA(message));
            if (netlink_error->error != 0) {
                throw std::system_error(-netlink_error->error, std::generic_category(),
                                        "CAN rtnetlink rejected health request");
            }
            continue;
        }
        if (message->nlmsg_type != RTM_NEWLINK) {
            continue;
        }
        const auto* link = static_cast<const ifinfomsg*>(NLMSG_DATA(message));
        if (link->ifi_index != static_cast<int>(index)) {
            continue;
        }
        int attribute_length = IFLA_PAYLOAD(message);
        for (auto* attribute = IFLA_RTA(link); RTA_OK(attribute, attribute_length);
             attribute = RTA_NEXT(attribute, attribute_length)) {
            if ((attribute->rta_type & NLA_TYPE_MASK) != IFLA_LINKINFO) {
                continue;
            }
            int info_length = RTA_PAYLOAD(attribute);
            for (auto* info = static_cast<rtattr*>(RTA_DATA(attribute));
                 RTA_OK(info, info_length); info = RTA_NEXT(info, info_length)) {
                const auto info_type = info->rta_type & NLA_TYPE_MASK;
                if (info_type == IFLA_INFO_XSTATS) {
                    snapshot.cumulative =
                        attribute_value<can_device_stats>(info, "CAN statistics");
                    found_cumulative = true;
                    continue;
                }
                if (info_type != IFLA_INFO_DATA) {
                    continue;
                }
                int can_length = RTA_PAYLOAD(info);
                for (auto* can_attribute = static_cast<rtattr*>(RTA_DATA(info));
                     RTA_OK(can_attribute, can_length);
                     can_attribute = RTA_NEXT(can_attribute, can_length)) {
                    const auto can_type = can_attribute->rta_type & NLA_TYPE_MASK;
                    if (can_type == IFLA_CAN_STATE) {
                        snapshot.state =
                            attribute_value<can_state>(can_attribute, "CAN state");
                        found_state = true;
                    } else if (can_type == IFLA_CAN_BERR_COUNTER) {
                        snapshot.live_errors = attribute_value<can_berr_counter>(
                            can_attribute, "CAN error counters");
                        found_live = true;
                    }
                }
            }
        }
    }
    if (!found_state || !found_live || !found_cumulative) {
        throw std::runtime_error("CAN rtnetlink response is incomplete for " + interface);
    }
    return snapshot;
}

}  // namespace vlai_l1
