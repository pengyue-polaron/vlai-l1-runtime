#include <xarm_teleop_sdk.h>

#include "runtime_safety.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The VLAI L1 x_air state protocol requires a little-endian host"
#endif

namespace {

constexpr std::size_t kMotorCount = 8;
constexpr std::chrono::milliseconds kHealthPoll{20};
constexpr std::size_t kControlMessageLimit = 256;
constexpr char kAdjustPositionRequest[] = "ADJUST_POSITION\n";

std::atomic<bool> g_stop_requested{false};

enum class CallbackFault : int {
    kNone = 0,
    kWrongDof = 1,
    kNonFinite = 2,
};

struct Options {
    std::string side;
    std::string leader_can;
    std::string follower_can;
    std::filesystem::path leader_urdf;
    std::filesystem::path follower_urdf;
    std::filesystem::path config_dir;
    std::filesystem::path state_socket;
    std::filesystem::path control_socket;
    int publish_hz = 0;
    int state_timeout_ms = 0;
    int rt_priority = 0;
    int can_health_poll_ms = 0;
    int control_owner_uid = 0;
    int control_owner_gid = 0;
};

struct StateSnapshot {
    std::uint64_t source_sequence = 0;
    std::uint64_t monotonic_ns = 0;
    std::array<double, kMotorCount> leader{};
    std::array<double, kMotorCount> follower{};
};

class StateSlot {
public:
    void update(const float* leader_arm, int arm_dof, float leader_gripper,
                const float* follower_arm, float follower_gripper) noexcept {
        if (leader_arm == nullptr || follower_arm == nullptr || arm_dof != 7) {
            fault_.store(CallbackFault::kWrongDof, std::memory_order_release);
            return;
        }
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            return;
        }
        StateSnapshot candidate;
        for (int index = 0; index < arm_dof; ++index) {
            candidate.leader[static_cast<std::size_t>(index)] = leader_arm[index];
            candidate.follower[static_cast<std::size_t>(index)] = follower_arm[index];
        }
        candidate.leader.back() = leader_gripper;
        candidate.follower.back() = follower_gripper;
        for (double value : candidate.leader) {
            if (!std::isfinite(value)) {
                fault_.store(CallbackFault::kNonFinite, std::memory_order_release);
                return;
            }
        }
        for (double value : candidate.follower) {
            if (!std::isfinite(value)) {
                fault_.store(CallbackFault::kNonFinite, std::memory_order_release);
                return;
            }
        }
        candidate.source_sequence = next_sequence_++;
        candidate.monotonic_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        latest_ = candidate;
        available_ = true;
        lock.unlock();
        updated_.notify_all();
    }

    bool copy(StateSnapshot& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!available_ || latest_.source_sequence == last_published_sequence_) {
            return false;
        }
        output = latest_;
        last_published_sequence_ = latest_.source_sequence;
        return true;
    }

    CallbackFault fault() const noexcept { return fault_.load(std::memory_order_acquire); }

    std::optional<std::uint64_t> last_update_ns() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!available_) {
            return std::nullopt;
        }
        return latest_.monotonic_ns;
    }

    std::optional<std::uint64_t> latest_sequence() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!available_) {
            return std::nullopt;
        }
        return latest_.source_sequence;
    }

    bool wait_for_update_after(std::optional<std::uint64_t> sequence,
                               std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return updated_.wait_for(lock, timeout, [this, sequence]() {
            return available_ &&
                   (!sequence.has_value() || latest_.source_sequence > *sequence);
        });
    }

private:
    std::mutex mutex_;
    std::condition_variable updated_;
    StateSnapshot latest_;
    std::uint64_t next_sequence_ = 0;
    std::uint64_t last_published_sequence_ = std::numeric_limits<std::uint64_t>::max();
    bool available_ = false;
    std::atomic<CallbackFault> fault_{CallbackFault::kNone};
};

#pragma pack(push, 1)
struct WirePacket {
    std::array<char, 4> magic{'V', 'L', '1', 'S'};
    std::uint16_t version = 1;
    std::uint8_t side = 0;
    std::uint8_t reserved = 0;
    std::uint64_t source_sequence = 0;
    std::uint64_t monotonic_ns = 0;
    std::array<double, kMotorCount> leader{};
    std::array<double, kMotorCount> follower{};
};
#pragma pack(pop)

static_assert(sizeof(WirePacket) == 152, "wire protocol layout changed");

class DatagramPublisher {
public:
    DatagramPublisher(std::filesystem::path destination, std::string side, int publish_hz)
        : destination_(std::move(destination)),
          side_code_(side == "left" ? 0 : 1),
          period_(std::chrono::nanoseconds(1'000'000'000LL / publish_hz)) {
        if (destination_.native().size() >= sizeof(sockaddr_un::sun_path)) {
            throw std::invalid_argument("state socket path is too long");
        }
        socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) {
            throw std::runtime_error(std::string("cannot create state socket: ") +
                                     std::strerror(errno));
        }
        address_.sun_family = AF_UNIX;
        std::strncpy(address_.sun_path, destination_.c_str(), sizeof(address_.sun_path) - 1);
    }

    DatagramPublisher(const DatagramPublisher&) = delete;
    DatagramPublisher& operator=(const DatagramPublisher&) = delete;

    ~DatagramPublisher() {
        if (socket_ >= 0) {
            ::close(socket_);
        }
    }

    void run(StateSlot& slot) {
        auto next_tick = std::chrono::steady_clock::now();
        while (!g_stop_requested.load(std::memory_order_relaxed)) {
            StateSnapshot snapshot;
            if (slot.copy(snapshot)) {
                WirePacket packet;
                packet.side = side_code_;
                packet.source_sequence = snapshot.source_sequence;
                packet.monotonic_ns = snapshot.monotonic_ns;
                packet.leader = snapshot.leader;
                packet.follower = snapshot.follower;
                const auto sent = ::sendto(
                    socket_, &packet, sizeof(packet), MSG_DONTWAIT,
                    reinterpret_cast<const sockaddr*>(&address_), sizeof(address_));
                if (sent < 0 && errno != ENOENT && errno != ECONNREFUSED && errno != EAGAIN &&
                    errno != ENOBUFS) {
                    std::cerr << "WARN state publish failed: " << std::strerror(errno) << '\n';
                }
            }
            next_tick += period_;
            std::this_thread::sleep_until(next_tick);
        }
    }

private:
    std::filesystem::path destination_;
    std::uint8_t side_code_;
    std::chrono::nanoseconds period_;
    int socket_ = -1;
    sockaddr_un address_{};
};

class TeleopHandle {
public:
    ~TeleopHandle() {
        if (handle_ != nullptr) {
            xarm_teleop_destroy(handle_);
        }
    }

    xarm_teleop_handle_t* output() { return &handle_; }
    xarm_teleop_handle_t get() const { return handle_; }

private:
    xarm_teleop_handle_t handle_ = nullptr;
};

class ControlServer {
public:
    ControlServer(std::filesystem::path path, int owner_uid, int owner_gid)
        : path_(std::move(path)) {
        if (!path_.is_absolute() || path_.native().size() >= sizeof(sockaddr_un::sun_path)) {
            throw std::invalid_argument("control socket path is invalid");
        }
        if (!std::filesystem::is_directory(path_.parent_path())) {
            throw std::invalid_argument("control socket directory is missing");
        }
        struct stat existing {};
        if (::lstat(path_.c_str(), &existing) == 0) {
            throw std::runtime_error("control socket path is already in use");
        } else if (errno != ENOENT) {
            throw std::runtime_error(std::string("cannot inspect control socket: ") +
                                     std::strerror(errno));
        }
        socket_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) {
            throw std::runtime_error(std::string("cannot create control socket: ") +
                                     std::strerror(errno));
        }
        sockaddr_un address {};
        address.sun_family = AF_UNIX;
        std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
        if (::bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            const std::string detail = std::strerror(errno);
            cleanup();
            throw std::runtime_error("cannot bind control socket: " + detail);
        }
        bound_ = true;
        struct stat identity {};
        if (::lstat(path_.c_str(), &identity) != 0 || !S_ISSOCK(identity.st_mode)) {
            cleanup();
            throw std::runtime_error("bound control endpoint is not a socket");
        }
        inode_ = identity.st_ino;
        if (::chown(path_.c_str(), static_cast<uid_t>(owner_uid),
                    static_cast<gid_t>(owner_gid)) != 0 ||
            ::chmod(path_.c_str(), S_IRUSR | S_IWUSR) != 0) {
            const std::string detail = std::strerror(errno);
            cleanup();
            throw std::runtime_error("cannot protect control socket: " + detail);
        }
        if (::listen(socket_, 2) != 0) {
            const std::string detail = std::strerror(errno);
            cleanup();
            throw std::runtime_error("cannot listen on control socket: " + detail);
        }
    }

    ControlServer(const ControlServer&) = delete;
    ControlServer& operator=(const ControlServer&) = delete;

    ~ControlServer() { cleanup(); }

    int accept_one() const {
        const int client = ::accept4(socket_, nullptr, nullptr, SOCK_CLOEXEC);
        if (client < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return -1;
        }
        if (client < 0) {
            throw std::runtime_error(std::string("control socket accept failed: ") +
                                     std::strerror(errno));
        }
        return client;
    }

    static std::string receive_request(int client) {
        timeval timeout {};
        timeout.tv_sec = 1;
        if (::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
            throw std::runtime_error("cannot set control request timeout");
        }
        std::string request;
        std::array<char, kControlMessageLimit> buffer {};
        while (request.find('\n') == std::string::npos) {
            const auto received = ::recv(client, buffer.data(), buffer.size(), 0);
            if (received == 0) {
                break;
            }
            if (received < 0) {
                throw std::runtime_error(std::string("cannot read control request: ") +
                                         std::strerror(errno));
            }
            request.append(buffer.data(), static_cast<std::size_t>(received));
            if (request.size() > kControlMessageLimit) {
                throw std::runtime_error("control request is too large");
            }
        }
        return request;
    }

    static void reply(int client, const std::string& response) noexcept {
        std::size_t offset = 0;
        while (offset < response.size()) {
            const auto sent =
                ::send(client, response.data() + offset, response.size() - offset, MSG_NOSIGNAL);
            if (sent <= 0) {
                return;
            }
            offset += static_cast<std::size_t>(sent);
        }
    }

private:
    void cleanup() noexcept {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
        if (!bound_) {
            return;
        }
        struct stat identity {};
        if (::lstat(path_.c_str(), &identity) == 0 && S_ISSOCK(identity.st_mode) &&
            identity.st_ino == inode_) {
            ::unlink(path_.c_str());
        }
        bound_ = false;
    }

    std::filesystem::path path_;
    int socket_ = -1;
    ino_t inode_ = 0;
    bool bound_ = false;
};

void on_signal(int) { g_stop_requested.store(true, std::memory_order_relaxed); }

void state_callback(const float* leader_arm, int arm_dof, float leader_gripper,
                    const float* follower_arm, float follower_gripper, void* user_data) {
    auto* slot = static_cast<StateSlot*>(user_data);
    if (slot != nullptr) {
        slot->update(leader_arm, arm_dof, leader_gripper, follower_arm, follower_gripper);
    }
}

std::string usage(const char* program) {
    return std::string("Usage: ") + program +
           " --side <left|right> --leader-can <if> --follower-can <if>"
           " --leader-urdf <path> --follower-urdf <path> --config-dir <path>"
           " --state-socket <path> --publish-hz <hz> --state-timeout-ms <ms>"
           " --rt-priority <1..99> --can-health-poll-ms <ms>"
           " --control-socket <path> --control-owner-uid <uid>"
           " --control-owner-gid <gid>";
}

Options parse_options(int argc, char** argv) {
    if (argc != 29) {
        throw std::invalid_argument(usage(argv[0]));
    }
    std::unordered_map<std::string, std::string> values;
    for (int index = 1; index < argc; index += 2) {
        const std::string key = argv[index];
        if (key.rfind("--", 0) != 0 || !values.emplace(key, argv[index + 1]).second) {
            throw std::invalid_argument(usage(argv[0]));
        }
    }
    const std::array<const char*, 14> required{
        "--side",         "--leader-can",   "--follower-can", "--leader-urdf",
        "--follower-urdf", "--config-dir",   "--state-socket", "--publish-hz",
        "--state-timeout-ms", "--rt-priority", "--can-health-poll-ms",
        "--control-socket", "--control-owner-uid", "--control-owner-gid",
    };
    for (const char* key : required) {
        if (values.find(key) == values.end() || values.at(key).empty()) {
            throw std::invalid_argument(usage(argv[0]));
        }
    }
    Options options;
    options.side = values.at("--side");
    options.leader_can = values.at("--leader-can");
    options.follower_can = values.at("--follower-can");
    options.leader_urdf = values.at("--leader-urdf");
    options.follower_urdf = values.at("--follower-urdf");
    options.config_dir = values.at("--config-dir");
    options.state_socket = values.at("--state-socket");
    options.control_socket = values.at("--control-socket");
    const auto integer = [&values](const char* key) {
        try {
            std::size_t consumed = 0;
            const auto& text = values.at(key);
            const int parsed = std::stoi(text, &consumed);
            if (consumed != text.size()) {
                throw std::invalid_argument("trailing characters");
            }
            return parsed;
        } catch (const std::exception&) {
            throw std::invalid_argument(std::string(key + 2) + " must be an integer");
        }
    };
    options.publish_hz = integer("--publish-hz");
    options.state_timeout_ms = integer("--state-timeout-ms");
    options.rt_priority = integer("--rt-priority");
    options.can_health_poll_ms = integer("--can-health-poll-ms");
    options.control_owner_uid = integer("--control-owner-uid");
    options.control_owner_gid = integer("--control-owner-gid");
    if ((options.side != "left" && options.side != "right") || options.publish_hz <= 0 ||
        options.publish_hz > 500 || options.state_timeout_ms <= 0 ||
        options.rt_priority <= 0 || options.rt_priority >= 100 ||
        options.can_health_poll_ms <= 0 ||
        options.can_health_poll_ms > options.state_timeout_ms ||
        options.control_owner_uid < 0 || options.control_owner_gid < 0 ||
        options.leader_can == options.follower_can) {
        throw std::invalid_argument(usage(argv[0]));
    }
    for (const auto& path : {options.leader_urdf, options.follower_urdf}) {
        if (!std::filesystem::is_regular_file(path)) {
            throw std::invalid_argument("URDF is not a regular file: " + path.string());
        }
    }
    if (!std::filesystem::is_directory(options.config_dir)) {
        throw std::invalid_argument("config-dir is not a directory");
    }
    for (const char* filename : {"leader.yaml", "follower.yaml"}) {
        const auto profile = options.config_dir / filename;
        if (!std::filesystem::is_regular_file(profile)) {
            throw std::invalid_argument("control profile is not a regular file: " +
                                        profile.string());
        }
    }
    return options;
}

int run(const Options& options) {
    vlai_l1::configure_fifo_priority_cap(options.rt_priority);
    vlai_l1::set_and_require_all_threads_fifo(options.rt_priority, 1);
    DatagramPublisher publisher(options.state_socket, options.side, options.publish_hz);
    ControlServer control(options.control_socket, options.control_owner_uid,
                          options.control_owner_gid);
    vlai_l1::CanHealthMonitor can_health({options.leader_can, options.follower_can});
    StateSlot slot;
    TeleopHandle handle;
    const std::string arm_side = options.side + "_arm";
    std::cout << "INFO creating x_air " << arm_side << " session: " << options.leader_can
              << " -> " << options.follower_can << '\n';
    const int create_result = xarm_teleop_create_unilateral(
        options.leader_can.c_str(), options.follower_can.c_str(),
        options.leader_urdf.c_str(), options.follower_urdf.c_str(), arm_side.c_str(),
        options.config_dir.c_str(), handle.output());
    if (create_result != XARM_TELEOP_OK) {
        std::cerr << "FAIL x_air create: " << xarm_teleop_get_last_error() << '\n';
        return 1;
    }
    can_health.check();
    if (xarm_teleop_set_full_state_callback(handle.get(), state_callback, &slot) !=
        XARM_TELEOP_OK) {
        std::cerr << "FAIL x_air callback: " << xarm_teleop_get_last_error() << '\n';
        return 1;
    }
    if (xarm_teleop_start(handle.get()) != XARM_TELEOP_OK) {
        std::cerr << "FAIL x_air start: " << xarm_teleop_get_last_error() << '\n';
        return 1;
    }
    vlai_l1::set_and_require_all_threads_fifo(options.rt_priority, 4);
    can_health.check();

    std::thread publisher_thread([&publisher, &slot]() { publisher.run(slot); });
    std::cout << "PASS x_air " << arm_side << " control running (SDK "
              << xarm_teleop_version() << ")\n";
    int result = 0;
    const auto started_at = std::chrono::steady_clock::now();
    const auto state_timeout = std::chrono::milliseconds(options.state_timeout_ms);
    const auto can_health_period = std::chrono::milliseconds(options.can_health_poll_ms);
    auto next_runtime_health_check = started_at;
    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        const auto fault = slot.fault();
        if (fault != CallbackFault::kNone) {
            std::cerr << "FAIL x_air state callback fault: " << static_cast<int>(fault) << '\n';
            result = 1;
            break;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto last_update_ns = slot.last_update_ns();
        const auto last_update = last_update_ns.has_value()
                                     ? std::chrono::steady_clock::time_point(
                                           std::chrono::nanoseconds(*last_update_ns))
                                     : started_at;
        if (now - last_update > state_timeout) {
            std::cerr << "FAIL x_air state callback is stale\n";
            result = 1;
            break;
        }
        if (xarm_teleop_is_running(handle.get()) != 1) {
            std::cerr << "FAIL x_air control loop stopped unexpectedly\n";
            result = 1;
            break;
        }
        const int client = control.accept_one();
        if (client >= 0) {
            bool fatal_control_error = false;
            try {
                const std::string request = ControlServer::receive_request(client);
                if (request != kAdjustPositionRequest) {
                    ControlServer::reply(client, "ERROR invalid control request\n");
                } else {
                    std::cout << "STEP running x_air " << arm_side
                              << " AdjustPosition\n";
                    const auto prior_sequence = slot.latest_sequence();
                    const int adjust_result = xarm_teleop_go_home(handle.get());
                    if (adjust_result != XARM_TELEOP_OK) {
                        const std::string detail = xarm_teleop_get_last_error();
                        ControlServer::reply(client, "ERROR " + detail + "\n");
                        std::cerr << "FAIL x_air AdjustPosition: " << detail << '\n';
                        fatal_control_error = true;
                    } else if (!slot.wait_for_update_after(prior_sequence, state_timeout)) {
                        ControlServer::reply(
                            client, "ERROR state callback did not resume after AdjustPosition\n");
                        std::cerr
                            << "FAIL x_air state callback did not resume after AdjustPosition\n";
                        fatal_control_error = true;
                    } else {
                        vlai_l1::require_all_threads_fifo(options.rt_priority, 5);
                        can_health.check();
                        ControlServer::reply(client, "OK\n");
                        std::cout << "PASS x_air " << arm_side
                                  << " AdjustPosition complete\n";
                        next_runtime_health_check = std::chrono::steady_clock::now();
                    }
                }
            } catch (const std::exception& error) {
                ControlServer::reply(client, std::string("ERROR ") + error.what() + "\n");
                std::cerr << "FAIL x_air control request: " << error.what() << '\n';
                fatal_control_error = true;
            }
            ::close(client);
            if (fatal_control_error) {
                result = 1;
                break;
            }
        }
        if (now >= next_runtime_health_check) {
            try {
                vlai_l1::require_all_threads_fifo(options.rt_priority, 5);
                can_health.check();
            } catch (const std::exception& error) {
                std::cerr << "FAIL x_air runtime health: " << error.what() << '\n';
                result = 1;
                break;
            }
            next_runtime_health_check = now + can_health_period;
        }
        std::this_thread::sleep_for(kHealthPoll);
    }
    g_stop_requested.store(true, std::memory_order_relaxed);
    publisher_thread.join();
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        return run(parse_options(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 2;
    }
}
