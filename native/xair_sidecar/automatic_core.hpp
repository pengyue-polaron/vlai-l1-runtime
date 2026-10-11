#pragma once
// Pure bounded diagnostic trajectory and safety rules, no SDK or devices.
#include "motor_feedback.hpp"
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace vlai_l1 {
constexpr double kDegrees = 180.0 / 3.14159265358979323846;
struct AutoMotor {
    FeedbackSpec feedback;
    int motor_type = 0;
    unsigned send_id = 0;
    double kp = 0, kd = 0, min_deg = 0, max_deg = 0;
    double tracking_deg = 0, speed_deg_s = 0, torque_nm = 0, excursion_deg = 0;
};
struct AutoSpec {
    std::array<AutoMotor, 8> motors{};
    double amplitude_deg = 0, transition_s = 0, hold_s = 0, hold_test_s = 0;
    double maximum_amplitude_deg = 0, maximum_reference_speed = 0, maximum_reference_acceleration = 0;
    unsigned cycles = 0, rate_hz = 0, capacity = 0;
    unsigned enabled_status = 0, disabled_status = 0, temperature = 0;
    std::uint64_t state_timeout_ns = 0, tracking_timeout_ns = 0, shutdown_timeout_ns = 0;
    unsigned health_poll_ms = 0;
    int priority = 0;
    void validate() const;
    double duration(bool hold_only) const {
        return hold_only ? hold_test_s : hold_s + cycles * 4 * (transition_s + hold_s);
    }
    double offset(double seconds, bool hold_only) const;
};
inline void AutoSpec::validate() const {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    for (double v : {amplitude_deg, transition_s, hold_s, hold_test_s, maximum_amplitude_deg,
                     maximum_reference_speed, maximum_reference_acceleration}) {
        if (!positive(v)) throw std::invalid_argument("invalid automatic scalar");
    }
    if (!cycles || !rate_hz || capacity < 8 || capacity > 2'000'000 || !state_timeout_ns || !tracking_timeout_ns ||
        !shutdown_timeout_ns || !health_poll_ms || priority < 1 || priority > 99 ||
        enabled_status > 15 || disabled_status > 15 || enabled_status == disabled_status ||
        !temperature || temperature > 255 || amplitude_deg > maximum_amplitude_deg ||
        1.875 * amplitude_deg / transition_s > maximum_reference_speed ||
        (10 * std::sqrt(3.0) / 3) * amplitude_deg / (transition_s * transition_s) > maximum_reference_acceleration ||
        1e9 / rate_hz >= state_timeout_ns || health_poll_ms * 1'000'000ULL > state_timeout_ns ||
        !std::isfinite(duration(false)) || std::max(duration(false),duration(true)) * rate_hz * 8 + rate_hz * 8 * 10 > capacity) {
        throw std::invalid_argument("invalid automatic timing, capacity or trajectory budget");
    }
    const std::array<std::string,8> names{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","gripper"};
    for (std::size_t i=0;i<8;++i) {
        const auto& m=motors[i];
        if (m.feedback.name != names[i] || m.send_id < 1 || m.send_id > 15 ||
            m.feedback.slave_id != m.send_id || m.feedback.response_id > 0x7ff ||
            m.motor_type < 0 || m.motor_type > 9 || !positive(m.feedback.position_max_rad) ||
            !positive(m.feedback.velocity_max_rad_s) || !positive(m.feedback.torque_max_nm) ||
            !positive(m.kp) || m.kp > 500 || !positive(m.kd) || m.kd > 5 ||
            !std::isfinite(m.min_deg) || !std::isfinite(m.max_deg) || m.min_deg >= m.max_deg ||
            !positive(m.tracking_deg) || !positive(m.speed_deg_s) || !positive(m.torque_nm) ||
            !positive(m.excursion_deg) || m.torque_nm > m.feedback.torque_max_nm ||
            std::max(std::abs(m.min_deg),std::abs(m.max_deg)) > m.feedback.position_max_rad*kDegrees) {
            throw std::invalid_argument("invalid automatic motor " + names[i]);
        }
        for (std::size_t j=0;j<i;++j) if (m.send_id==motors[j].send_id || m.feedback.response_id==motors[j].feedback.response_id)
            throw std::invalid_argument("duplicate motor ID");
    }
    if (amplitude_deg > motors[6].excursion_deg) throw std::invalid_argument("trajectory exceeds excursion");
}
inline double AutoSpec::offset(double seconds, bool hold_only) const {
    if (!std::isfinite(seconds) || seconds < 0) throw std::invalid_argument("invalid trajectory time");
    if (hold_only || seconds <= hold_s || seconds >= duration(false)) return 0;
    const double t = seconds - hold_s;
    const unsigned segment = static_cast<unsigned>(t / (transition_s + hold_s));
    const double local = t - segment * (transition_s + hold_s);
    const std::array<double,4> starts{0,amplitude_deg,0,-amplitude_deg};
    const std::array<double,4> ends{amplitude_deg,0,-amplitude_deg,0};
    const unsigned leg=segment%4;
    const double u=std::min(1.0,local/transition_s);
    return starts[leg]+(ends[leg]-starts[leg])*u*u*u*(10-15*u+6*u*u);
}
class AutoGuard {
public:
    explicit AutoGuard(AutoSpec spec):spec_(std::move(spec)) { spec_.validate(); }
    void set_reference(const std::array<double,8>& q) {
        if (ready_ || failed_) throw std::runtime_error("reference already set/faulted");
        reference_=q;
        for (std::size_t i=0;i<8;++i) {
            const auto& m=spec_.motors[i];
            if (!std::isfinite(q[i]) || q[i] < m.min_deg || q[i] > m.max_deg) fail("initial bounds:"+m.feedback.name);
        }
        if (q[6]-spec_.amplitude_deg < spec_.motors[6].min_deg || q[6]+spec_.amplitude_deg > spec_.motors[6].max_deg)
            fail("wrist trajectory bounds");
        ready_=true;
    }
    void check(std::uint64_t now, const std::array<DecodedFeedback,8>& states,
               const std::array<double,8>& target, bool enabled) {
        if (failed_ || !ready_ || now<=last_now_) fail("invalid/non-monotonic guard state");
        last_now_=now;
        for (std::size_t i=0;i<8;++i) {
            const auto& s=states[i]; const auto& m=spec_.motors[i]; const auto stamp=s.raw.host_receive_ns;
            if (!stamp || stamp>now || now-stamp>spec_.state_timeout_ns || stamp<last_source_[i]) fail("stale/regressed motor:"+m.feedback.name);
            last_source_[i]=stamp;
            const auto q=s.position_rad*kDegrees;
            if (!std::isfinite(q) || !std::isfinite(target[i]) || !std::isfinite(s.velocity_rad_s) || !std::isfinite(s.torque_nm) ||
                q<m.min_deg || q>m.max_deg || target[i]<m.min_deg || target[i]>m.max_deg ||
                std::abs(q-reference_[i])>m.excursion_deg || std::abs(target[i]-reference_[i])>m.excursion_deg ||
                std::abs(s.velocity_rad_s*kDegrees)>m.speed_deg_s || std::abs(s.torque_nm)>m.torque_nm ||
                s.mos_temperature_raw>spec_.temperature || s.rotor_temperature_raw>spec_.temperature || s.at_protocol_endpoint)
                fail("position/speed/effort/temperature guard:"+m.feedback.name);
            if (enabled ? s.status_nibble!=spec_.enabled_status : (s.status_nibble!=spec_.disabled_status && s.status_nibble!=spec_.enabled_status))
                fail("motor status:"+m.feedback.name);
            if (std::abs(q-target[i])>m.tracking_deg) {
                if (!error_since_[i]) error_since_[i]=now;
                if (now-error_since_[i]>=spec_.tracking_timeout_ns) fail("tracking error:"+m.feedback.name);
            } else error_since_[i]=0;
        }
    }
private:
    [[noreturn]] void fail(const std::string& message) { failed_=true; throw std::runtime_error(message); }
    AutoSpec spec_; bool ready_=false,failed_=false;
    std::array<double,8> reference_{};
    std::array<std::uint64_t,8> last_source_{},error_since_{};
    std::uint64_t last_now_=0;
};
}
