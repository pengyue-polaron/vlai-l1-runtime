#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace vlai_l1 {
// Continuous motor-reference conditioning, independent of policy chunk boundaries.
// Model samples retain their 30 Hz clock; this does not claim bounded physical speed.
class TaskCommandFilter {
public:
    static constexpr double speed_deg_s=20.0;
    static constexpr double time_constant_s=0.15;
    TaskCommandFilter(std::array<double,8> last_sent,std::uint64_t now,double rate_hz)
        :q_(last_sent),stamp_(now),period_(1.0/rate_hz) {
        if(!std::isfinite(rate_hz) || rate_hz<=0) throw std::runtime_error("invalid filter rate");
        for(double q:q_) if(!std::isfinite(q)) throw std::runtime_error("invalid filter initial reference");
    }
    const std::array<double,8>& advance(const std::array<double,8>& requested,std::uint64_t now) {
        if(now<stamp_) throw std::runtime_error("regressed filter time");
        for(double q:requested) if(!std::isfinite(q)) throw std::runtime_error("nonfinite filter target");
        // No catch-up jump after a delayed cycle; repeated calls at one time do not move.
        const double dt=std::min(period_,static_cast<double>(now-stamp_)/1e9);
        stamp_=now;
        const double alpha=-std::expm1(-dt/time_constant_s);
        for(std::size_t i=0;i<8;++i) {
            const double step=alpha*(requested[i]-q_[i]);
            q_[i]+=std::clamp(step,-speed_deg_s*dt,speed_deg_s*dt);
        }
        return q_;
    }
private:
    std::array<double,8> q_;
    std::uint64_t stamp_;
    double period_;
};
}
