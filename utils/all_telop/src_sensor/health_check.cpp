#include "health_check.h"

#include <algorithm>
#include <cmath>

namespace {

void quat_conj(const std::array<double, 4>& q, std::array<double, 4>& out) {
    out = {q[0], -q[1], -q[2], -q[3]};
}

void quat_mul(const std::array<double, 4>& a,
              const std::array<double, 4>& b,
              std::array<double, 4>& out) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

double quat_angle(const std::array<double, 4>& qa, const std::array<double, 4>& qb) {
    std::array<double, 4> qc{}, diff{};
    quat_conj(qa, qc);
    quat_mul(qb, qc, diff);
    const double w = std::min(std::max(std::abs(diff[0]), 0.0), 1.0);
    return 2.0 * std::acos(w);
}

double pos_dist(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

HealthStats analyze_stationary(const std::vector<PoseSample>& samples,
                               const HealthThresholds& th) {
    HealthStats st;
    st.n = samples.size();
    if (samples.size() < th.min_samples) {
        return st;
    }

    for (const auto& s : samples) {
        st.drift_m = std::max(st.drift_m, pos_dist(s.pos, samples.front().pos));
        st.drift_rad = std::max(st.drift_rad, quat_angle(samples.front().rot, s.rot));
    }

    double prev_speed = -1.0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - samples[i - 1].t;
        if (dt < th.min_dt_sec) {
            continue;
        }
        const double speed = pos_dist(samples[i].pos, samples[i - 1].pos) / dt;
        const double ang_speed = quat_angle(samples[i - 1].rot, samples[i].rot) / dt;
        st.max_speed = std::max(st.max_speed, speed);
        st.max_ang_speed = std::max(st.max_ang_speed, ang_speed);
        if (prev_speed >= 0.0) {
            st.max_acc = std::max(st.max_acc, std::abs(speed - prev_speed) / dt);
        }
        prev_speed = speed;
    }

    st.ok = (st.drift_m <= th.max_drift_m && st.drift_rad <= th.max_drift_rad &&
             st.max_speed <= th.max_speed && st.max_ang_speed <= th.max_ang_speed &&
             st.max_acc <= th.max_acc);
    return st;
}

JumpStats analyze_jump(const PoseSample& prev, const PoseSample& curr,
                       const JumpThresholds& th) {
    JumpStats st;
    st.dt = curr.t - prev.t;
    if (st.dt < th.min_dt_sec) {
        // Duplicate / out-of-order stamp: skip as non-failure.
        return st;
    }
    if (st.dt > th.max_gap_sec) {
        st.gap = true;
        st.ok = false;
        return st;
    }
    st.delta_m = pos_dist(curr.pos, prev.pos);
    st.speed = st.delta_m / st.dt;
    st.ang_speed = quat_angle(prev.rot, curr.rot) / st.dt;
    st.ok = (st.delta_m <= th.max_delta_m && st.speed <= th.max_speed &&
             st.ang_speed <= th.max_ang_speed);
    return st;
}
