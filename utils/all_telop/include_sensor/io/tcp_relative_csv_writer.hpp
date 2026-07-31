#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

// Records the desired TCP pose (position + orientation) as a trace relative to
// a fixed origin T0 = (p0, R0). T0 is locked on the FIRST call to record() and
// is NEVER reset afterwards (in particular, stopping/resuming teleop does not
// re-zero it) -- callers that want a fresh T0 must construct a new writer.
//
// CSV columns: Frame_Index,Timestamp(s),Delay(s),x,y,z,qw,qx,qy,qz
// where (x,y,z,qw,qx,qy,qz) is the pose expressed relative to T0:
//   p_rel = p - p0
//   R_rel = R0^T * R  (written as a wxyz quaternion)
class TcpRelativeCsvWriter {
public:
    TcpRelativeCsvWriter() = default;
    ~TcpRelativeCsvWriter();

    TcpRelativeCsvWriter(const TcpRelativeCsvWriter&) = delete;
    TcpRelativeCsvWriter& operator=(const TcpRelativeCsvWriter&) = delete;

    // Opens `path` (expanding a leading '~' and creating parent directories as
    // needed) and writes the CSV header. Returns false on failure.
    bool open(const std::string& path);

    void close();
    bool is_open() const;

    // Gate: record() is a no-op whenever !enabled(). Does not affect T0 lock.
    void set_enabled(bool enabled) { enabled_ = enabled; }
    bool enabled() const { return enabled_; }

    // Records one (p, R) sample at wall-clock `timestamp_sec`. No-op when
    // !enabled() or the file could not be opened.
    void record(const Eigen::Vector3d& p, const Eigen::Matrix3d& R, double timestamp_sec);

private:
    mutable std::mutex mutex_;
    std::ofstream file_;
    bool enabled_ = true;

    bool origin_set_ = false;
    Eigen::Vector3d p0_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r0_ = Eigen::Matrix3d::Identity();

    bool prev_timestamp_valid_ = false;
    double prev_timestamp_ = 0.0;
    uint64_t frame_index_ = 0;
};
