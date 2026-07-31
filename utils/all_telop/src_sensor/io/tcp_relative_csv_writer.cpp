#include <io/tcp_relative_csv_writer.hpp>

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace {

std::string expand_home(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }
    const char* home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
        return path;
    }
    if (path.size() == 1) {
        return home;
    }
    if (path[1] == '/') {
        return std::string(home) + path.substr(1);
    }
    // "~otheruser/..." is not supported; return as-is.
    return path;
}

}  // namespace

TcpRelativeCsvWriter::~TcpRelativeCsvWriter() { close(); }

bool TcpRelativeCsvWriter::open(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.close();
    }

    const std::string resolved = expand_home(path);
    try {
        const std::filesystem::path p(resolved);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        std::cerr << "[tcp-csv] WARN: failed to create parent dirs for " << resolved << ": "
                  << e.what() << std::endl;
    }

    file_.open(resolved, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "[tcp-csv] ERROR: failed to open " << resolved << std::endl;
        return false;
    }

    file_ << "Frame_Index,Timestamp(s),Delay(s),x,y,z,qw,qx,qy,qz\n";
    file_.flush();

    origin_set_ = false;
    prev_timestamp_valid_ = false;
    frame_index_ = 0;

    std::cout << "[tcp-csv] recording relative TCP pose -> " << resolved << std::endl;
    return true;
}

void TcpRelativeCsvWriter::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.close();
    }
}

bool TcpRelativeCsvWriter::is_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_.is_open();
}

void TcpRelativeCsvWriter::record(const Eigen::Vector3d& p, const Eigen::Matrix3d& R,
                                  double timestamp_sec) {
    if (!enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) {
        return;
    }

    if (!origin_set_) {
        p0_ = p;
        r0_ = R;
        origin_set_ = true;
    }

    const Eigen::Vector3d p_rel = p - p0_;
    const Eigen::Matrix3d r_rel = r0_.transpose() * R;
    const Eigen::Quaterniond q_rel(r_rel);

    const double delay = prev_timestamp_valid_ ? (timestamp_sec - prev_timestamp_) : 0.0;
    prev_timestamp_ = timestamp_sec;
    prev_timestamp_valid_ = true;

    file_ << (frame_index_++) << ',' << std::fixed << std::setprecision(6) << timestamp_sec << ','
          << delay << ',' << p_rel.x() << ',' << p_rel.y() << ',' << p_rel.z() << ',' << q_rel.w()
          << ',' << q_rel.x() << ',' << q_rel.y() << ',' << q_rel.z() << '\n';
    file_.flush();
}
