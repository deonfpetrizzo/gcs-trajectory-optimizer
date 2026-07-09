// path_io.cpp
#include "gcs_planner/path_io.hpp"

#include "gcs_planner/orientation.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace gcs_planner {

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

}  // namespace

std::string vehicle_key_from_csv_path(const std::string& file) {
    std::string base = file;
    const auto slash = base.find_last_of('/');
    if (slash != std::string::npos) base = base.substr(slash + 1);
    const auto dot = base.find_last_of('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    const std::string prefix = "path_";
    if (base.rfind(prefix, 0) == 0) base = base.substr(prefix.size());
    return base;
}

std::vector<PathSegment> load_path_segments_csv(const std::vector<std::string>& files,
                                                const rclcpp::Logger& logger) {
    std::vector<PathSegment> segments;
    bool have_current = false;
    long current_seg_idx = 0;

    for (const auto& file : files) {
        std::ifstream in(file);
        if (!in.is_open()) {
            throw std::runtime_error("failed to open nominal path CSV: " + file);
        }

        std::string header_line;
        if (!std::getline(in, header_line)) {
            throw std::runtime_error("empty nominal path CSV: " + file);
        }
        auto header = split_csv_line(header_line);
        int xi = -1, yi = -1, zi = -1, segi = -1, tagi = -1;
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i] == "x") xi = static_cast<int>(i);
            else if (header[i] == "y") yi = static_cast<int>(i);
            else if (header[i] == "z") zi = static_cast<int>(i);
            else if (header[i] == "seg_idx") segi = static_cast<int>(i);
            else if (header[i] == "tag") tagi = static_cast<int>(i);
        }
        if (xi < 0 || yi < 0 || zi < 0) {
            throw std::runtime_error("nominal path CSV missing x/y/z column: " + file);
        }

        std::string line;
        size_t row_count = 0;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            auto fields = split_csv_line(line);
            size_t need = static_cast<size_t>(std::max({xi, yi, zi, segi, tagi}));
            if (fields.size() <= need) {
                throw std::runtime_error("malformed row in nominal path CSV: " + file);
            }

            long seg_idx = (segi >= 0) ? std::stol(fields[segi]) : 0;
            std::string tag = (tagi >= 0) ? fields[tagi] : "";
            Eigen::VectorXd p(3);
            p << std::stod(fields[xi]), std::stod(fields[yi]), std::stod(fields[zi]);

            if (!have_current || seg_idx != current_seg_idx) {
                segments.push_back({tag, {}});
                current_seg_idx = seg_idx;
                have_current = true;
            }
            segments.back().waypoints.push_back(p);
            ++row_count;
        }
        RCLCPP_INFO(logger, "loaded %zu waypoints from %s", row_count, file.c_str());
    }
    return segments;
}

void write_trajectory_csv(const planner_msgs::msg::Trajectory& traj,
                          const std::vector<std::string>& seg_tags,
                          const std::string& file_path,
                          const rclcpp::Logger& logger) {
    std::ofstream out(file_path);
    if (!out.is_open()) {
        RCLCPP_ERROR(logger, "failed to open trajectory CSV for writing: %s", file_path.c_str());
        return;
    }

    out << std::setprecision(9);
    out << "seg_idx,tag,x,y,z,qx,qy,qz,qw\n";

    size_t seg_idx = 0;
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        const std::string& tag = (seg_idx < seg_tags.size()) ? seg_tags[seg_idx] : "";

        double qx, qy, qz, qw;
        trajectory_point_quaternion(pt, qx, qy, qz, qw);

        out << seg_idx << ',' << tag << ','
            << pt.position.x << ',' << pt.position.y << ',' << pt.position.z << ','
            << qx << ',' << qy << ',' << qz << ',' << qw << '\n';

        if (i + 1 < traj.points.size()) {
            const double t_here = rclcpp::Duration(pt.time_from_start).seconds();
            const double t_next = rclcpp::Duration(traj.points[i + 1].time_from_start).seconds();
            if (t_next <= t_here + 1e-9) ++seg_idx;
        }
    }

    RCLCPP_INFO(logger, "wrote %zu trajectory points (%zu segments) to %s",
                traj.points.size(), seg_tags.size(), file_path.c_str());
}

}  // namespace gcs_planner
