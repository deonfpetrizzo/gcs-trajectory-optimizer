// path_io.hpp
#ifndef GCS_PLANNER__PATH_IO_HPP_
#define GCS_PLANNER__PATH_IO_HPP_

#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <planner_msgs/msg/trajectory.hpp>

#include <string>
#include <vector>

namespace gcs_planner {

using Path = std::vector<Eigen::VectorXd>;

/**
 * @brief A contiguous run of waypoints sharing one seg_idx/tag from the nominal path
 * CSV (e.g. "GROUND", "TRANS", "AERIAL"). Corridor generation and trajectory
 * optimization run independently per segment.
 */
struct PathSegment {
    std::string tag;
    Path waypoints;
};

/**
 * @brief Derives a short vehicle key from a nominal_path_csv_files entry, used to
 * namespace that vehicle's topics (trajectory/g1m4, corridor/g1m4, ...).
 * @param file A CSV path, e.g. "/home/x/paths/path_g1m4.csv".
 * @return The derived key, e.g. "g1m4".
 */
std::string vehicle_key_from_csv_path(const std::string& file);

/**
 * @brief Loads x,y,z waypoints from one or more CSV files, grouped into segments by
 * contiguous seg_idx.
 * @remark Header must include "x","y","z"; "seg_idx" and "tag" are optional (absent
 * means one untagged segment). Segments are appended across files in list order.
 * @param files CSV file paths, in load order.
 * @param logger Logger for per-file load diagnostics.
 * @return The parsed path segments.
 */
std::vector<PathSegment> load_path_segments_csv(const std::vector<std::string>& files,
                                                const rclcpp::Logger& logger);

/**
 * @brief Writes an exported trajectory to `file_path`.
 * @remark Columns are seg_idx,tag,x,y,z,qx,qy,qz,qw (mirroring the input path CSV
 * format, plus orientation). Segment boundaries within the flat, concatenated
 * Trajectory message are detected the same way trajectory_executor_node does: two
 * consecutive points sharing the same time_from_start mark a boundary. Orientation
 * comes from each point's roll/pitch/yaw fields (see trajectory_point_quaternion()).
 * @param traj The trajectory to export.
 * @param seg_tags Each detected segment's tag, in order.
 * @param file_path Output CSV path.
 * @param logger Logger for diagnostics.
 */
void write_trajectory_csv(const planner_msgs::msg::Trajectory& traj,
                          const std::vector<std::string>& seg_tags,
                          const std::string& file_path,
                          const rclcpp::Logger& logger);

}  // namespace gcs_planner
#endif  // GCS_PLANNER__PATH_IO_HPP_
