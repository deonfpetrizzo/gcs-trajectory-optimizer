// trajectory_msg_builders.hpp
#ifndef GCS_PLANNER__TRAJECTORY_MSG_BUILDERS_HPP_
#define GCS_PLANNER__TRAJECTORY_MSG_BUILDERS_HPP_

#include "gcs_core/gcs_core.hpp"
#include "gcs_core/region_viz.hpp"
#include "gcs_planner/path_io.hpp"
#include "traversability_core/traversability_core.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <planner_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace gcs_planner {

/**
 * @brief Marker color keyed by segment tag (GROUND/AERIAL/TRANS/other).
 * @param tag Segment tag.
 * @return The color for that tag.
 */
std_msgs::msg::ColorRGBA tag_color(const std::string& tag);

/**
 * @brief Concatenates per-segment composite trajectories into one time-parameterized
 * planner_msgs/Trajectory.
 * @remark Timing accumulates across segments (each segment's rest-to-rest endpoint
 * sampled exactly). Attitude: yaw from travel heading; roll/pitch from differential
 * flatness (attitude_from_accel_yaw()) for non-GROUND segments, zero for GROUND
 * (which is not thrust-vectoring).
 * @param trajs Per-segment composite trajectories.
 * @param tags Each segment's tag, aligned with trajs (tags[si] tags trajs[si]).
 * @param hdr Header to stamp the message with.
 * @param sample_dt Sampling interval for the output points.
 * @return The concatenated trajectory message.
 */
planner_msgs::msg::Trajectory build_trajectory_msg(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr, double sample_dt);

/**
 * @brief Builds a nav_msgs/Path from raw waypoints (used for the nominal path).
 * @param pts Waypoints.
 * @param hdr Header to stamp the message with.
 * @return The path message.
 */
nav_msgs::msg::Path make_path_msg(const Path& pts, const std_msgs::msg::Header& hdr);

/**
 * @brief Builds a nav_msgs/Path sampling the concatenated composite trajectories' positions.
 * @param trajs Per-segment composite trajectories.
 * @param hdr Header to stamp the message with.
 * @param sample_dt Sampling interval.
 * @return The sampled path message.
 */
nav_msgs::msg::Path make_traj_path(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std_msgs::msg::Header& hdr, double sample_dt);

/**
 * @brief Builds one LINE_STRIP marker per segment trajectory, colored by its tag.
 * @param trajs Per-segment composite trajectories.
 * @param tags Each segment's tag, aligned with trajs.
 * @param hdr Header to stamp the markers with.
 * @param sample_dt Sampling interval for each strip.
 * @return The marker array.
 */
visualization_msgs::msg::MarkerArray make_traj_segment_markers(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr, double sample_dt);

/**
 * @brief Builds LINE_LIST wireframe markers for the corridor regions, colored by tag.
 * @param regions Grown corridor regions.
 * @param tags Each region's tag, aligned with regions.
 * @param hdr Header to stamp the markers with.
 * @return The marker array.
 */
visualization_msgs::msg::MarkerArray make_corridor_markers(
    const std::vector<gcs::ConvexRegion>& regions,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr);

/**
 * @brief Builds a CUBE_LIST marker of the traversability map, one cube per valid cell
 * at its fitted ground height, colored by per-cell traversability score.
 * @remark Cubes sit at (cell_center.x, cell_center.y, ground_z) with edge = grid
 * resolution; color ramps green (score 1) -> red (score 0) via
 * trav::traversability_score(cell, robot).
 * @param map Traversability map.
 * @param robot Robot model used to score each cell.
 * @param hdr Header to stamp the marker with.
 * @return The marker array (a single CUBE_LIST marker).
 */
visualization_msgs::msg::MarkerArray make_traversability_markers(
    const trav::TraversabilityMap& map, const trav::RobotModel& robot,
    const std_msgs::msg::Header& hdr);

/**
 * @brief Builds a nav_msgs/OccupancyGrid from a 2D occupancy grid.
 * @remark Occupied cells map to 100, free cells to 0; row-major layout matches
 * trav::GridInfo::index (iy*nx + ix).
 * @param grid Binary occupancy grid.
 * @param hdr Header to stamp the message with.
 * @return The occupancy grid message.
 */
nav_msgs::msg::OccupancyGrid make_occupancy_grid_msg(const trav::OccupancyGrid2D& grid,
                                                     const std_msgs::msg::Header& hdr);

}  // namespace gcs_planner
#endif  // GCS_PLANNER__TRAJECTORY_MSG_BUILDERS_HPP_
