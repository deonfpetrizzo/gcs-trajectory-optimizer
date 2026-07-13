// trajectory_msg_builders.cpp
#include "gcs_planner/trajectory_msg_builders.hpp"

#include "gcs_planner/orientation.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/duration.hpp>

#include <cmath>

namespace gcs_planner {

std_msgs::msg::ColorRGBA tag_color(const std::string& tag) {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.85f;
    if (tag == "GROUND")      { c.r = 0.2f; c.g = 0.85f; c.b = 0.2f; }
    else if (tag == "AERIAL") { c.r = 0.2f; c.g = 0.6f;  c.b = 1.0f; }
    else if (tag == "TRANS")  { c.r = 1.0f; c.g = 0.8f;  c.b = 0.1f; }
    else                      { c.r = 0.6f; c.g = 0.6f;  c.b = 0.6f; }
    return c;
}

planner_msgs::msg::Trajectory build_trajectory_msg(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr, double sample_dt) {
    planner_msgs::msg::Trajectory out;
    out.header = hdr;
    double t_offset = 0.0;
    for (size_t si = 0; si < trajs.size(); ++si) {
        const auto& tr = trajs[si];
        const bool is_ground = (si < tags.size()) && (tags[si] == "GROUND");
        const double T = tr.total_duration();
        auto add_point = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            Eigen::VectorXd v = tr.velocity(tc);
            Eigen::VectorXd a = tr.acceleration(tc);

            planner_msgs::msg::TrajectoryPoint pt;
            pt.time_from_start = rclcpp::Duration::from_seconds(t_offset + tc);
            pt.position.x = p(0); pt.position.y = p(1); pt.position.z = p(2);
            pt.velocity.x = v(0); pt.velocity.y = v(1); pt.velocity.z = v(2);
            pt.acceleration.x = a(0); pt.acceleration.y = a(1); pt.acceleration.z = a(2);
            pt.yaw = std::atan2(v(1), v(0));
            if (is_ground) {
                pt.roll = 0.0;
                pt.pitch = 0.0;
            } else {
                double roll, pitch;
                attitude_from_accel_yaw(Eigen::Vector3d(a(0), a(1), a(2)), pt.yaw, roll, pitch);
                pt.roll  = roll;
                pt.pitch = pitch;
            }
            pt.speed = v.norm();
            out.points.push_back(pt);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt) add_point(t);
        add_point(T);
        t_offset += T;
    }
    out.total_duration = t_offset;
    return out;
}

nav_msgs::msg::Path make_path_msg(const Path& pts, const std_msgs::msg::Header& hdr) {
    nav_msgs::msg::Path msg;
    msg.header = hdr;
    for (const auto& p : pts) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = hdr;
        ps.pose.position.x = p(0);
        ps.pose.position.y = p(1);
        ps.pose.position.z = (p.size() > 2) ? p(2) : 0.0;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

nav_msgs::msg::Path make_traj_path(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std_msgs::msg::Header& hdr, double sample_dt) {
    nav_msgs::msg::Path msg;
    msg.header = hdr;
    for (const auto& tr : trajs) {
        const double T = tr.total_duration();
        auto add_pose = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            geometry_msgs::msg::PoseStamped ps;
            ps.header = hdr;
            ps.pose.position.x = p(0);
            ps.pose.position.y = p(1);
            ps.pose.position.z = p(2);
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt) add_pose(t);
        add_pose(T);
    }
    return msg;
}

visualization_msgs::msg::MarkerArray make_traj_segment_markers(
    const std::vector<gcs::CompositeTimingTrajectory>& trajs,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr, double sample_dt) {
    visualization_msgs::msg::MarkerArray arr;
    for (size_t i = 0; i < trajs.size(); ++i) {
        const auto& tr = trajs[i];
        visualization_msgs::msg::Marker m;
        m.header = hdr;
        m.ns = "trajectory_segments";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.08;
        m.color = tag_color(tags[i]);
        m.pose.orientation.w = 1.0;

        const double T = tr.total_duration();
        auto add_pt = [&](double tc) {
            Eigen::VectorXd p = tr.position(tc);
            geometry_msgs::msg::Point pt;
            pt.x = p(0); pt.y = p(1); pt.z = p(2);
            m.points.push_back(pt);
        };
        for (double t = 0.0; t < T - 1e-9; t += sample_dt) add_pt(t);
        add_pt(T);
        arr.markers.push_back(m);
    }
    return arr;
}

visualization_msgs::msg::MarkerArray make_corridor_markers(
    const std::vector<gcs::ConvexRegion>& regions,
    const std::vector<std::string>& tags,
    const std_msgs::msg::Header& hdr) {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (size_t ri = 0; ri < regions.size(); ++ri) {
        const auto& reg = regions[ri];
        Eigen::MatrixXd V = gcs::region_vertices(reg);
        if (V.rows() == 0) continue;
        auto edges = gcs::region_edges(reg, V);

        visualization_msgs::msg::Marker m;
        m.header = hdr;
        m.ns = "corridor";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        m.color = tag_color(tags[ri]);
        m.pose.orientation.w = 1.0;

        for (const auto& e : edges) {
            geometry_msgs::msg::Point a, b;
            a.x = V(e[0], 0); a.y = V(e[0], 1); a.z = (V.cols() > 2) ? V(e[0], 2) : 0.0;
            b.x = V(e[1], 0); b.y = V(e[1], 1); b.z = (V.cols() > 2) ? V(e[1], 2) : 0.0;
            m.points.push_back(a);
            m.points.push_back(b);
        }
        arr.markers.push_back(m);
    }
    return arr;
}

visualization_msgs::msg::MarkerArray make_traversability_markers(
    const trav::TraversabilityMap& map, const trav::RobotModel& robot,
    const std_msgs::msg::Header& hdr) {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker m;
    m.header = hdr;
    m.ns = "traversability";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = map.info.resolution;
    m.scale.y = map.info.resolution;
    m.scale.z = 0.02;
    m.pose.orientation.w = 1.0;

    for (int iy = 0; iy < map.info.ny; ++iy) {
        for (int ix = 0; ix < map.info.nx; ++ix) {
            const trav::TraversabilityCell& cell = map.at(ix, iy);
            if (!cell.valid) continue;

            const Eigen::Vector2d ctr = map.info.cell_center(ix, iy);
            geometry_msgs::msg::Point p;
            p.x = ctr.x();
            p.y = ctr.y();
            p.z = cell.ground_z;
            m.points.push_back(p);

            const double s = trav::traversability_score(cell, robot);
            std_msgs::msg::ColorRGBA c;
            c.r = static_cast<float>(1.0 - s);
            c.g = static_cast<float>(s);
            c.b = 0.0f;
            c.a = 0.85f;
            m.colors.push_back(c);
        }
    }
    arr.markers.push_back(m);
    return arr;
}

nav_msgs::msg::OccupancyGrid make_occupancy_grid_msg(const trav::OccupancyGrid2D& grid,
                                                     const std_msgs::msg::Header& hdr) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header = hdr;
    msg.info.resolution = static_cast<float>(grid.info.resolution);
    msg.info.width = static_cast<uint32_t>(grid.info.nx);
    msg.info.height = static_cast<uint32_t>(grid.info.ny);
    msg.info.origin.position.x = grid.info.origin.x();
    msg.info.origin.position.y = grid.info.origin.y();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid.occ.size());
    for (size_t i = 0; i < grid.occ.size(); ++i)
        msg.data[i] = grid.occ[i] ? static_cast<int8_t>(100) : static_cast<int8_t>(0);
    return msg;
}

}  // namespace gcs_planner
