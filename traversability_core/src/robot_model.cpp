// robot_model.cpp
#include "traversability_core/robot_model.hpp"

#include <algorithm>

namespace trav {

double traversability_score(const TraversabilityCell& cell, const RobotModel& robot) {
    if (!cell.valid) return 0.0;
    auto sat = [](double x) { return std::clamp(x, 0.0, 1.0); };

    const double s_slope =
        robot.max_slope_rad > 0.0 ? sat(1.0 - cell.slope_rad / robot.max_slope_rad) : 0.0;
    const double s_rough =
        robot.max_roughness > 0.0 ? sat(1.0 - cell.roughness / robot.max_roughness) : 0.0;
    const double s_step = robot.allow_stairs
                              ? 1.0
                              : (robot.max_step > 0.0
                                     ? sat(1.0 - cell.step_height / robot.max_step)
                                     : 0.0);
    return s_slope * s_rough * s_step;
}

bool passable(const TraversabilityCell& cell, const RobotModel& robot) {
    if (!cell.valid) return false;
    if (cell.slope_rad > robot.max_slope_rad) return false;
    if (cell.roughness > robot.max_roughness) return false;
    if (!robot.allow_stairs && cell.step_height > robot.max_step) return false;
    return true;
}

}  // namespace trav
