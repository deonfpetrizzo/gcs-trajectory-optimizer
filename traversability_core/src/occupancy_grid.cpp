// occupancy_grid.cpp
#include "traversability_core/occupancy_grid.hpp"

namespace trav {

OccupancyGrid2D make_occupancy_grid(const TraversabilityMap& map, const RobotModel& robot,
                                    const OccupancyParams& params) {
    OccupancyGrid2D grid;
    grid.info = map.info;
    grid.occ.assign(map.cells.size(), 1);

    for (int iy = 0; iy < map.info.ny; ++iy) {
        for (int ix = 0; ix < map.info.nx; ++ix) {
            const TraversabilityCell& cell = map.at(ix, iy);
            const bool free = cell.valid && passable(cell, robot) &&
                              cell.min_overhead_clearance >= robot.height &&
                              (params.score_threshold <= 0.0 ||
                               traversability_score(cell, robot) >= params.score_threshold);
            grid.occ[map.info.index(ix, iy)] = free ? 0 : 1;
        }
    }
    return grid;
}

}  // namespace trav
