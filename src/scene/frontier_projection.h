#ifndef FRONTIER_PROJECTION_H
#define FRONTIER_PROJECTION_H

#include "piping/messages.h"
#include "scene/occupancy_map.h"

ColumnSummary *create_column_summaries(const OccupancyMap *occupancy_grid_3d,
                                       int rover_height_cells);

void apply_updates_to_projected_map(ColumnSummary *column_summaries,
                                    const VoxelUpdate *updates,
                                    int count,
                                    const OccupancyMap *occupancy_grid_3d,
                                    OccupancyMap *occupancy_grid_2d,
                                    int rover_height_cells);

#endif // FRONTIER_PROJECTION_H