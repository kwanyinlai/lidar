#include "scene/frontier_projection.h"

#include <stdio.h>
#include <stdlib.h>

#define OCCUPIED_CONFIDENCE_MARGIN 1.0f
#define FREE_CONFIDENCE_MARGIN 0.2f

ColumnSummary *create_column_summaries(const OccupancyMap *occupancy_grid_3d,
                                       int rover_height_cells) {
    int column_count = occupancy_grid_3d->width * occupancy_grid_3d->depth;
    ColumnSummary *column_summaries = malloc((int)column_count * sizeof(ColumnSummary));
    if (!column_summaries) {
        perror("malloc");
        return NULL;
    }

    for (int i = 0; i < column_count; i++) {
        column_summaries[i].is_blocking_count = rover_height_cells;
    }

    return column_summaries;
}

void apply_updates_to_projected_map(ColumnSummary *column_summaries,
                                    const VoxelUpdate *updates,
                                    int count,
                                    const OccupancyMap *occupancy_grid_3d,
                                    OccupancyMap *occupancy_grid_2d,
                                    int rover_height_cells) {

    update_column_summaries(column_summaries, (VoxelUpdate *)updates, count, occupancy_grid_3d);

    for (int i = 0; i < count; i++) {
        const VoxelUpdate *update = &updates[i];
        int x = update->idx % occupancy_grid_3d->width;
        int y = (update->idx / occupancy_grid_3d->width) % occupancy_grid_3d->height;
        int z = update->idx / (occupancy_grid_3d->width * occupancy_grid_3d->height);

        if (y >= rover_height_cells) {
            continue;
        }
        ColumnSummary *col = &(column_summaries[x * occupancy_grid_3d->depth + z]);
        if (col->is_blocking_count > 0) {
            occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = 
                OCCUPIED_THRESHOLD + OCCUPIED_CONFIDENCE_MARGIN;
        }
        else {
            occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = 
                FREE_THRESHOLD - FREE_CONFIDENCE_MARGIN;
        }
    }
}