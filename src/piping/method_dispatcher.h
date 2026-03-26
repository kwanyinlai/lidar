
# ifndef METHOD_DISPATCHER_H
# define METHOD_DISPATCHER_H

# include "rendering/scene.h"
# include "lidar/occupancy_map.h"

# define NUM_WORKERS 4

extern int g_scan_cmd_fd;

void run_occupancy_updater_loop(int , OccupancyMap*);

void run_coordinator_loop(int, int, int[NUM_WORKERS][2], int[NUM_WORKERS][2], int);

void run_worker_loop(int, int, TriangleArray*);

#endif // METHOD_DISPATCHER_H