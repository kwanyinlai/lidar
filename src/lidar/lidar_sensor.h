
/**
 * @file lidar_sensor.h
 * @brief Lidar sensor simulation interface.
 *
 * Provides functions for initializing, updating, and moving a simulated lidar sensor,
 * as well as retrieving its position. Integrates with scene geometry and point cloud output.
 */

# ifndef LIDAR_SENSOR_H
# define LIDAR_SENSOR_H


# define MATH_PI 3.14159f
# define MATH_DEG_TO_RAD (MATH_PI / 180.0f)
# define MAX_RANGE 30.f

#include "rendering/scene.h"
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"
#include "lidar/sensor_control.h"


extern int g_scan_cmd_fd; // file descriptor for sending scan commands to coordinator process, set in main.c after forking processes
extern int g_ray_batch_results_fd; // file descriptor for receiving ray batch results from worker processes, set in main.c after forking processes

float elevations[NUM_RINGS];

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param map Pointer to the occupancy map.
 */
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *map);

static inline float gaussian_noise() {
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f); // avoid log(0)
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159f * u2);
    // TODO: hard coded PI, if needed more, then define a constant for it.
}


void init_sensor_rays();

#endif // LIDAR_SENSOR_H

