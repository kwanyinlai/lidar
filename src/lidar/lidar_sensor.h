
/**
 * @file lidar_sensor.h
 * @brief Lidar sensor simulation interface.
 *
 * Provides functions for initializing, updating, and moving a simulated lidar sensor,
 * as well as retrieving its position. Integrates with scene geometry and point cloud output.
 */

# ifndef LIDAR_SENSOR_H
# define LIDAR_SENSOR_H

# define NUM_RINGS 256
# define MATH_PI 3.14159f
# define MATH_DEG_TO_RAD (MATH_PI / 180.0f)
# define MAX_RANGE 30.f



#include "rendering/scene.h"
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param map Pointer to the occupancy map.
 */
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *map);


void init_sensor_rays();

#endif // LIDAR_SENSOR_H

