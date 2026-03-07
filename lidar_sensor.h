#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include "scene.h"
#include "point_cloud.h"

void init_sensor_state();
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud);

#endif  // LIDAR_SENSOR_H