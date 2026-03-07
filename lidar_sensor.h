#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include "scene.h"

void init_sensor_state();
void cast_all_rays(const TriangleArray *scene);
void rotate();

#endif  // LIDAR_SENSOR_H