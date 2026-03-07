#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H
#include "vec3.h"

typedef struct {
    Vector3 position;
    float distance;
    float intensity;
} PointCloudEntry;

typedef struct {
    PointCloudEntry *data;
    size_t size;
    size_t capacity;
} PointCloud;

void point_cloud_push_back(PointCloud *cloud, Vector3 position, float distance, float intensity);
void point_cloud_free(PointCloud *cloud);
void init_point_cloud(PointCloud *cloud);

#endif // POINT_CLOUD_H