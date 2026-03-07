#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H
#include "vec3.h"

typedef struct {
    Vector3 position;
    float distance;
} PointCloudEntry;

typedef struct {
    PointCloudEntry *data;
    size_t size;
    size_t capacity;
} PointCloud;

void point_cloud_push_back(PointCloud *cloud, Vector3 position, float distance);
void point_cloud_free(PointCloud *cloud);

#endif // POINT_CLOUD_H