#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include "scene/point_cloud.h"

typedef struct {
    float dx;
    float dz;
    float dtheta;
    float error;     // mean closest-point distance, useful for rejecting bad matches
    int converged;
} ICPResult;

ICPResult run_icp(const PointCloud *source,
                  const PointCloud *target,
                  int max_iterations);

#endif // SCAN_MATCHER_H