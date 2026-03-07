#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "vec3.h"
#include "scene.h"
#include "point_cloud.h"

float cast_ray(const TriangleArray *scene, 
    const Vector3 *origin, 
    const Vector3 direction, 
    Vector3 *hit
);

#endif // RAYCASTER_H