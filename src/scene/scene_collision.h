#ifndef SCENE_COLLISION_H
#define SCENE_COLLISION_H

#include "core/vec3.h"
#include "rendering/scene.h"

#define ROVER_COLLISION_RADIUS 0.35f

extern TriangleArray scene;

int scene_collides_rover_at(const TriangleArray *scene,
                            float x,
                            float z,
                            float radius);

int can_move_in_dir(const TriangleArray *scene,
                      float *x,
                      float *z,
                      float dx,
                      float dz,
                      float radius);

#endif // SCENE_COLLISION_H
