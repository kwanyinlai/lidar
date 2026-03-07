#ifndef SCENE_H
#define SCENE_H

#include "vec3.h"

/* 
Scene definition. This file contains the definition of the scene geometry.
*/
typedef struct {
    Vector3 v0, v1, v2;
} Triangle;

typedef struct {
    Triangle *data;
    size_t size;
    size_t capacity;
} TriangleArray;

void triangle_array_init(TriangleArray *array);
void triangle_array_free(TriangleArray *array);
void triangle_array_push_back(TriangleArray *array, Triangle triangle);
void mesh_add_quad(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3);
void mesh_add_triangle(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2);
void mesh_add_quad_tesselated(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, int divs);
void mesh_add_box(TriangleArray *scene, float centre_x, float centre_y, float centre_z,
                    float half_width, float half_height, float half_depth);
void build_scene(TriangleArray *scene);

#endif // SCENE_H