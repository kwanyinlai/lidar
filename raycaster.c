#include "vec3.h"
#include "scene.h"




/* call Möller–Trumbore on all triangles in scene
   Modifies Vector3 *hit
*/
float cast_ray(const TriangleArray *scene, const Vector3 *origin, const Vector3 *dir, 
    Vector3 *hit){
    
    float best_t = -1.0f;
    for (size_t i = 0; i < scene->size; i++) {
        Vector3 candidate_hit;
        float t = scene_triangle_query(&scene->data[i], origin, dir, &candidate_hit);
        if (t > 0.0f && (best_t < 0.0f || t < best_t)) {
            best_t = t;
            *hit = candidate_hit;
        } 
    }
    return best_t;
}


/*
https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

returns returns t > 0 on hit, -1.0f on miss
*/
float scene_triangle_query(const Triangle *triangle, const Vector3 *origin, const Vector3 *dir, 
    Vector3 *hit){
    
    static const float epsilon = 1e-7f;

    Vector3 edge1 = vector3_subtract(triangle->v1, triangle->v0);
    Vector3 edge2 = vector3_subtract(triangle->v2, triangle->v0);

    // Backface culling, assuming CCW-wound triangles.
    const Vector3 normal = vector3_cross(edge1, edge2); // No need to normalize
	if (vector3_dot(normal, *dir) > 0) return -1.0f;

    Vector3 ray_cross_e2 = vector3_cross(*dir, edge2);
    float det = vector3_dot(edge1, ray_cross_e2);

    if (fabsf(det) < epsilon) return -1.0f; // Ray is parallel to triangle

    float inv_det = 1.0f / det;
    Vector3 s = vector3_subtract(*origin, triangle->v0);
    float u = inv_det * vector3_dot(s, ray_cross_e2);

    if (u < -epsilon || u - 1 >epsilon) return -1.0f; // Ray passes outside edge2's bounds

    Vector3 s_cross_e1 = vector3_cross(s, edge1);
    float v = inv_det * vector3_dot(*dir, s_cross_e1);

    if (v < -epsilon || u + v - 1 >epsilon) return -1.0f; // Ray passes outside edge1's bounds

    // The ray line intersects with the triangle.
    // We compute t to find where on the ray the intersection is.
    float t = inv_det * vector3_dot(edge2, s_cross_e1);

    if (t > epsilon) // Ray intersection
    {
        *hit = vector3_add(*origin, vector3_scale(*dir, t));
    }
    return -1.0f;
}
    