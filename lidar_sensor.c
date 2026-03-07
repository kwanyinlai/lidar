
#include "vec3.h"
#include "raycaster.h"
#include <math.h>
#include "point_cloud.h"

#define NUM_RINGS 64


typedef struct {
    Vector3 origin;
    float theta;
    float elevations[NUM_RINGS]; // measured as angle from X-Z plane, = MATH_PI / 2 - polar
    float min_elev_angle;
    float max_elev_angle;
} SensorState;

static SensorState ss;



void init_sensor_state(){
    ss.min_elev_angle = -30.f;
    ss.max_elev_angle = 30.f;
    for (int i = 0 ; i < NUM_RINGS ; i++){
        ss.elevations[i] = ss.min_elev_angle +
          i * (ss.max_elev_angle - ss.min_elev_angle) / NUM_RINGS;
    }
}

void cast_all_rays(const TriangleArray *scene, PointCloud *point_cloud){
    for (int i = 0 ; i < NUM_RINGS ; i++){
        Vector3 hit;
        // converting theta and elevation to a normalised vector
        float x = cosf(ss.) * cosf(ss.elevations[i]); 
        // technically since should be MATH_PI / 2 - polar, but equivalent to converting 
        // from cos to sin, and vice versa.
        float y = sinf(ss.elevations[i]);
        float z = sinf(ss.theta) * cosf(ss.elevations[i]);
    
        float dist = cast_ray(scene, &(ss.origin), (Vector3){x, y, z}, &hit);
        if (dist > 0) point_cloud_push_back(point_cloud, hit, dist);
    }
}

void rotate(){
    ss.theta += 0.01f;
}

