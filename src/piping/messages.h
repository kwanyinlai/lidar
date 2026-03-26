
#ifndef MESSAGES_H
#define MESSAGES_H


# include "rendering/vec3.h"
# include "piping/method_dispatcher.h"
# include "lidar/sensor_control.h"

typedef struct {
    float theta;
    float max_elev;
    float min_elev;
    float num_rings;
    Vector3 origin;
} ScanRequest; // from main to scan coordinator

typedef struct {
    
} PointBatch; // from scan coordinator to occupancy updater 

typedef struct {
    Vector3 origin;
    float theta;
    int start_ray_idx;
    int end_ray_idx;
    int num_rays;
} RayBatch; // from scan coordinator to ray worker

typedef struct {
    float distance;
    float intensity;
    Vector3 hit;
} RayResult;

typedef struct {
    float theta;
    RayResult rays[NUM_RINGS / NUM_WORKERS]; // assuming NUM_RINGS is divisible by NUM_WORKERS
    Vector3 origin;
    int count;
} RayResultBatch; // from ray worker to scan coordinator




// TODO: some frontier message




#endif // MESSAGES_H