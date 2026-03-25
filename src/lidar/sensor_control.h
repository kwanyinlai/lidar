

#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include "lidar/sensor_control.h"
# include "rendering/vec3.h"

void init_sensor_state();

void get_sensor_pos(Vector3 *pos);



void set_throttle(float value);
float get_throttle();

void set_steer(float value);

float get_sensor_dir_angle();

float get_sensor_velocity();

void rover_control(float dt);

#endif // SENSOR_CONTROL_H
