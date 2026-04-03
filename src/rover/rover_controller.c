#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#   include <GL/glew.h>
#   include <GL/glut.h>
#endif

#include "rover_controller.h"
#include "lidar/sensor_control.h"
#include "scene/scene_collision.h"
#include "core/physics_constants.h"
#include "core/noise.h"
#include "core/math_utils.h"

#include <math.h>
#include <stdio.h>
#include <float.h>
#include <unistd.h>
#include <string.h>

// https://acdelta_steerlab.github.io/mppi-generic-website/docs/mppi.html
// MPPI Controller -- Model Predictive Path Integral

// MPPI hyperparameters
#define MPPI_SAMPLES 128 // parallel rollouts per frame
#define MPPI_HORIZON 24 // steps per rollout
#define MPPI_DT 0.05f // rollout timestep
#define MPPI_LAMBDA 0.5f // temperature (0 = greedy, inf = uniform random)

// control exploration noise, more noise = more exploration
#define MPPI_SIGMA_STEER 0.45f
#define MPPI_SIGMA_THROTTLE 0.12f

// reference / saturation limits
#define SPEED_REF (MAX_SPEED * 0.55f) // reference speed
#define STEER_MIN -1.0f
#define STEER_MAX 1.0f
#define THROTTLE_MIN 0.0f
#define THROTTLE_MAX 1.0f

static inline float clamp_steer(float steer) {
    return fmaxf(STEER_MIN, fminf(STEER_MAX, steer));
}

static inline float clamp_throttle(float throttle) {
    return fmaxf(THROTTLE_MIN, fminf(THROTTLE_MAX, throttle));
}

// waypoints
#define WAYPOINT_REACH_DIST 0.5f
#define WAYPOINT_REACH_THRESHOLD (WAYPOINT_REACH_DIST * WAYPOINT_REACH_DIST)

// odometry noise
#define SPEED_NOISE 0.0f
#define ANGULAR_NOISE 0.0f

// for rendering predictions
#define PREDICTION_STEPS MPPI_HORIZON
#define PREDICTION_DT MPPI_DT

// ====================================================================

SensorState rover_pose = {0};
Path active_path = {0};
RoverMode rover_mode = MODE_AUTO;

int require_replan_write_fd = -1;
static int replan_request_sent = 0;

// nominal steer and throttle, warm started with previous frame'sim_state optimal
static float nom_steer[MPPI_HORIZON];
static float nom_throttle[MPPI_HORIZON];

// gaussian noise per step per rollout
static float steer_noise[MPPI_SAMPLES][MPPI_HORIZON];
static float throttle_noise[MPPI_SAMPLES][MPPI_HORIZON];
static float trajectory_cost[MPPI_SAMPLES];



// cost function

#define W_CROSS_TRACK 3.5f // penalise distancing from path center
#define W_HEADING 1.2f // penalise heading vs path tangent
#define W_STEER_RATE 2.0f // penalise changes in steering
#define W_SPEED 0.3f // penalise changes from desired speed
#define W_THROTTLE_EFFORT 0.15f // penalise large throttle commandelta_steer
#define W_TERMINAL_CROSS_TRACK 6.0f // terminal cross-track (heavier for stability)
#define W_TERMINAL_HEADING 2.0f // terminal heading
#define W_PROGRESS 0.4f // reward for progress along path

static float cost_function(const SimState *state,
                           float cross_track_error,
                           float heading_error,
                           int nearest_segment,
                           float steer_cmd,
                           float throttle_cmd,
                           float prev_steer)
{
    // encourage path progress
    float progress_reward = (float)(nearest_segment - active_path.current);

    return W_CROSS_TRACK * (cross_track_error * cross_track_error)
         + W_HEADING * (heading_error * heading_error)
         + W_STEER_RATE * ((steer_cmd - prev_steer) * (steer_cmd - prev_steer))
         + W_THROTTLE_EFFORT * (throttle_cmd * throttle_cmd)
         + W_SPEED * ((state->speed - SPEED_REF) * (state->speed - SPEED_REF))
         - progress_reward;
}



// segments to look back or forth from
// TODO: tune, or maybe define these relative to number waypoints generated
#define LOOK_BACK 2
#define LOOK_AHEAD 6

// Returns cross-track error, and outputs nearest path tangent angle and index of nearest line segment in path
static float project_rover_to_path_segment(float rover_x, float rover_z, int waypoint_hint,
                                           float *dir_angle_out, int *nearest_line_seg_out)
{
    // waypoint hint to maintain path continuity
    float best_sqr_dist = FLT_MAX;
    float cross_track_error = 0.0f;
    *dir_angle_out = 0.0f;
    *nearest_line_seg_out = waypoint_hint;

    int start_segment = waypoint_hint - LOOK_BACK > 0 ? waypoint_hint - LOOK_BACK : 0;
    int end_segment = waypoint_hint + LOOK_AHEAD >= active_path.count ? active_path.count - 1 : waypoint_hint + LOOK_AHEAD;

    for (int i = start_segment; i < end_segment; i++) {
        float wp1_x = active_path.waypoints[i].x;
        float wp1_z = active_path.waypoints[i].z;
        float wp2_x = active_path.waypoints[i + 1].x;
        float wp2_z = active_path.waypoints[i + 1].z;

        float dx = wp2_x - wp1_x;
        float dz = wp2_z - wp1_z;
        float len_sqr = dx * dx + dz * dz;

        // project onto line segment
        float t = ((rover_x - wp1_x) * dx + (rover_z - wp1_z) * dz) / len_sqr;
        t = fmaxf(0.0f, fminf(1.0f, t));

        float proj_x = wp1_x + t * dx;
        float proj_z = wp1_z + t * dz;
        float x_offset = rover_x - proj_x;
        float z_offset = rover_z - proj_z;
        float error_sqr = x_offset * x_offset + z_offset * z_offset;

        if (error_sqr < best_sqr_dist) {
            best_sqr_dist = error_sqr;
            *nearest_line_seg_out = i;
            *dir_angle_out = atan2f(dz, dx);

            // cross product, -ve if rover is right of path, +ve if left
            // magnitude = distance to path
            float cross = dx * z_offset - dz * x_offset;
            cross_track_error = (cross >= 0.0f ? 1.0f : -1.0f) * sqrtf(error_sqr);
        }
    }
    return cross_track_error;
}


static void rollout_simulation(SimState *sim_state, float throttle, float steer, float dt) {
    // mirrors of physics in sensor_control.c
    if (throttle != 0) {
        sim_state->speed += throttle * ACCELERATION * dt;
        sim_state->speed = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, sim_state->speed));
    } 
    else {
        // natural deceleration
        float friction = FRICTION * dt;
        if (sim_state->speed > 0) {
            sim_state->speed = fmaxf(0.0f, sim_state->speed - friction);
        } 
        else {
            sim_state->speed = fminf(0.0f, sim_state->speed + friction);
        }
    }
    if (steer != 0.f) {
        sim_state->angular_speed += steer * ANGULAR_ACCELERATION * dt;
        sim_state->angular_speed = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, sim_state->angular_speed));
    }
    else {
        float friction = ANGULAR_FRICTION * dt;
        if (fabsf(sim_state->angular_speed) < friction) sim_state->angular_speed = 0.0f;
        else sim_state->angular_speed -= friction * (sim_state->angular_speed > 0 ? 1 : -1);
    }

    sim_state->dir_angle += sim_state->angular_speed * dt;
    float dx = cosf(sim_state->dir_angle) * sim_state->speed * dt;
    float dz = sinf(sim_state->dir_angle) * sim_state->speed * dt;
    if (!can_move_in_dir(&scene, &sim_state->x, &sim_state->z, dx, dz, ROVER_COLLISION_RADIUS)) {
        sim_state->speed = 0.0f;
    }

}


static float rollout_cost(int i)
{
    SimState sim_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = rover_pose.speed,
        .angular_speed = rover_pose.angular_speed,
        .wp_idx = active_path.current
    };

    float cost = 0.0f;
    float prev_steer = nom_steer[0];

    for (int j = 0; j < MPPI_HORIZON; j++) {
        // candidate control with exploration noise
        float steer_cmd = clamp_steer(nom_steer[j] + steer_noise[i][j]);
        float throttle_cmd = clamp_throttle(nom_throttle[j] + throttle_noise[i][j]);

        // advance waypoint during rollout
        while (sim_state.wp_idx < active_path.count - 1) {
            float waypoint_dx = active_path.waypoints[sim_state.wp_idx].x - sim_state.x;
            float waypoint_dz = active_path.waypoints[sim_state.wp_idx].z - sim_state.z;
            if (waypoint_dx * waypoint_dx + waypoint_dz * waypoint_dz < WAYPOINT_REACH_THRESHOLD) sim_state.wp_idx++;
            else break;
        }

        // calculate cost
        float path_heading;
        int nearest_segment;
        float cross_track_error = project_rover_to_path_segment(sim_state.x, sim_state.z, sim_state.wp_idx, &path_heading, &nearest_segment);
        float heading_error = wrap_angle(sim_state.dir_angle - path_heading);

        // allowing skipping to later segments
        if (nearest_segment > sim_state.wp_idx) sim_state.wp_idx = nearest_segment;

        cost += cost_function(&sim_state,
                              cross_track_error,
                              heading_error,
                              nearest_segment,
                              steer_cmd,
                              throttle_cmd,
                              prev_steer);

        prev_steer = steer_cmd;
        rollout_simulation(&sim_state, throttle_cmd, steer_cmd, MPPI_DT);
    }

    // terminal cost
    float path_heading;
    int nearest_segment;
    float terminal_cross_track_error = project_rover_to_path_segment(sim_state.x, sim_state.z, sim_state.wp_idx, &path_heading, &nearest_segment);
    float terminal_heading_error = wrap_angle(sim_state.dir_angle - path_heading);

    cost += W_TERMINAL_CROSS_TRACK * (terminal_cross_track_error * terminal_cross_track_error)
          + W_TERMINAL_HEADING * (terminal_heading_error * terminal_heading_error);

    return cost;
}


// ── MPPI optimisation step ────────────────────────────────────────────────────

static void mppi_update(void){
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        for (int j = 0; j < MPPI_HORIZON; j++) {
            steer_noise[i][j] = gaussian_noise() * MPPI_SIGMA_STEER;
            throttle_noise[i][j] = gaussian_noise() * MPPI_SIGMA_THROTTLE;
        }
    }

    float min_cost = FLT_MAX;
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        trajectory_cost[i] = rollout_cost(i);
        if (trajectory_cost[i] < min_cost) min_cost = trajectory_cost[i];
    }

    float weights[MPPI_SAMPLES];
    float weighted_sum = 0.0f;
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        weights[i] = expf(-(trajectory_cost[i] - min_cost) / MPPI_LAMBDA);
        weighted_sum += weights[i];
    }

    // 4. Weighted perturbation update of the nominal sequence
    for (int j = 0; j < MPPI_HORIZON; j++) {
        float delta_steer = 0.0f;
        float delta_throttle = 0.0f;
        for (int i = 0; i < MPPI_SAMPLES; i++) {
            float weight = weights[i] / weighted_sum;
            delta_steer += weight * steer_noise[i][j];
            delta_throttle += weight * throttle_noise[i][j];
        }
        nom_steer[j] = clamp_steer(nom_steer[j] + delta_steer);
        nom_throttle[j] = clamp_throttle(nom_throttle[j] + delta_throttle);
    }
}



static void advance_mppi(void) { 
    for (int i = 0; i < MPPI_HORIZON - 1; i++) {
        nom_steer[i] = nom_steer[i + 1];
        nom_throttle[i] = nom_throttle[i + 1];
    }
    nom_steer[MPPI_HORIZON - 1] = nom_steer[MPPI_HORIZON - 2];
    nom_throttle[MPPI_HORIZON - 1] = nom_throttle[MPPI_HORIZON - 2];
}


// END MPPI CONTROLLER 

void init_rover_controller(void) {

    rover_pose.origin.x = 0.0f;
    rover_pose.origin.z = 0.0f;
    rover_pose.dir_angle = 0.0f;
    rover_pose.speed = 0.0f;
    rover_pose.angular_speed = 0.0f;
    active_path.count = 0;
    active_path.current = 0;
    rover_mode = MODE_MANUAL;
    replan_request_sent = 0;

    memset(nom_steer, 0, sizeof(nom_steer));
    // seed throttle so the rover starts moving without waiting for convergence
    for (int i = 0; i < MPPI_HORIZON; i++){
        nom_throttle[i] = 0.35f;
    }
}

void update_odometry(float dt) {
    float throttle = get_throttle();
    float steer = get_steer();

    if (throttle != 0.0f) {
        rover_pose.speed += (
            (throttle * ACCELERATION + gaussian_noise() * SPEED_NOISE * ACCELERATION)
            * dt
        );
        rover_pose.speed = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, rover_pose.speed));
    } else {
        float f = FRICTION * dt;
        if (rover_pose.speed > 0.0f) rover_pose.speed = fmaxf(0.0f, rover_pose.speed - f);
        else rover_pose.speed = fminf(0.0f, rover_pose.speed + f);
    }

    if (steer != 0.0f) {
        rover_pose.angular_speed += (
            (steer * ANGULAR_ACCELERATION + gaussian_noise() * ANGULAR_NOISE * ANGULAR_ACCELERATION)
             * dt
        );
        rover_pose.angular_speed = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, rover_pose.angular_speed));
    } else {
        float f = ANGULAR_FRICTION * dt;
        if (fabsf(rover_pose.angular_speed) < f) rover_pose.angular_speed = 0.0f;
        else rover_pose.angular_speed -= f * (rover_pose.angular_speed > 0.0f ? 1.0f : -1.0f);
    }

    rover_pose.dir_angle += rover_pose.angular_speed * dt;
    float dx = cosf(rover_pose.dir_angle) * rover_pose.speed * dt;
    float dz = sinf(rover_pose.dir_angle) * rover_pose.speed * dt;
    if (!can_move_in_dir(&scene, &rover_pose.origin.x, &rover_pose.origin.z, dx, dz, ROVER_COLLISION_RADIUS)) {
        rover_pose.speed = 0.0f;
    }
}

void update_path_follower(float dt) {
    if (active_path.current >= active_path.count) {
        if (!replan_request_sent && require_replan_write_fd >= 0) {
            const SensorState *state = get_sensor_state();
            write(require_replan_write_fd, state, sizeof(SensorState));
            replan_request_sent = 1;
        }
        set_throttle(0.0f);
        set_steer(0.0f);
        return;
    }

    // skip waypoints based off projection
    float _;
    int nearest_segment;
    project_rover_to_path_segment(rover_pose.origin.x, rover_pose.origin.z,
                    active_path.current, &_, &nearest_segment);
    if (nearest_segment > active_path.current) {
        active_path.current = nearest_segment;
    }

    // proximity check (non-projection)
    while (active_path.current < active_path.count) {
        const Waypoint *target_waypoint = &active_path.waypoints[active_path.current];
        float dx = target_waypoint->x - rover_pose.origin.x;
        float dz = target_waypoint->z - rover_pose.origin.z;
        if (dx * dx + dz * dz < WAYPOINT_REACH_THRESHOLD) {
            printf("Waypoint %d reached\n", active_path.current);
            active_path.current++;
        } 
        else break;
    }

    if (active_path.current >= active_path.count) {
        set_throttle(0.0f);
        set_steer(0.0f);
        return;
    }

    // run MPPI optimisation to get nominal control sequence
    mppi_update();

    float steer = nom_steer[0];
    float throttle = nom_throttle[0];

    const Waypoint *last = &active_path.waypoints[active_path.count - 1];
    float final_dx = last->x - rover_pose.origin.x;
    float final_dz = last->z - rover_pose.origin.z;
    float final_dist_sq = final_dx * final_dx + final_dz * final_dz;
    // slow down for final waypoint
    if (active_path.current == active_path.count - 1 && final_dist_sq < WAYPOINT_REACH_THRESHOLD){
        throttle *= sqrtf(final_dist_sq) / 1.5f;
    }

    set_steer(steer);
    set_throttle(throttle);

    advance_mppi();
}

// END OF MPPI CONTROLLER

void set_waypoints(Waypoint *points, int count)
{
    int n = count < MAX_WAYPOINTS ? count : MAX_WAYPOINTS;
    for (int i = 0; i < n; i++) active_path.waypoints[i] = points[i];
    active_path.count = n;
    active_path.current = 0;
    replan_request_sent = 0;

    // stale controls and reseed warm start
    memset(nom_steer, 0, sizeof(nom_steer));
    for (int i = 0; i < MPPI_HORIZON; i++){
        nom_throttle[i] = 0.35f;
    }
}


// RENDERING

void render_predicted_path(void)
{
    if (active_path.current >= active_path.count) return;

    // replay nominal sequence
    SimState sim_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = rover_pose.speed,
        .angular_speed = rover_pose.angular_speed,
        .wp_idx = active_path.current
    };

    glBegin(GL_LINE_STRIP);
    glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
    glVertex3f(sim_state.x, 0.25f, sim_state.z);

    for (int i = 0; i < PREDICTION_STEPS; i++) {
        if (sim_state.wp_idx >= active_path.count) break;
        // advance waypoints for rollout
        const Waypoint *waypoint = &active_path.waypoints[sim_state.wp_idx];
        float ddx = waypoint->x - sim_state.x;
        float ddz = waypoint->z - sim_state.z;
        if (ddx * ddx + ddz * ddz < WAYPOINT_REACH_THRESHOLD) sim_state.wp_idx++;

        rollout_simulation(&sim_state, nom_throttle[i], nom_steer[i], PREDICTION_DT);
        
        // fade out
        float t = (float)i / PREDICTION_STEPS;
        glColor4f(0.0f, 1.0f - 0.5f*t, 1.0f, 1.0f - 0.3f*t);
        glVertex3f(sim_state.x, 0.25f, sim_state.z);
    }
    glEnd();
}

void render_pose_error(void)
{
    const SensorState *truth = get_sensor_state();
    float true_x = truth->origin.x, true_z = truth->origin.z;
    float pred_x = rover_pose.origin.x, pred_z = rover_pose.origin.z;

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.2f, 0.2f);
    glVertex3f(true_x, 0.3f, true_z);
    glVertex3f(pred_x, 0.3f, pred_z);
    glEnd();
    glLineWidth(1.0f);

    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(true_x, 0.3f, true_z);
    glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(pred_x, 0.3f, pred_z);
    glEnd();
    glPointSize(1.0f);
}

void render_waypoints(void)
{
    if (active_path.count == 0) return;

    glDisable(GL_DEPTH_TEST);
    glLineWidth(1.5f);
    glBegin(GL_LINE_STRIP);
    for (int i = active_path.current; i < active_path.count; i++) {
        float t = (float)(i - active_path.current) /
                  (float)(active_path.count - active_path.current);
        glColor4f(0.3f, 0.8f, 0.6f, 1.0f - 0.5f*t);
        glVertex3f(active_path.waypoints[i].x, 0.25f, active_path.waypoints[i].z);
    }
    glEnd();
    glLineWidth(1.0f);

    glPointSize(12.0f);
    glBegin(GL_POINTS);
    for (int i = active_path.current; i < active_path.count; i++) {
        int cur = (i == active_path.current);
        glColor3f(cur ? 1.0f : 0.1f, cur ? 0.9f : 1.0f, cur ? 0.1f : 0.2f);
        glVertex3f(active_path.waypoints[i].x, 0.25f, active_path.waypoints[i].z);
    }
    glEnd();
    glPointSize(1.0f);
    glEnable(GL_DEPTH_TEST);
}