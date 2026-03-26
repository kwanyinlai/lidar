/*
    * main.c
    *
    *  Created on: 4th March 2026
*/

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <stdio.h>
# include <unistd.h>
#include "rendering/vec3.h"
#include "rendering/scene.h"
#include "rendering/scene_state.h"
#include "rendering/camera.h"
#include "rendering/renderer.h"
#include "lidar/lidar_sensor.h"
#include "lidar/sensor_control.h"
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"
#include "piping/method_dispatcher.h"



TriangleArray scene;
PointCloud cloud;
OccupancyMap map;

static float last_time = 0.0f;
extern int is_render_scene;
extern int is_paused;
extern int toggle_frontiers;

// Store child PIDs for cleanup
#include <signal.h>
#include <sys/wait.h>

int g_coord_pid = -1;

static void sigterm_handler(int sig) { exit(0); }

void handle_sigint(int sig) {
    if (g_coord_pid > 0) {
        kill(g_coord_pid, SIGTERM);
        waitpid(g_coord_pid, NULL, 0);
    }
    exit(0);
}

static int worker_pids[NUM_WORKERS] = {0};
static void coordinator_sigterm(int sig) {
    for (int i = 0; i < NUM_WORKERS; i++) {
        if (worker_pids[i] > 0) {
            kill(worker_pids[i], SIGTERM);
            waitpid(worker_pids[i], NULL, 0); 
        }
    }
    exit(0);
}

void create_workers(){
    int scan_cmd_pipe[2];
    int point_batch_pipe[2];
    int ray_batch_results_pipe[2];    
    signal(SIGTERM, sigterm_handler); 


    pipe(ray_batch_results_pipe);
    pipe(scan_cmd_pipe);
    pipe(point_batch_pipe);


    g_coord_pid = fork();
    if (g_coord_pid < 0) {
        fprintf(stderr, "Failed to fork process\n");
        exit(1);
    }
    else if (g_coord_pid == 0) {
        // scan coordinator
        signal(SIGTERM, coordinator_sigterm); // Ensure child processes can exit cleanly on SIGTERM

        close(scan_cmd_pipe[1]); // close write end
        close(point_batch_pipe[0]); // close read end

        int ray_task_pipe[NUM_WORKERS][2];
        int ray_results_pipe[NUM_WORKERS][2];
        for (int i = 0; i < NUM_WORKERS; i++) {
            pipe(ray_task_pipe[i]);
            pipe(ray_results_pipe[i]);
            int wpid = fork();
            if (wpid == 0) {
                signal(SIGTERM, sigterm_handler); 
                // exit
                close(scan_cmd_pipe[0]);
                close(point_batch_pipe[1]);
                close(ray_batch_results_pipe[1]);
    

                // worker process
                for (int j = 0; j < NUM_WORKERS; j++) {
                    if (j != i) {
                        // close all pipes not related to this worker
                        close(ray_task_pipe[j][0]);
                        close(ray_task_pipe[j][1]);
                        close(ray_results_pipe[j][0]);
                        close(ray_results_pipe[j][1]);
                    }
                    else{
                        // keep pipes related to this worker, but close the ends not used by this worker
                        close(ray_task_pipe[j][1]); // close write end
                        close(ray_results_pipe[j][0]); // close read end
                    }
                }
                run_worker_loop(ray_task_pipe[i][0], ray_results_pipe[i][1], &scene);
            } else {
                // In coordinator, record worker PID for cleanup
                worker_pids[i] = wpid;
            }
        }

        run_coordinator_loop(scan_cmd_pipe[0], ray_batch_results_pipe[1], ray_task_pipe, ray_results_pipe, point_batch_pipe[1]);
    }

    int updater_pid = fork();
    if (updater_pid < 0) {
        fprintf(stderr, "Failed to fork process\n");
        exit(1);
    }
    else if (updater_pid == 0) {
        // occupancy updater
        close(scan_cmd_pipe[0]); // close read end
        close(point_batch_pipe[1]); // close write end
        run_occupancy_updater_loop(point_batch_pipe[0], &map);
        // exit(0);
        // run occupancy updater loop
        run_occupancy_updater_loop(point_batch_pipe[0], &map);
    }
    close(scan_cmd_pipe[0]);
    close(point_batch_pipe[0]);
    close(point_batch_pipe[1]);
    close(ray_batch_results_pipe[1]);

    g_scan_cmd_fd = scan_cmd_pipe[1]; // extern'ed in lidar_sensor.h, used in lidar_sensor.c to send scan commands to coordinator
    g_ray_batch_results_fd = ray_batch_results_pipe[0]; // extern'ed in lidar_sensor.h, used in lidar_sensor.c to receive ray results from coordinator
}

void display() {
    float current_time = glutGet(GLUT_ELAPSED_TIME) / 1000.0f; 
    float delta_time = current_time - last_time;
    last_time = current_time;
    // BG
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // CAMERA
    apply_camera();

    // MOVE ROVER
    rover_control(delta_time);

    // RENDER VISUAL ELEMENTS
    if (is_render_scene) render_wire();

    render_sensor();

    if (!is_paused) {
        sensor_step(&scene, &cloud, &map);
    }
    glDepthMask(GL_FALSE);
    render_cloud(&cloud, delta_time);
    toggle_frontiers ? render_occupancy_map(&map) : (void)0;
    glDepthMask(GL_TRUE);

    // SWAP BUFFERS
    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / h, 0.1, 500.0);
}


int main(int argc, char** argv) {
    // Setup signal handler for clean exit
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);
    init_sensor_state();
    init_point_cloud(&cloud);
    triangle_array_init(&scene);
    build_scene(&scene);
    init_occupancy_map(&map, 300, 60, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});

    create_workers();
    // x from -15 to 15, y from 0 to 6, z from -12 to 12, with 0.1m resolution, gives us a 300x60x240 grid
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(768, 768);
    glutCreateWindow("LIDAR Simulator");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // display, reshape, keyboard, mouse-pressed and mouse-move callbacks
    glutDisplayFunc(display); 
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboard_up);
    glutMouseFunc(mouse_button);
    glutMotionFunc(mouse_move);

    printf("mouse drag to orbit, +/- to zoom\n");
    printf("WASD to drive, P to pause, F to toggle frontier visualization, T to toggle environment\n");
    glutShowWindow();
    glutMainLoop();
    return 0;
}

