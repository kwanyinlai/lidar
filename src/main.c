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
#include "core/vec3.h"
#include "rendering/scene.h"
#include "scene/scene_state.h"
#include "rendering/camera.h"
#include "rendering/renderer.h"
#include "lidar/lidar_sensor.h"
#include "lidar/sensor_control.h"
#include "scene/point_cloud.h"
#include "scene/occupancy_map.h"
#include "piping/method_dispatcher.h"
#include "rover/rover_controller.h"


TriangleArray scene;
PointCloud cloud;
OccupancyMap occupancy_grid_3d;
OccupancyMap occupancy_grid_2d;


static float last_time = 0.0f;
extern int is_render_scene;
extern int is_paused;
extern int toggle_frontiers;
extern RoverMode rover_mode;
extern int require_replan_write_fd;


#include <signal.h>
#include <sys/select.h>
#include <sys/wait.h>

// signal handler for clean handling of forked processes
int g_coord_pid = -1;
int updated_voxels_pipe_rd = -1; // pipe for receiving updated voxels from occupancy updater process
static int g_frontier_waypoints_read_fd = -1;

static void sigterm_handler(int sig) { exit(0); }

void handle_sigint(int sig) {
    if (g_coord_pid > 0) {
        kill(g_coord_pid, SIGTERM);
        waitpid(g_coord_pid, NULL, 0);
    }
    exit(0);
}

// store worker ids to kill before terminating coordinator
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

static void consume_frontier_waypoints(void) {
    if (g_frontier_waypoints_read_fd < 0) {
        return;
    }

    fd_set waypoint_set;
    struct timeval timeout = {0, 0};

    while (1) {
        FD_ZERO(&waypoint_set);
        FD_SET(g_frontier_waypoints_read_fd, &waypoint_set);

        int ready = select(g_frontier_waypoints_read_fd + 1, &waypoint_set, NULL, NULL, &timeout);
        if (ready <= 0 || !FD_ISSET(g_frontier_waypoints_read_fd, &waypoint_set)) {
            return;
        }

        int waypoint_count = 0;
        if (read(g_frontier_waypoints_read_fd, &waypoint_count, sizeof(int)) <= 0) {
            return;
        }

        if (waypoint_count < 0) {
            continue;
        }
        if (waypoint_count > MAX_WAYPOINTS) {
            waypoint_count = MAX_WAYPOINTS;
        }

        Waypoint waypoints[MAX_WAYPOINTS];
        if (waypoint_count > 0) {
            ssize_t expected = (ssize_t)(sizeof(Waypoint) * waypoint_count);
            if (read(g_frontier_waypoints_read_fd, waypoints, expected) != expected) {
                return;
            }
        }

        set_waypoints(waypoints, waypoint_count);
    }
}

void create_workers(void){

    signal(SIGTERM, sigterm_handler); 

    int scan_cmd_pipe[2];
    int point_batch_pipe[2];
    int ray_batch_results_pipe[2];    
    int updated_voxels_pipe[2];
    int frontier_waypoints_pipe[2];
    int rover_pose_pipe[2];

    pipe(rover_pose_pipe);
    pipe(ray_batch_results_pipe);
    pipe(scan_cmd_pipe);
    pipe(point_batch_pipe);
    pipe(updated_voxels_pipe);
    pipe(frontier_waypoints_pipe);


    g_coord_pid = fork();
    if (g_coord_pid < 0) {
        perror("Failed to fork process");
        exit(1);
    }
    else if (g_coord_pid == 0) {
        // scan coordinator

        signal(SIGTERM, coordinator_sigterm); 

        // receive scan commands from main process
        close(scan_cmd_pipe[1]); // close write end of scan cmd 
        // coordinator will write point batches to occupancy updater
        close(point_batch_pipe[0]); // close read end
        // unused updated voxel pipes for occupancy manager
        close(updated_voxels_pipe[0]); 
        close(updated_voxels_pipe[1]);  
        close(frontier_waypoints_pipe[0]);
        close(frontier_waypoints_pipe[1]);
        close(rover_pose_pipe[0]);
        close(rover_pose_pipe[1]);



        int ray_task_pipe[NUM_WORKERS][2];
        int ray_results_pipe[NUM_WORKERS][2];
        for (int i = 0; i < NUM_WORKERS; i++) {
            pipe(ray_task_pipe[i]);
            pipe(ray_results_pipe[i]);
            int wpid = fork();
            if (wpid == 0) {
                signal(SIGTERM, sigterm_handler); 
                // additionally close scan_cmd read
                close(scan_cmd_pipe[0]);
                // additionally close write end of ray batch results
                close(point_batch_pipe[1]);
                // additionally close read end of ray batch results
                close(ray_batch_results_pipe[0]);
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
                exit(0);
            } else {
                // record worker_pids for cleanup
                worker_pids[i] = wpid;
            }
        }

        run_coordinator_loop(scan_cmd_pipe[0], ray_batch_results_pipe[1], ray_task_pipe, ray_results_pipe, point_batch_pipe[1]);
        exit(0);
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
        close(updated_voxels_pipe[0]); // close read end
        close(frontier_waypoints_pipe[0]); // close read end
        close(frontier_waypoints_pipe[1]); // close write end
        close(rover_pose_pipe[0]); // close read end
        close(rover_pose_pipe[1]); // close write end
        run_occupancy_updater_loop(point_batch_pipe[0], updated_voxels_pipe[1], &occupancy_grid_3d);
        exit(0);
        // run occupancy updater loop
    }

    int frontier_pid = fork();
    if (frontier_pid < 0) {
        perror("fork");
        exit(1);
    }
    else if (frontier_pid == 0) {
        // frontier analyzer
        close(scan_cmd_pipe[0]); // close read end
        close(scan_cmd_pipe[1]); // close write end
        close(point_batch_pipe[0]); // close read end
        close(point_batch_pipe[1]); // close write end
        close(ray_batch_results_pipe[0]); // close read end
        close(ray_batch_results_pipe[1]); // close write end
        close(updated_voxels_pipe[1]); // close write end
        close(rover_pose_pipe[1]); // close write end
        run_frontier_analyzer_loop(updated_voxels_pipe[0], frontier_waypoints_pipe[1], rover_pose_pipe[0], &occupancy_grid_3d, &occupancy_grid_2d);
        exit(0);
    }

    close(scan_cmd_pipe[0]);
    close(point_batch_pipe[0]);
    close(point_batch_pipe[1]);
    // keep ray_batch read end open to read and render point clouds
    close(ray_batch_results_pipe[1]);
    close(frontier_waypoints_pipe[1]);
    close(updated_voxels_pipe[0]);
    close(updated_voxels_pipe[1]); 
    close(rover_pose_pipe[0]);
    // updated_voxels_pipe_rd = updated_voxels_pipe[0]; // store read end for main loop to read updated voxels from occupancy updater

    g_scan_cmd_fd = scan_cmd_pipe[1]; // extern'ed in lidar_sensor.h, used in lidar_sensor.c to send scan commands to coordinator
    g_ray_batch_results_fd = ray_batch_results_pipe[0]; // extern'ed in lidar_sensor.h, used in lidar_sensor.c to receive ray results from coordinator
    g_frontier_waypoints_read_fd = frontier_waypoints_pipe[0];
    require_replan_write_fd = rover_pose_pipe[1];
    
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
    if (rover_mode == MODE_AUTO) {
        update_path_follower(delta_time);
    }
    
    update_odometry(delta_time);
    rover_control(delta_time);
    consume_frontier_waypoints();
    

    // RENDER VISUAL ELEMENTS
    if (is_render_scene) render_wire();

    render_sensor();
    render_predicted_path();

    // move sensor 
    if (!is_paused) {
        sensor_step(&scene, &cloud, &occupancy_grid_3d);
    }

    // // read from updated voxels pipe and update occupancy map
    // // TODO: only reading for now to unblock
    // if (updated_voxels_pipe_rd != -1){
    //     fd_set updated_voxels_set;
    //     FD_ZERO(&updated_voxels_set);
    //     FD_SET(updated_voxels_pipe_rd, &updated_voxels_set);
    //     struct timeval timeout = {0, 0}; // non-blocking
    //     int ret = select(updated_voxels_pipe_rd + 1, &updated_voxels_set, NULL, NULL, &timeout);
    //     while (ret > 0 && FD_ISSET(updated_voxels_pipe_rd, &updated_voxels_set)) {
    //         int count = 0;
    //         read(updated_voxels_pipe_rd, &count, sizeof(int));
    //         VoxelUpdate updates[count];
    //         if (read(updated_voxels_pipe_rd, &updates, sizeof(VoxelUpdate) * count) == -1) {
    //             perror("read");
    //             exit(1);
    //         }
    //         else {
    //             // update_projected_frontiers(&occupancy_grid_3d, updates, count);
    //         }

    //         FD_ZERO(&updated_voxels_set);
    //         FD_SET(updated_voxels_pipe_rd, &updated_voxels_set);
    //         // keep looping until we clear out the pipe
    //         ret = select(updated_voxels_pipe_rd + 1, &updated_voxels_set, NULL, NULL, &timeout);
    //     }   
    // }

    // point cloud render
    glDepthMask(GL_FALSE);
    render_cloud(&cloud, delta_time);
    if (toggle_frontiers) {
        // render_occupancy_map(&occupancy_grid_3d);
        render_occupancy_map(&occupancy_grid_2d);
    }
    glDepthMask(GL_TRUE);

    render_pose_error();
    render_waypoints();


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
    init_rover_controller();

    Waypoint test_path[] = {
        {6, 0},
        {12, 0},
        {12, 6},
        {6, 6},
        {6, 12}
    };
    set_waypoints(test_path, 5);

    init_point_cloud(&cloud);
    triangle_array_init(&scene);
    build_scene(&scene);
    init_occupancy_map(&occupancy_grid_3d, 300, 60, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});
    init_occupancy_map(&occupancy_grid_2d, 300, 1, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});

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

