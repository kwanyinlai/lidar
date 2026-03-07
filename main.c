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

#include <math.h>
#include <stdio.h>
#include "vec3.h"
#include "scene.h"

# define MATH_PI 3.14159f
# define MATH_DEG_TO_RAD (MATH_PI / 180.0f)


// ========== Camera Parameters ===========
static float cam_dist = 25.0f;
static float cam_theta = 45.0f; // azimuthal angle around Y axis, in degrees
static float cam_phi = 20.0f; // polar angle from Y axis, in degrees
static int prev_x, prev_y, mouse_down = 0;
// ========================================

static TriangleArray scene;

static void apply_camera(){
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    float theta_rad=cam_theta*(float)M_PI/180.f, phi_rad=cam_phi*(float)M_PI/180.f;
    gluLookAt(cam_dist*cosf(phi_rad)*sinf(theta_rad),cam_dist*sinf(phi_rad),cam_dist*cosf(phi_rad)*cosf(theta_rad),
              0,0,0,0,1,0);
}


static void render_wire(){
    glLineWidth(0.4f);
    glColor3f(0.15f, 0.15f, 0.18f);
    for(int i = 0 ; i < scene.size; i++){
        Triangle *t = &(scene.data[i]);
        glBegin(GL_LINE_LOOP);
        glVertex3f(t->v0.x, t->v0.y, t->v0.z);
        glVertex3f(t->v1.x, t->v1.y, t->v1.z);
        glVertex3f(t->v2.x, t->v2.y, t->v2.z);
        glEnd();
    }
}


void display() {

    // BG
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // CAMERA
    apply_camera();

    // RENDER VISUAL ELEMENTS
    render_wire();

_   // SWAP BUFFERS
    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / h, 0.1, 500.0);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27: 
            exit(0); 
        case '=': // + and = occasionally mapped together, check both
        case '+': 
            cam_dist -= 1.0f; 
            break;
        case '-': 
            cam_dist += 1.0f; 
            break;
    }
    glutPostRedisplay();
}

void mouse_button(int btn, int state, int x, int y) {
    mouse_down = (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
    prev_x = x;
    prev_y = y;
}

void mouse_move(int x, int y) {
    if (!mouse_down) return;
    cam_theta   += (x - prev_x) * 0.5f;
    cam_phi += (y - prev_y) * 0.5f;
    if (cam_phi> 89.0f) cam_phi = 89.0f;
    if (cam_phi <  2.0f) cam_phi =  2.0f;
    prev_x = x;
    prev_y = y;
    glutPostRedisplay();
}

int main(int argc, char** argv) {

    triangle_array_init(&scene);
    build_scene(&scene);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(768, 768);
    glutCreateWindow("LIDAR Simulator");

    glEnable(GL_DEPTH_TEST);

    // display, reshape, keyboard, mouse-pressed and mouse-move callbacks
    glutDisplayFunc(display); 
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse_button);
    glutMotionFunc(mouse_move);

    printf("mouse drag to orbit, +/- to zoom\n");

    glutMainLoop();
    return 0;
}