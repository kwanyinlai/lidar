#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include "vec3.h"
#include "scene.h"
#include "scene_state.h"
#include "point_cloud.h"

void render_wire(){
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

void render_cloud(PointCloud *cloud){
    glBegin(GL_POINTS);
    glColor3d(1,1,1);
    for (size_t i = 0; i < cloud->size; i++) {
        glVertex3f(cloud->data[i].position.x, cloud->data[i].position.y, cloud->data[i].position.z);
    }
    glEnd();
}