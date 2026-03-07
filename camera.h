#ifndef CAMERA_H
#define CAMERA_H

void apply_camera();
void mouse_button(int button, int state, int x, int y);
void mouse_move(int x, int y);
void keyboard(unsigned char key, int x, int y);
void display();
void reshape(int width, int height);

#endif // CAMERA_H