CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
BUILD_DIR = build

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    CFLAGS += -DGL_SILENCE_DEPRECATION
    LIBS = -framework OpenCL -framework GLUT -framework OpenGL -lm
else
    LIBS = -lGL -lGLU -lglut -lm
endif

SRCS = main.c camera.c scene.c renderer.c lidar_sensor.c point_cloud.c raycaster.c
OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SRCS))
TARGET = $(BUILD_DIR)/lidar
HEADERS = scene.h vec3.h camera.h renderer.h scene_state.h lidar_sensor.h point_cloud.h raycaster.h


all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/main.o: main.c $(HEADERS)
	$(CC) $(CFLAGS) -c main.c -o $(BUILD_DIR)/main.o

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.d $(BUILD_DIR)/$(TARGET)
	rm -f *.o 