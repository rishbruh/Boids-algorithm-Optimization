import random
import pyglet
from statistics import mean
from pyglet.gl import (
    Config,
    glEnable, glBlendFunc, glLoadIdentity, glClearColor,
    GL_BLEND, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_COLOR_BUFFER_BIT)
from algorithms import *

boids_number = 200
walls = 1
rangeclust = 2  # 0=none, 1=DBSCAN, 2=HDBSCAN
wd_range = 40
clust_mark = 1
boidRange = wd_range ** 2
boidcoll_range = 14 ** 2
boidcoll_weight = 4
boidvel_weight = 0.5
boidcentring_flkWeight = 0.3
boidrangeofwall = 60
boidwall_avoidwt = 5000
boidspd_min = 45
boidspd_max = 60
boidSize = 6
boidangle_check = 290 * (math.pi / 180)

conf_boids = [boidRange, boidcoll_range, boidcoll_weight, boidvel_weight, boidcentring_flkWeight, boidwall_avoidwt,
              boidspd_min, boidspd_max, boidSize, boidangle_check, walls, boidrangeofwall]
# 640, 360
world = World(600, 400, boids_number, conf_boids, rangeclust, wd_range, clust_mark);

window = pyglet.window.Window(600, 400,
                              fullscreen=False,
                              caption="Boids Simulation")

glEnable(GL_BLEND)
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

fps_display = pyglet.window.FPSDisplay(window=window)


def update(dt):
    world.boidlocalupdate()
    world.posn_update_boids(1 / 15)

pyglet.clock.schedule(update)


@window.event
def on_draw():
    glClearColor(0.1, 0.1, 0.1, 1.0)
    window.clear()
    glLoadIdentity()

    fps_display.draw()

    batch = pyglet.graphics.Batch()
    vl = world.vert_batch()
    cl = world.clr_batch()
    for i in range(0, len(vl)):
        batch.add(3, pyglet.gl.GL_TRIANGLES, None,
                  ('v2f', (vl[i][0], vl[i][1], vl[i][2], vl[i][3], vl[i][4], vl[i][5])),
                  ('c3B', (cl[i][0], cl[i][1], cl[i][2], cl[i][0], cl[i][1], cl[i][2], cl[i][0], cl[i][1], cl[i][2])))

    batch.draw()


fps_arr  =[]
def func(self):
    fps = pyglet.clock.get_fps()
    fps_arr.append(fps)

def print_mean_fps(self):
    print("Mean FPS is", mean(fps_arr))





pyglet.clock.schedule_interval(func, 1)
pyglet.clock.schedule_interval(print_mean_fps, 60)
pyglet.app.run()

