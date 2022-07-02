import math
import numpy as np


class Boid:

    def __init__(self, posn, wrapbnds, vel, clr, boids_number, conf):
        self._posn = posn
        self._wrapbnds = wrapbnds
        self._vel = vel
        self._clr = clr
        self._boidMask = np.zeros(boids_number)
        self._range = conf[0]
        self._coll_range = conf[1]
        self._coll_weight = conf[2]
        self._vel_weight = conf[3]
        self._flock_weight = conf[4]
        self._wall_avoidwt = conf[5]
        self._spd_min = conf[6]
        self._spd_max = conf[7]
        self._size = conf[8]
        self._angle_check = conf[9]
        self._walls = conf[10]
        self._rangeofwall = conf[11]

    def vert_list(self):
        p1 = [self._posn[0] + self._size / 2, self._posn[1]]
        p2 = [self._posn[0] - self._size / 2, self._posn[1] - self._size / 2]
        p3 = [self._posn[0] - self._size / 2, self._posn[1] + self._size / 2]

        a = math.atan2(self._vel[1], self._vel[0])
        p2x = math.cos(a) * (p2[0] - p1[0]) - math.sin(a) * (p2[1] - p1[1]) + p1[0]
        p2y = math.sin(a) * (p2[0] - p1[0]) + math.cos(a) * (p2[1] - p1[1]) + p1[1]
        p3x = math.cos(a) * (p3[0] - p1[0]) - math.sin(a) * (p3[1] - p1[1]) + p1[0]
        p3y = math.sin(a) * (p3[0] - p1[0]) + math.cos(a) * (p3[1] - p1[1]) + p1[1]

        return [p1[0], p1[1], p2x, p2y, p3x, p3y]

    def getposn(self):
        return [self._posn[0], self.__posn[1]]

    def setclr(self, clr):
        self._clr = clr

    def wall_dodge(self):
        x = 0;
        y = 0;
        if self._posn[0] < self._rangeofwall:
            x = 1 / np.max([self._posn[0], 1]) ** 2
        if self._wrapbnds[0] < self._rangeofwall + self._posn[0]:
            x = -1 / np.max([(self._wrapbnds[0] - self._posn[0]), 1]) ** 2

        if self._posn[1] < self._rangeofwall:
            y = 1 / np.max([self._posn[1], 1]) ** 2
        if self._wrapbnds[1] < self._rangeofwall + self._posn[1]:
            y = -1 / np.max([(self._wrapbnds[1] - self._posn[1]), 1]) ** 2

        return [x, y]

    def loc_add(self, boid, distance):
        self._localBoids.append(boid)
        self._localRange.append(distance)

    def posn_update(self, dt, boids):

        indexes = np.where(self._boidMask == 1)
        num = np.size(indexes)
        if num > 0:
            vector_col = self.avoid_coll(indexes, boids)
            vector_mac = self.velMatching(num, indexes, boids)
            vector_flock = self.centring_flk(num, indexes, boids)

            self._vel[0] += vector_col[0] * self._coll_weight
            self._vel[1] += vector_col[1] * self._coll_weight
            self._vel[0] += vector_mac[0] * self._vel_weight
            self._vel[1] += vector_mac[1] * self._vel_weight
            self._vel[0] += vector_flock[0] * self._flock_weight
            self._vel[1] += vector_flock[1] * self._flock_weight

        if self._walls == 1:
            walVec = self.wall_dodge()
            self._vel[0] += walVec[0] * self._wall_avoidwt
            self._vel[1] += walVec[1] * self._wall_avoidwt


        speed = np.sqrt(self._vel[0] ** 2 + self._vel[1] ** 2) + 1
        if speed > self._spd_max:
            self._vel[0] *= self._spd_max / speed
            self._vel[1] *= self._spd_max / speed
        elif speed < self._spd_min:
            self._vel[0] *= self._spd_min / speed
            self._vel[1] *= self._spd_min / speed


        self._posn[0] += dt * self._vel[0]
        self._posn[1] += dt * self._vel[1]

        if self._walls == 0:
            if self._posn[0] >= self._wrapbnds[0]:
                self._posn[0] = 0
            elif self._posn[0] < 0:
                self._posn[0] = self._wrapbnds[0]
            if self._posn[1] >= self._wrapbnds[1]:
                self._posn[1] = 0
            elif self._posn[1] < 0:
                self._posn[1] = self._wrapbnds[1]

    def boid_angles(self, vel, diff):
        a = math.atan2(self._vel[1], self._vel[0])
        a1 = math.atan2(diff[1], diff[0])
        if a < 0:
            a += math.pi * 2
        if a1 < 0:
            a1 += math.pi * 2

        return np.abs(a - a1)

    def avoid_coll(self, indexes, boids):
        xTotal, yTotal = 0, 0
        for i in np.nditer(indexes):
            dif = [boids[i]._posn[0] - self._posn[0], boids[i]._posn[1] - self._posn[1]]

            if dif[0] ** 2 + dif[1] ** 2 < self._coll_range:
                xTotal -= boids[i]._posn[0] - self._posn[0]
                yTotal -= boids[i]._posn[1] - self._posn[1]

        return [xTotal, yTotal]

    def velMatching(self, num, indexes, boids):
        xTotal, yTotal = 0, 0
        for i in np.nditer(indexes):
            xTotal += boids[i]._vel[0]
            yTotal += boids[i]._vel[1]

        return [xTotal / num - self._vel[0], yTotal / num - self._vel[1]]

    def centring_flk(self, num, indexes, boids):
        xTotal, yTotal = 0, 0
        for i in np.nditer(indexes):
            xTotal += boids[i]._posn[0]
            yTotal += boids[i]._posn[1]

            self._boidMask[i] = 0

        return [xTotal / num - self._posn[0], yTotal / num - self._posn[1]]
