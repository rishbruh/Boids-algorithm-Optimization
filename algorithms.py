from Boid_Behaviour import *
from random import randint
import random
from itertools import repeat
from sklearn.cluster import DBSCAN
import hdbscan


class World(object):

    def __init__(self, wd, ht, boids_number, conf_boids, clust, wd_range, clust_mark):
        random.seed(1)
        self.boids = self.boid_create(wd, ht, boids_number, conf_boids)
        self._boids_number = boids_number
        self._clrList = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], [0, 255, 255],
                            [255, 127, 0], [127, 255, 0], [127, 0, 255], [255, 0, 127], [0, 127, 255], [0, 255, 127],
                            [127, 0, 0], [0, 127, 0], [0, 0, 127], [127, 127, 0], [127, 0, 127], [0, 127, 127],
                            [127, 127, 127], [63, 127, 0], [63, 0, 127], [0, 63, 127], [127, 63, 0], [127, 0, 63],
                            [0, 127, 63], [190, 127, 0], [190, 0, 127], [0, 190, 127], [127, 190, 0], [127, 0, 190],
                            [0, 127, 190], [190, 63, 0], [190, 0, 63], [0, 190, 63], [63, 190, 0], [63, 0, 190],
                            [0, 63, 190], [63, 255, 0], [63, 0, 255], [0, 63, 255], [255, 63, 0], [255, 0, 63],
                            [0, 255, 63], [190, 255, 0], [190, 0, 255], [0, 190, 255], [255, 190, 0], [255, 0, 190],
                            [0, 255, 190], [64, 64, 64], [190, 190, 190], [127, 127, 127], [255, 0, 0], [0, 255, 0],
                            [0, 0, 255], [255, 255, 0], [255, 0, 255], [0, 255, 255], [255, 127, 0], [127, 255, 0],
                            [127, 0, 255], [255, 0, 127], [0, 127, 255], [0, 255, 127], [127, 0, 0], [0, 127, 0],
                            [0, 0, 127], [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255],
                            [0, 255, 255], [255, 127, 0], [127, 255, 0], [127, 0, 255], [255, 0, 127], [0, 127, 255],
                            [0, 255, 127], [127, 0, 0], [0, 127, 0], [0, 0, 127], [127, 127, 0], [127, 0, 127],
                            [0, 127, 127], [127, 127, 127], [63, 127, 0], [63, 0, 127], [0, 63, 127], [127, 63, 0],
                            [127, 0, 63], [0, 127, 63], [190, 127, 0], [190, 0, 127], [0, 190, 127], [127, 190, 0],
                            [127, 0, 190], [0, 127, 190], [190, 63, 0], [190, 0, 63], [0, 190, 63], [63, 190, 0],
                            [63, 0, 190], [0, 63, 190], [63, 255, 0], [63, 0, 255], [0, 63, 255], [255, 63, 0],
                            [255, 0, 63], [0, 255, 63], [190, 255, 0], [190, 0, 255], [0, 190, 255], [255, 190, 0],
                            [255, 0, 190], [0, 255, 190], [64, 64, 64], [190, 190, 190], [127, 127, 127], [255, 0, 0],
                            [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], [0, 255, 255], [255, 127, 0],
                            [127, 255, 0], [127, 0, 255], [255, 0, 127], [0, 127, 255], [0, 255, 127], [127, 0, 0],
                            [0, 127, 0], [0, 0, 127], [255, 255, 255]]
        self._clust_mark = clust_mark
        self.wd = wd
        self.conf_boids = conf_boids
        self.ht = ht
        self.count = 0
        self.updategrp = 1
        self.rangeclust = clust
        self.wd_range = wd_range
       

    def boidlocalupdate(self):

        if self.rangeclust == 0:
            i= 0
            while(i <len(self.boids)):
                k= i+1
                while(k<len(self.boids)-1):
                    dif = [self.boids[i]._posn[0] - self.boids[k]._posn[0],
                                  self.boids[i]._posn[1] - self.boids[k]._posn[1]]
                    if dif[0] ** 2 + dif[1] ** 2 < self.conf_boids[0]:
                        if self.boids[i].boid_angles(self.boids[i]._vel, dif) < self.boids[i]._angle_check:
                            self.boids[i]._boidMask[k] = 1
                        if self.boids[k].boid_angles(self.boids[k]._vel, dif) < self.boids[k]._angle_check:
                            self.boids[k]._boidMask[i] = 1
                    k= k+1
                i= i+1

        elif self.rangeclust == 1:

            # APPLYING DBSCAN
            X = self.loc(self.boids)
            db = DBSCAN(eps=self.wd_range, min_samples=2).fit(X)
            labels = db.labels_
            n_clusters_ = len(set(labels))

            grp = [[] for i in repeat(None, n_clusters_)]
            i=0
            while (i < len(self.boids)):
                if self._clust_mark == 1:
                    self.boids[i].setclr(self._clrList[labels[i]])
                if labels[i] != -1:
                    grp[labels[i]].append(i)
                i=i+1

            for i in range(n_clusters_):
                k=0
                while (k < len(grp[i]) - 1):
                    j = k+1
                    while (j < len(grp[i])):
                        dif = [self.boids[grp[i][k]]._posn[0] - self.boids[grp[i][j]]._posn[0],self.boids[grp[i][k]]._posn[1] - self.boids[grp[i][j]]._posn[1]]
                        if dif[0] ** 2 + dif[1] ** 2 < self.conf_boids[0]:
                            if self.boids[grp[i][k]].boid_angles(self.boids[grp[i][k]]._vel,dif) < self.boids[
                                grp[i][k]]._angle_check:
                                self.boids[grp[i][k]]._boidMask[grp[i][j]] = 1
                            if self.boids[grp[i][j]].boid_angles(self.boids[grp[i][j]]._vel,dif) < self.boids[
                                grp[i][j]]._angle_check:
                                self.boids[grp[i][j]]._boidMask[grp[i][k]] = 1
                        j=j+1
                    k=k+1



        elif self.rangeclust == 2:

            # APPLYING HDBSCAN
            X = self.loc(self.boids)
            hdb = hdbscan.HDBSCAN(min_cluster_size=20, min_samples=2,cluster_selection_epsilon=self.wd_range).fit(X)
            labels = hdb.labels_
            n_clusters_ = len(set(labels))
            grp = [[] for i in repeat(None, n_clusters_)]
            i = 0
            while (i < len(self.boids)):
                if self._clust_mark == 1:
                    self.boids[i].setclr(self._clrList[labels[i]])
                if labels[i] != -1:
                    grp[labels[i]].append(i)
                i = i + 1

            for i in range(n_clusters_):
                k = 0
                while (k < len(grp[i]) - 1):
                    j = k + 1
                    while (j < len(grp[i])):
                        dif = [self.boids[grp[i][k]]._posn[0] - self.boids[grp[i][j]]._posn[0],
                               self.boids[grp[i][k]]._posn[1] - self.boids[grp[i][j]]._posn[1]]
                        if dif[0] ** 2 + dif[1] ** 2 < self.conf_boids[0]:
                            if self.boids[grp[i][k]].boid_angles(self.boids[grp[i][k]]._vel, dif) < self.boids[
                                grp[i][k]]._angle_check:
                                self.boids[grp[i][k]]._boidMask[grp[i][j]] = 1
                            if self.boids[grp[i][j]].boid_angles(self.boids[grp[i][j]]._vel, dif) < self.boids[
                                grp[i][j]]._angle_check:
                                self.boids[grp[i][j]]._boidMask[grp[i][k]] = 1
                        j = j + 1
                    k = k + 1

    def posn_update_boids(self, dt):
        i=0
        while (i < len(self.boids)):
            self.boids[i].posn_update(dt, self.boids)
            i=i+1

    def boid_create(self, wd, ht, num, conf_boids):
        boids = []
        i = 0
        while(i in range((num) -1)):
            x = randint(conf_boids[6], conf_boids[7])
            if randint(0, 1) == 1:
                x = -x
            y = randint(conf_boids[6], conf_boids[7])
            if randint(0, 1) == 1:
                y = -y
            boids.append(Boid([wd / 2, ht / 2], [wd, ht], [x, y], [255, 255, 255], num, conf_boids))
            i=i+1
        return boids

    def vert_batch(self):
        batch = []
        i = 0
        while (i < len(self.boids)):
            batch.append(self.boids[i].vert_list())
            i = i + 1
        return batch

    def clr_batch(self):
        batch = []
        i = 0
        while (i < len(self.boids)):
            batch.append(self.boids[i]._clr)
            i = i + 1
        return batch

    def loc(self, boids):
        batch = []
        i = 0
        while (i < len(self.boids)):
            batch.append(self.boids[i]._posn)
            i = i + 1
        return batch