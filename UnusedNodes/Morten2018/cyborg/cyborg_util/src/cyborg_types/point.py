#!/usr/bin/env python

from collections import namedtuple
from scipy.spatial import distance


class Point(namedtuple('Point', ('x', 'y', 'z'))):
    @classmethod
    def from_pose(cls, pose):
        instance = cls(
                pose.position.x,
                pose.position.y,
                pose.position.z)
        return instance

    @classmethod
    def from_pose2d(cls, pose):
        instance = cls(pose.x, pose.y, 0)
        return instance

    def __init__(self, x, y, z):
        self.__x = x
        self.__y = y
        self.__z = z

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    def distance_to(self, other):
        return distance.euclidean(self, other)
