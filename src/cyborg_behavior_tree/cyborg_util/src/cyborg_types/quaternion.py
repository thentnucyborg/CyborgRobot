#!/usr/bin/env python

from collections import namedtuple
from tf.transformations import quaternion_from_euler


class Quaternion(namedtuple('Quaternion', ('x', 'y', 'z', 'w'))):
    @classmethod
    def from_pose(cls, pose):
        instance = cls(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        return instance

    @classmethod
    def from_pose2d(cls, pose):
        q = quaternion_from_euler(0, 0, pose.theta)
        instance = cls(*q)
        return instance

    @classmethod
    def from_pose_stamped(cls, pose_stamped):
        return cls.from_pose(pose_stamped.pose)

    def __init__(self, x, y, z, w):
        self.__x = x
        self.__y = y
        self.__z = z
        self.__w = w

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    @property
    def w(self):
        return self.__w
