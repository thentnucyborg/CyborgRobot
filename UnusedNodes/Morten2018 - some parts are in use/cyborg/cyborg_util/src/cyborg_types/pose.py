#!/usr/bin/env python

from collections import namedtuple
from point import Point
from quaternion import Quaternion
from scipy.spatial import distance


class Pose(namedtuple('Pose', ('position', 'orientation'))):
    @classmethod
    def from_pose(cls, pose):
        position = Point.from_pose(pose)
        orientation = Quaternion.from_pose(pose)
        instance = cls(position, orientation)
        return instance

    @classmethod
    def from_pose2d(cls, pose):
        position = Point.from_pose2d(pose)
        orientation = Quaternion.from_pose2d(pose)
        instance = cls(position, orientation)
        return instance

    @classmethod
    def from_pose_stamped(cls, pose_stamped):
        return cls.from_pose(pose_stamped.pose)

    def __init__(self, position, orientation):
        self.__position = position
        self.__orientation = orientation

    @property
    def position(self):
        return self.__position

    @property
    def orientation(self):
        return self.__orientation

    # Euclidean distance to another pose
    def distance_to(self, other):
        return distance.euclidean(self.position, other.position)
