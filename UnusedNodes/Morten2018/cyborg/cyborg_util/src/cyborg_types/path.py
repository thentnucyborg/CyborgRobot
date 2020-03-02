#!/usr/bin/env python

import copy
import itertools
from collections import deque
from pose import Pose


class Path():
    @classmethod
    def from_posearray(cls, array, maxlen=None):
        steps = [Pose.from_pose(elem) for elem in reversed(array)]
        instance = cls(steps, maxlen)
        return instance

    def __init__(self, steps=None, maxlen=None):
        # super(Path, self).__init__(poses)
        self.__steps = deque(steps, maxlen)
        self.__cached_length = None

    def __delitem__(self, key):
        try:
            return self.__steps.__delitem__(key)
        except Exception:
            raise

    def __getitem__(self, key):
        try:
            return self.__steps.__getitem__(key)
        except Exception:
            raise

    def __iter__(self):
        try:
            return self.__steps.__iter__()
        except Exception:
            raise

    def __missing__(self, key):
        try:
            return self.__steps.__missing__(key)
        except Exception:
            raise

    def __setitem__(self, key, value):
        try:
            return self.__steps.__setitem__(key, value)
        except Exception:
            raise

    def __reversed__(self):
        try:
            return self.__steps.__reversed__()
        except Exception:
            raise

    # Sum the length of all coordinate pair distances
    def _calculate_length(self):
        if not self.__steps:
            return 0

        a, b = itertools.tee(self.__steps)
        next(b, None)
        pairs = itertools.izip(a, b)

        return sum(map(
            lambda p: p[0].position.distance_to(p[1].position), pairs))

    # Set new length. New length will be greater or equal to zero
    def _update_length(self, change):
        # Use self.length here, as __cached_length is set on the first access
        self.__cached_length = max(0, self.length + change)

    @property
    def length(self):
        if self.__cached_length is None:
            self.__cached_length = self._calculate_length()
        return self.__cached_length

    # Add a position
    def append(self, pos):
        # Update length
        try:
            next_pos = next(reversed(self.__steps))
            diff = pos.distance_to(next_pos)
        except Exception:
            diff = 0

        self._update_length(diff)
        self.__steps.append(pos)

    # Get a the next position
    def pop(self):
        try:
            pos = self.__steps.pop()
        except Exception:
            raise

        # Update length
        try:
            next_pos = next(reversed(self.__steps))
            diff = -pos.distance_to(next_pos)
        except Exception:
            diff = 0

        # Use self.length here, as it is lazy instantiated
        self._update_length(diff)

        return pos

    # Advance our position along the path to the given position
    def advance_to(self, position):
        new = copy.copy(self)
        for elem in self:
            new.pop()
            if position.distance_to(elem) < 0.5:
                return new
        return self
