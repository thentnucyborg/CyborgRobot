#!/usr/bin/env python
import b3

__all__ = ['BlackboardCompare']

class BlackboardCompare(b3.Decorator):
    def __init__(self, child, key=None, value=None, equal=True):
        super(Blackboard, self).__init__(child)

        self.key = key
        self.value = value

    def open(self, tick):
        pass

    def tick(self, tick):
        try:
            compare_val = tick.blackboard.get(key, tick.tree.id, None)
            should_execute = (compare_val == self.value) if equal
                else (compare_val != self.value)

            return self.child._execute(tick) if should_execute else b3.FAILURE
        else:
            return b3.FAILURE
