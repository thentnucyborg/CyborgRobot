import rospy
from cyborg_msgs.srv import AvailableGoals
from cyborg_types import Point, Quaternion, Pose


class Locations():
    def __init__(self):
        # Available goals service proxy
        srv_name = '/cyborg/nav/get_available_goals'
        rospy.wait_for_service(srv_name)
        try:
            self._goals_svc = rospy.ServiceProxy(srv_name, AvailableGoals)
        except rospy.ServiceException as e:
            print("Service call to %s failed: %s" % (srv_name, e))

    def __iter__(self):
        return self

    def __getitem__(self, key):
        if not hasattr(self, '_cache'):
            self.create_cache()
        return self._cache[key]

    def create_cache(self):
        self._cache = dict()
        for goal in self._goals_svc().goals:
            position = Point.from_pose2d(goal.position)
            orientation = Quaternion(0, 0, 0, 1)
            pose = Pose(position, orientation)

            self._cache[goal.name] = pose

    # Get all named locations
    def next(self):
        if not hasattr(self, '_iter'):
            if not hasattr(self, '_cache'):
                self.create_cache()
            self._iter = iter(self._cache)
        return next(self._iter)
