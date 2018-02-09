import rospy

from std_msgs.msg import Int32
from test import test_movement
from modules.control.motor import Motor
from modules.control.direction import Direction


class AUV():
    """AUV Master, automates tasks"""

    def __init__(self, state=0, tasks=['gate', 'dice', 'slots']):
        self.state = state
        self.tasks = tasks

        # self.test
        self.motor = Motor()  # initialize Motor() class
        self.direction = Direction()  # initialize Direction() class
        # TODO construct modules, refactor robosub.py

    def toggle_state(self, arg=None):
        """Toggles the state of the AUV (1 == on, 0 == off, empty == toggle)"""

        # Toggles the state if there is no argument passed
        if arg is None:
            if self.state == 0:
                self.state = 1
            else:
                self.state = 0

        print('state:%d' % self.state)
        print(self.tasks)
