import rospy
from std_msgs.msg import Int32
from time import sleep


class Motor():
    """Controls motors"""

    def __init__(self, state=0):
        self.state = state

        def callback(data):
            self.state = data.data

        rospy.Subscriber('motor_state', Int32, callback)

    def get_state(self):
        return self.state

    def toggle_state(self, arg=None):
        """Toggles the state of the motors (1 == on, 0 == off, empty == toggle)"""

        pub = rospy.Publisher('motor_state', Int32, queue_size=10)

        # Toggles the state if there is no argument passed
        if arg is None:
            if self.state == 0:
                arg = 1
            else:
                arg = 0

        sleep(.5)
        pub.publish(arg)
        print('\nmotor state set to %d' % arg)
