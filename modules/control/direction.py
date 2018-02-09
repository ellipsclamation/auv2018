import rospy
from std_msgs.msg import Int32
from time import sleep


class Direction():
    """Controls thrusters to point AUV to a certain direction given rotation and z values"""

    def __init__(self, rotation=0, z=0):
        # rotation, negative = left, positive = right, 0 = no rotation
        self.rotation = rotation
        # vertical movement, negative = down, positive = up, 0 = no vertical movement
        self.z = z

    def set_direction(self, rotation=0, z=0):
        self.rotation = rotation
        self.z = z

    def move_direction(self):
        """Points AUV to direction with given rotation and z values"""

        pub_rotation = rospy.Publisher('rotation_direction', Int32, queue_size=10)

        pub_vertical = rospy.Publisher('vertical_direction', Int32, queue_size=10)

        sleep(.5)
        pub_rotation.publish(self.rotation)
        pub_vertical.publish(self.z)
        print('moving AUV to rotation=%d, z=%d' % (self.rotation, self.z))
