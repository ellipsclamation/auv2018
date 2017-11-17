"""Import line-oriented command interpreter"""
import cmd
import rospy
import subprocess
import time
import os

from scripts import setup_ros
from modules.control.motor import Motor


class AUV(cmd.Cmd):
    """AUV command interpreter"""

    intro = '\nType help or ? to list commands.'
    prompt = 'auv> '

    # ros_setup
    def do_setup_ros(self, arg):
        '\nDo first time setup for ROS. Installs ROS lunar for Ubuntu 17.04. [y/n]'

        response = 'n'

        if arg.lower() != 'y' and arg.lower() != 'n':
            response = raw_input(
                '\nAre you sure you want to do first time setup for ROS? [y/n]: '
            ).lower()

        if arg == 'y' or response == 'y':
            setup_ros.install()

    def complete_setup_ros(self, text, line, start_index, end_index):
        args = ['y', 'n']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # motor
    def do_motor(self, arg):
        '\nTurn on or off motors [on/off] or [1/0]\
         \n[toggle] to toggle the current state\
         \n[state] or no argument to print current state'

        if arg.lower() == 'on' or arg == '1':
            motor.toggle_state(1)
        elif arg.lower() == 'off' or arg == '0':
            motor.toggle_state(0)
        elif arg.lower() == 'toggle':
            motor.toggle_state()
        else:
            print('\nmotor state: %d' % motor.get_state())

    def complete_motor(self, text, line, start_index, end_index):
        args = ['on', 'off', 'toggle', 'state']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # exit
    def do_exit(self, arg):
        '\nExits auv'

        return True


def parse(arg):
    'Convert a series of zero or more numbers to an argument tuple'
    return tuple(map(int, arg.split()))


if __name__ == '__main__':
    # open roscore in subprocess
    print('Setting up roscore.')
    roscore = subprocess.Popen('roscore')
    time.sleep(1)

    rospy.init_node('AUV', anonymous=True)
    motor = Motor()  # initialize Motor() class

    AUV().cmdloop()  # run AUV command interpreter

    # close roscore and rosmaster on exit
    subprocess.Popen.kill(roscore)
    os.system('killall -9 rosmaster')
    os.system('killall -9 rosout')
