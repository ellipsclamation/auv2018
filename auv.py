"""Import line-oriented command interpreter"""
import cmd
import subprocess
import time
import os

"""If ROS is not detected, installs ROS lunar for Ubuntu 17.04."""
try:
    import rospy
    from modules.control.motor import Motor
except ImportError:
    import sys
    from scripts import setup_ros

    print('No ROS detected')
    response = raw_input(
                '\nAre you sure you want to do first time setup for ROS? [y/n]: '
            ).lower()
    if response == 'y':
        print('Setting up ROS lunar for Ubuntu 17.04')
        setup_ros.install()

    sys.exit()


class AUV(cmd.Cmd):
    """AUV command interpreter"""

    intro = '\nType help or ? to list commands.'
    prompt = 'auv> '


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
