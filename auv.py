"""Import line-oriented command interpreter"""
import cmd

from scripts import setup_ros
from modules import example


class AUV(cmd.Cmd):
    """AUV command interpreter"""
    intro = '\nType help or ? to list commands.'
    prompt = 'auv> '

    def do_test(self, arg):
        '\nthis is documentation'

        print('test')
        print(arg)

    def do_example(self, arg):
        '\nCalls the example module\'s print_example function'

        example.print_example(arg)

    def do_setup_ros(self, arg):
        '\nDo first time setup for ROS. Installs ROS lunar for Ubuntu 17.04. (y/n)'

        response = 'n'

        if arg.lower() != 'y' and arg.lower() != 'n':
            response = raw_input(
                '\nAre you sure you want to do first time setup for ROS? (y/n): '
            ).lower()

        if arg == 'y' or response == 'y':
            setup_ros.install()

    def do_exit(self, arg):
        '\nExits auv'

        return True


if __name__ == '__main__':
    AUV().cmdloop()
