import os
import stat
from subprocess import call


def change_permissions(script):
    """Change the permission of the script file to user executable"""
    st = os.stat(script)
    os.chmod(script, st.st_mode | stat.S_IEXEC)


def run_script(script):
    """Run the script"""
    call(['sh', script])


def install():
    """Installs scripts for first time setup"""
    scripts = ['scripts/1setup_ros.sh', 'scripts/2ros_environment.sh']

    for script in scripts:
        change_permissions(script)
        run_script(script)
