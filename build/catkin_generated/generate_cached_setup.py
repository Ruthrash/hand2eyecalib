# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ruthz/workspaces/devel_isolated/rqt_easy_handeye;/home/ruthz/workspaces/devel_isolated/handeye;/home/ruthz/workspaces/devel_isolated/fiducials;/home/ruthz/workspaces/devel_isolated/fiducial_slam;/home/ruthz/workspaces/devel_isolated/aruco_detect;/home/ruthz/workspaces/devel_isolated/fiducial_msgs;/home/ruthz/workspaces/devel_isolated/easy_handeye_msgs;/home/ruthz/workspaces/devel_isolated/easy_handeye;/home/ruthz/utm/base_ws/devel;/home/ruthz/utm/git/frankapy/catkin_ws/devel;/home/ruthz/utm/intel_ws/devel_isolated/realsense2_description;/home/ruthz/utm/intel_ws/devel_isolated/realsense2_camera;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ruthz/utm/git/calibration/hand2eyecalib/build/devel/env.sh')

output_filename = '/home/ruthz/utm/git/calibration/hand2eyecalib/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
