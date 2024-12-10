import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kulka136/av/ros_ws/src/av_project_19/install/lane_assist_package'
