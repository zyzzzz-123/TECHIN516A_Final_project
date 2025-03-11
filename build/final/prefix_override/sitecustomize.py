import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yuzhez23@netid.washington.edu/ros2_ws/src/final/install/final'
