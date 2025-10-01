import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/edson/ros2_ws/src/p3at_simulation/install/p3at_simulation'
