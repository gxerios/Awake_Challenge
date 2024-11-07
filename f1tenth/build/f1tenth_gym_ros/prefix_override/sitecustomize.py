import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gelonch/Documents/ROS2-WRK/awake/f1tenth/install/f1tenth_gym_ros'
