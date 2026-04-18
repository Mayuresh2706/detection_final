import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mayuresh/turtlebot3_ws/src/aruco_detection/install/aruco_detection'
