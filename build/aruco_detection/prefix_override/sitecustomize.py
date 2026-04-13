import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yammmyu/turtlebot3_ws/src/detection_final/install/aruco_detection'
