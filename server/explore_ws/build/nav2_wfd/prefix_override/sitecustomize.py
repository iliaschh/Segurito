import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/neverdiedooms/explore_ws/install/nav2_wfd'
