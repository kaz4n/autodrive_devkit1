import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/t31/test/ws_Comp/autodrive_devkit1/install/roboracer_autonomy'
