import sys
if sys.prefix == '/home/acroci/repos/ros2_ws_pixi/.pixi/envs/default':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/acroci/repos/ros2_ws_pixi/install/my_package'
