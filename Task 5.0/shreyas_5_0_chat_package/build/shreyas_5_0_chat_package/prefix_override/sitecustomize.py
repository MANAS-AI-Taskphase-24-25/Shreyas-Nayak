import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shreyas/Desktop/Prograz/Ros2_WrkSpace/src/shreyas_5_0_chat_package/install/shreyas_5_0_chat_package'
