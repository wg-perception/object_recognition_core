from object_recognition_core.io.sink import *
from object_recognition_core.io.source import *

try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

if ECTO_ROS_FOUND:
    from object_recognition_core.io.ros import *

from object_recognition_core.io.voter import *
