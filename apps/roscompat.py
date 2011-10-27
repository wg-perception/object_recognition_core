try:
    import rospkg
    import os
    if rospkg.on_ros_path(os.path.dirname(__file__)):
        import roslib; roslib.load_manifest('object_recognition')
except ImportError:
    pass
except Exception as e :
    pass