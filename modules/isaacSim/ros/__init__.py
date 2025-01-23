try:
    import rospy
    from . import ros1_dict as ros_dict 
    from . import ros1_numpy as ros_numpy
    
    ROS_INITIALIZED = True
    def initialize_ros():
        pass
    
except ImportError:
    import rclpy
    from . import ros2_dict as ros_dict 
    from . import ros2_numpy as ros_numpy
    
    ROS_INITIALIZED = False

    def initialize_ros():
        global ROS_INITIALIZED
        if not ROS_INITIALIZED:
            rclpy.init()
            ROS_INITIALIZED = True
