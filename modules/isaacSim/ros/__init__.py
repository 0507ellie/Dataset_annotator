import os

# Determine ROS version from environment variable
ROS_VERSION = os.getenv("ROS_DISTRO")

if ROS_VERSION and ROS_VERSION in ["foxy", "galactic", "humble", "iron", "jazzy"]:
    # ROS 2 detected
    import rclpy
    from . import ros2_dict as ros_dict
    from . import ros2_numpy as ros_numpy

    ROS_INITIALIZED = False

    def initialize_ros():
        """Initialize ROS 2 if not already initialized."""
        global ROS_INITIALIZED
        if not ROS_INITIALIZED:
            rclpy.init()
            ROS_INITIALIZED = True

    print(f"Detected ROS 2: {ROS_VERSION}")

else:
    # Assume ROS 1 if ROS 2 is not detected
    try:
        import rospy
        from . import ros1_dict as ros_dict
        from . import ros1_numpy as ros_numpy

        ROS_INITIALIZED = True

        def initialize_ros():
            """ROS 1 initializes automatically, no manual initialization required."""
            pass

        print("Detected ROS 1.")

    except ImportError:
        print("Error: Could not detect ROS 1 or ROS 2. Ensure ROS is installed and sourced properly.")
        ROS_INITIALIZED = False
