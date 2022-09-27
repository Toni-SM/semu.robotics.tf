import time
import yaml

import carb


def acquire_tf_listener_interface(use_tf2: bool = True):
    interface = TFListener(use_tf2=use_tf2)
    return interface

def release_tf_listener_interface(interface):
    interface.shutdown()


class TFListener:
    def __init__(self, node_name: str = "ros_tf_listener", use_tf2: bool = True) -> None:
        self._node_name = node_name
        self._use_tf2 = use_tf2
        
        self._listener = None

        if not self._init_ros_node():
            return

        # tf2 implementation
        if self._use_tf2:
            import rospy
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._listener = tf2_ros.TransformListener(self._tf_buffer)
            # internal methods
            self._time = rospy.Time
            self._lookup_transform = self._tf_buffer.lookup_transform
            self._all_frames_as_yaml = self._tf_buffer.all_frames_as_yaml
        # tf implementation
        else:
            import tf
            self._listener = tf.TransformListener()
            # internal methods
            self._time = self._listener.getLatestCommonTime
            self._lookup_transform = self._listener.lookupTransform
            self._all_frames_as_yaml = self._listener._buffer.all_frames_as_yaml

    def _init_ros_node(self) -> bool:
        """Initialize the ROS node
        """
        import rospy
        import rosgraph
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            carb.log_warn("ROS master is not running")
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            time.sleep(0.1)
            carb.log_info("ROS node started ({})".format(self._node_name))
        except rospy.ROSException as e:
            carb.log_error("ROS node ({}): {}".format(self._node_name, e))
            return False
        return True

    def shutdown(self):
        if self._listener:
            if self._use_tf2:
                self._listener.unregister()
            self._listener = None
        # # shutdown ROS node
        # import rospy
        # rospy.signal_shutdown("semu.robotics.tf")

    def get_transforms(self, root_frame: str = "world"):
        relations = []
        transforms = {}

        if self._listener:
            frames = yaml.load(self._all_frames_as_yaml(), Loader=yaml.SafeLoader)
            if frames is not None:
                for frame, info in frames.items():
                    relations.append((frame, info["parent"]))
                    try:
                        transform = self._lookup_transform(root_frame, frame, self._time() if self._use_tf2 else self._time(root_frame, frame))
                        if self._use_tf2:
                            translation = transform.transform.translation
                            rotation = transform.transform.rotation
                            transform = ([translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w])
                        transforms[frame] = transform
                    except:
                        pass
        
        return transforms, relations

    def reset(self):
        # remove "TF_OLD_DATA ignoring data from the past" warning
        if self._listener:
            carb.log_info("Reset TF listener (ROS)")
            if self._use_tf2:
                self._tf_buffer.clear()
            else:
                self._listener._buffer.clear()

    def is_ready(self):
        return self._listener != None
