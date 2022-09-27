import yaml
import threading

import carb


def acquire_tf_listener_interface():
    interface = TFListener()
    return interface

def release_tf_listener_interface(interface):
    interface.shutdown()


class TFListener:
    def __init__(self, node_name: str = "ros2_tf_listener") -> None:        
        import rclpy
        import tf2_ros

        self._node = None
        self._listener = None

        rclpy.init()
        self._node = rclpy.node.Node(node_name)

        # tf2 implementation
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._time = rclpy.time.Time
        self._lookup_transform = self._tf_buffer.lookup_transform
        self._all_frames_as_yaml = self._tf_buffer.all_frames_as_yaml

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin).start()

    def shutdown(self):
        import rclpy
        if self._listener:
            self._listener.unregister()
            self._listener = None
        if self._executor:
            self._executor.shutdown()
            self._executor = None
        if self._node:
            self._node.destroy_node()
            self._node = None
        rclpy.shutdown()

    def get_transforms(self, root_frame: str = "world"):
        relations = []
        transforms = {}

        if self._listener:
            frames = yaml.load(self._all_frames_as_yaml(), Loader=yaml.SafeLoader)
            if type(frames) is dict:
                for frame, info in frames.items():
                    relations.append((frame, info["parent"]))
                    try:
                        transform = self._lookup_transform(root_frame, frame, self._time())
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        transform = ([translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w])
                        transforms[frame] = transform
                    except:
                        pass

        return transforms, relations

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self._lookup_transform(target_frame, source_frame, self._time())
        except Exception as e:
            return (), type(e).__name__
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform = ([translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w])
        return transform, ""

    def reset(self):
        # remove "TF_OLD_DATA ignoring data from the past" warning
        if self._listener:
            carb.log_info("Reset TF listener (ROS2)")
            self._tf_buffer.clear()

    def is_ready(self):
        return self._listener != None
