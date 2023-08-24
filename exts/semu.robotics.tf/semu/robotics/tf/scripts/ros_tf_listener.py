import os
import sys
import math

import carb


def acquire_tf_listener_interface(extension_path):
    return TfListener(extension_path)

def release_tf_listener_interface(interface):
    interface.shutdown()


class TfListener:
    def __init__(self, extension_path: str = "") -> None:
        self._listener = None
        self._root_frame_name = ""

        sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)
        sys.path.append(os.path.join(extension_path, "bin"))

        # load shared library
        current_dir= os.getcwd()
        os.chdir(extension_path)
        import ros_tf_listener_p
        os.chdir(current_dir)

        self._listener = ros_tf_listener_p.RosTfListener()

    def shutdown(self):
        self._listener = None

    def get_transforms(self, root_frame_name: str = "world"):
        # set root frame
        if self._root_frame_name != root_frame_name:
            self._root_frame_name = root_frame_name
            self._listener.set_root_frame_name(self._root_frame_name)

        relations = []
        transforms = {}
        for frame in self._listener.get_frames():
            relations.append((frame["name"], frame["parentName"]))
            if not math.isnan(frame["translation"]["x"]):
                transforms[frame["name"]] = ([frame["translation"][a] for a in ["x", "y", "z"]],
                                             [frame["rotation"][a] for a in ["x", "y", "z", "w"]])

        return transforms, relations

    def get_transform(self, target_frame, source_frame):
        return (), "TODO"
        # try:
        #     transform = self._lookup_transform(target_frame, source_frame, self._time() if self._use_tf2 else self._time(target_frame, source_frame))
        # except Exception as e:
        #     return (), type(e).__name__
        # if self._use_tf2:
        #     translation = transform.transform.translation
        #     rotation = transform.transform.rotation
        #     transform = ([translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w])
        # return transform, ""

    def reset(self):
        """Clear all tf2_ros::Buffer data

        Clearing the data stop the warning message "TF_OLD_DATA ignoring data from the past"
        """
        if self._listener:
            carb.log_info("Clear all tf2_ros::Buffer data")
            self._listener.reset()

    def is_ready(self):
        return self._listener != None
