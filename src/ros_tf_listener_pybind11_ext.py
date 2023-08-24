import os
import sys
import glob
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

# kit python
if sys.platform == "win32":
    raise NotImplementedError
elif sys.platform == "linux":
    python_library_dir = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "..", "lib"))
    isaac_sim_dir = "/isaac-sim" 
    if not os.path.exists("/isaac-sim"):
        isaac_sim_dir = glob.glob(os.path.join(os.path.expanduser("~"), ".local/share/ov/pkg/isaac_sim-*"))
        isaac_sim_dir = os.path.abspath(sorted(isaac_sim_dir)[-1])  # latest version
    ros_distro_dir = os.path.join(isaac_sim_dir, "exts", "omni.isaac.ros_bridge", "noetic")

if not os.path.exists(python_library_dir):
    raise Exception(f"Kit Python library directory not found: {python_library_dir}")
if not os.path.exists(ros_distro_dir):
    raise Exception(f"ROS distro directory not found: {ros_distro_dir}")

ext_modules = [
    Pybind11Extension(
        name="ros_tf_listener_p",
        sources=["ros_tf_listener_pybind11_wrapper.cpp"],
        include_dirs=[os.path.join(ros_distro_dir, "include")],
        library_dirs=[os.path.join(ros_distro_dir, "lib"), python_library_dir],
        libraries=["roscpp", "roscpp_serialization", "rosconsole", "rostime", "tf2", "tf2_ros"],
        # extra_link_args=["-Wl,-rpath=./bin"],
        define_macros=[("_GLIBCXX_USE_CXX11_ABI", "0"),
                       ("ROSCONSOLE_BACKEND_LOG4CXX",),
                       ("ROS_BUILD_SHARED_LIBS", "1")],
        undef_macros=["CTYPES", "EXECUTABLE"],
    ),
]

setup(name="ros-tf-listener",
      cmdclass={"build_ext": build_ext},
      ext_modules=ext_modules)
