
#include <pybind11/pybind11.h>

#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "ros_tf_listener.cpp"

namespace py = pybind11;


namespace pybind11 { namespace detail {
    template <> struct type_caster<Frame>{
        public:
            PYBIND11_TYPE_CASTER(Frame, _("Frame"));

            // conversion from C++ to Python
            static handle cast(Frame src, return_value_policy /* policy */, handle /* parent */){
                
                PyObject * translation = PyDict_New();
                PyDict_SetItemString(translation, "x", PyFloat_FromDouble(src.translation.x));
                PyDict_SetItemString(translation, "y", PyFloat_FromDouble(src.translation.y));
                PyDict_SetItemString(translation, "z", PyFloat_FromDouble(src.translation.z));

                PyObject * rotation = PyDict_New();
                PyDict_SetItemString(rotation, "x", PyFloat_FromDouble(src.rotation.x));
                PyDict_SetItemString(rotation, "y", PyFloat_FromDouble(src.rotation.y));
                PyDict_SetItemString(rotation, "z", PyFloat_FromDouble(src.rotation.z));
                PyDict_SetItemString(rotation, "w", PyFloat_FromDouble(src.rotation.w));

                PyObject * frame = PyDict_New();
                PyDict_SetItemString(frame, "name", PyUnicode_FromString(src.name.c_str()));
                PyDict_SetItemString(frame, "parentName", PyUnicode_FromString(src.parentName.c_str()));
                PyDict_SetItemString(frame, "translation", translation);
                PyDict_SetItemString(frame, "rotation", rotation);

                return frame;
            }
    };
}}


PYBIND11_MODULE(ros_tf_listener_p, m){
    py::class_<RosTfListener>(m, "RosTfListener")
        .def(py::init<>())
        // methods
        .def("start", &RosTfListener::start)
        .def("stop", &RosTfListener::stop)
        .def("reset_buffer", &RosTfListener::resetBuffer)
        .def("get_frames", &RosTfListener::getFrames)
        .def("get_transform", &RosTfListener::getTransform)
        .def("set_root_frame_name", &RosTfListener::setRootFrameName);
}
