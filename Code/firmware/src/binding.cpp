#include <pybind11/pybind11.h>
#include "motion.h"

namespace py = pybind11;

PYBIND11_MODULE(motion, m) {
    m.doc() = "motion module";
    
    m.def("motion_init", &motion_init, "motion_init");
    m.def("motion_tick", &motion_tick, "motion_tick", py::arg("t"));
    m.def("motion_reset", &motion_reset, "motion_reset");
    m.def("motion_is_moving", &motion_is_moving, "motion_is_moving");
    m.def("motion_set_f", &motion_set_f, "motion_set_f", py::arg("f"));
    m.def("motion_set_h", &motion_set_h, "motion_set_h", py::arg("h"));
    m.def("motion_set_r", &motion_set_r, "motion_set_r", py::arg("r"));
    m.def("motion_set_x_speed", &motion_set_x_speed, "motion_set_x_speed", py::arg("x_speed"));
    m.def("motion_set_y_speed", &motion_set_y_speed, "motion_set_y_speed", py::arg("y_speed"));
    m.def("motion_set_turn_speed", &motion_set_turn_speed, "motion_set_turn_speed", py::arg("turn_speed"));
    m.def("motion_extra_x", &motion_extra_x, "motion_extra_x", py::arg("index"), py::arg("x"));
    m.def("motion_extra_y", &motion_extra_y, "motion_extra_y", py::arg("index"), py::arg("y"));
    m.def("motion_extra_z", &motion_extra_z, "motion_extra_z", py::arg("index"), py::arg("z"));
    m.def("motion_extra_angle", &motion_extra_angle, "motion_extra_angle", py::arg("index"), py::arg("motor"), py::arg("angle"));
    m.def("motion_get_f", &motion_get_f, "motion_get_f");
    m.def("motion_get_motor", &motion_get_motor, "motion_get_motor", py::arg("idx"));
    
    
}
