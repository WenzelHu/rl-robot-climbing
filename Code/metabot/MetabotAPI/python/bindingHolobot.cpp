#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Holobot.h"

using namespace Metabot;
namespace py = pybind11;

PYBIND11_PLUGIN(holobot) {
    py::module m("holobot", "Holobot robot");
    
    py::class_<Robot>(m, "Robot")
        .def("monitor", &Holobot::monitor)
        ;

    py::class_<Holobot, Robot>(m, "Holobot")
        .def(py::init<std::string, int>())
        .def(py::init<std::string>())

        .def("waitUpdate", &Holobot::waitUpdate)
        .def("set_board_led", &Holobot::set_board_led)

        .def("rhock_mode", &Holobot::rhock_mode)
        .def("control", &Holobot::control)
        .def("move_toward", &Holobot::move_toward)
        .def("turn", &Holobot::turn)
        .def("stop_all", &Holobot::stop_all)
        .def("beep", &Holobot::beep)
        .def("calibrate_magneto", &Holobot::calibrate_magneto)
        .def("play", &Holobot::play)
      
        .def("get_time", &Holobot::get_time)
        .def("get_dist", &Holobot::get_dist)
        .def("get_opt", &Holobot::get_opt)
        .def("get_opts", &Holobot::get_opts)
        .def("get_yaw", &Holobot::get_yaw)
        .def("get_wheel_speeds", &Holobot::get_wheel_speeds)
        .def("reset_yaw", &Holobot::reset_yaw)
        .def("print_state", &Holobot::print_state)
        .def("debug_state", &Holobot::debug_state)
        .def("set_leds", &Holobot::set_leds)
        .def("leds_breath", &Holobot::leds_breath)
        .def_readonly("current_time", &Holobot::current_time)
        ;

    return m.ptr();
}
