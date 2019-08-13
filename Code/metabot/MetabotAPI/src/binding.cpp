#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "MetabotV2.h"

using namespace Metabot;
namespace py = pybind11;

PYBIND11_PLUGIN(metabot) {
    py::module m("metabot", "Metabot robot");
    
    py::class_<Robot>(m, "Robot")
        .def("monitor", &MetabotV2::monitor)
        ;

    py::class_<MetabotV2, Robot>(m, "MetabotV2")
        .def(py::init<std::string, int>())
        .def(py::init<std::string>())

        .def("waitUpdate", &MetabotV2::waitUpdate)
        .def("start", &MetabotV2::start)
        .def("stop", &MetabotV2::stop)
        .def("reset", &MetabotV2::reset)
        .def("beep", &MetabotV2::beep)
        .def("control", &MetabotV2::control)
        .def("setLeds", (void (MetabotV2::*)(uint8_t)) &MetabotV2::setLeds)
        .def("setLeds", (void (MetabotV2::*)(std::string)) &MetabotV2::setLeds)
        .def("setMotors", &MetabotV2::setMotors)

        .def_readwrite("motors", &MetabotV2::motors)
        .def_readonly("voltage", &MetabotV2::voltage)
        .def_readonly("yaw", &MetabotV2::yaw)
        .def_readonly("pitch", &MetabotV2::pitch)
        .def_readonly("roll", &MetabotV2::roll)
        .def_readonly("distance", &MetabotV2::distance)
        .def_readwrite("leds", &MetabotV2::leds)
        ;

    return m.ptr();
}
