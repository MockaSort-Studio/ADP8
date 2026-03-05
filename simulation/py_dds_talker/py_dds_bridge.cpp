
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>

class PyDDSBridge {
 public:
  void Init() { std::cout << "Hello!" << std::endl; }
};

PYBIND11_MODULE(py_dds_bridge, module) {
  module.doc() = "The pybind11 extension of Javelina";
  pybind11::class_<PyDDSBridge>(module, "PyDDSBridge")
      .def(pybind11::init<>())
      .def("init", &PyDDSBridge::Init);
}
