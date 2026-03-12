
#include <pybind11/pybind11.h>
#include "sim/init.hh"

PYBIND11_MODULE(_m5, m) {
    gem5::EmbeddedPyBind::initAll(m);
}
