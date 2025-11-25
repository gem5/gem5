/**
 * Python bindings for WayGuardTable runtime control
 * Exposes a small subset of WayGuardTable methods to Python so configs
 * can set/get masks at runtime without modifying generated param bindings.
 */

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "mem/cache/cache_blk.hh"
#include "mem/cache/dawg.hh"
#include "mem/cache/tags/base.hh"
#include "sim/init.hh"

namespace py = pybind11;

namespace gem5
{

static void
pybind_wayguard(py::module_ &m_internal)
{
    py::module_ m = m_internal.def_submodule("wayguard");

    cprintf("PYBIND-WG: embedded WayGuardTable_py module initialized\n");

    // expose small helper functions instead of re-binding the C++ class
    m.def("set_mask",
          [](gem5::SimObject *so, uint32_t set, uint32_t domain, uint32_t mask) {
              auto *w = dynamic_cast<gem5::WayGuardTable*>(so);
              if (!w)
                  throw std::runtime_error("set_mask: provided SimObject is not a WayGuardTable");
              w->setMask(set, domain, mask);
          },
          py::arg("w"), py::arg("set"), py::arg("domain"), py::arg("mask"));

    m.def("get_mask",
          [](gem5::SimObject *so, uint32_t set, uint32_t domain) {
              auto *w = dynamic_cast<gem5::WayGuardTable*>(so);
              if (!w)
                  throw std::runtime_error("get_mask: provided SimObject is not a WayGuardTable");
              return w->getMask(set, domain);
          },
          py::arg("w"), py::arg("set"), py::arg("domain"));

    m.def("set_all_masks",
          [](gem5::SimObject *so, uint32_t domain, uint32_t mask) {
              auto *w = dynamic_cast<gem5::WayGuardTable*>(so);
              if (!w)
                  throw std::runtime_error("set_all_masks: provided SimObject is not a WayGuardTable");
              w->setAllMasks(domain, mask);
          },
          py::arg("w"), py::arg("domain"), py::arg("mask"));

    m.def("get_domains",
          [](gem5::SimObject *so) {
              auto *w = dynamic_cast<gem5::WayGuardTable*>(so);
              if (!w)
                  throw std::runtime_error("get_domains: provided SimObject is not a WayGuardTable");
              return w->getDomains();
          },
          py::arg("w"));
}

static EmbeddedPyBind embed_obj("WayGuardTable_py", pybind_wayguard, "SimObject");

} // namespace gem5
