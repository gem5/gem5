"""Optional directed test objects; build with TPC_TESTS=1."""
from m5.SimObject import SimObject, PyBindMethod
from m5.params import *
from m5.objects.Framework import TPCComponent
class TPCStateFixture(SimObject):
    type = "TPCStateFixture"
    cxx_class = "gem5::prefetch::StateFixture"
    cxx_header = "mem/cache/prefetch/state_fixture.hh"
    cxx_exports = [PyBindMethod("run")]
    cache = Param.BaseCache("I-cache under test")
    controller = Param.TPCController("Bound controller")
    component = Param.TPCComponent("Component under test")
class TPCFixtureComponent(TPCComponent):
    type = "TPCFixtureComponent"
    cxx_class = "gem5::prefetch::FixtureComponent"
    cxx_header = "mem/cache/prefetch/state_fixture.hh"
    recognize = Param.Bool(True, "Recognize without generating candidates")
