from m5.objects.SimObject import SimObject
from m5.params import *


class HelloSimObject(SimObject):
    type = "HelloSimObject"  # Same as the name of the simobject
    cxx_header = (
        "bootcamp/hello-sim-object/hello_sim_object.hh"  # w.r.t. gem5/src/
    )
    cxx_class = "gem5::HelloSimObject"

    # The below statement cna only be parsed by gem5's python
    # add a param of Int class type (defined in m5/params.py)\
    # with av vlaue and description.
    num_hellos = Param.Int(7, "Number of times to say hello")

    # Param.GoodByeSimObject will be created during gem5 \
    # compilation or building
    goodbye_object = Param.GoodByeSimObject(
        "GoodByeSimObject to say googbye after hello gem5"
    )


class GoodByeSimObject(SimObject):
    type = "GoodByeSimObject"
    cxx_header = "bootcamp/hello-sim-object/goodbye_sim_object.hh"
    cxx_class = "gem5::GoodByeSimObject"

    useless_var = Param.Int(
        2, "It's a useless variable is accounted to let go of default error"
    )
