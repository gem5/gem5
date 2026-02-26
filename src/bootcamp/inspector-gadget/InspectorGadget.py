from m5.objects.ClockedObject import ClockedObject

# importing ports from m5.params
from m5.params import *


class InspectorGadget(ClockedObject):
    type = "InspectorGadget"
    cxx_header = "bootcamp/inspector-gadget/inspector_gadget.hh"
    cxx_class = "gem5::InspectorGadget"

    # this will be a object between the cpu and the memory
    cpu_side_port = ResponsePort("ResponsePort to receive req from cpu side")
    mem_side_port = RequestPort("RequestPort to raise requests to the memory")

    # there is one inspection buffer to inspect the request \
    # recieved form the cpu
    inspection_buffer_entries = Param.Int(
        32, "inspectionbuffer size to store the rqust rcvd from cpu"
    )

    # there is one response buffer to store the responses \
    # recieved form the memory
    response_buffer_entries = Param.Int(
        32, "response buffer size to store response recd from memory"
    )
