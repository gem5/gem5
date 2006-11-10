from m5 import build_env
from m5.params import *
from m5.proxy import *
from MemObject import MemObject
from Tsunami import BadAddr

class Bus(MemObject):
    type = 'Bus'
    port = VectorPort("vector port for connecting devices")
    bus_id = Param.Int(0, "blah")
    clock = Param.Clock("1GHz", "bus clock speed")
    width = Param.Int(64, "bus width (bytes)")
    responder_set = Param.Bool(False, "Did the user specify a default responder.")
    if build_env['FULL_SYSTEM']:
        default = Port(Self.responder.pio, "Default port for requests that aren't handled by a device.")
        responder = BadAddr(pio_addr=0x0, pio_latency="1ps")
    else:
        default = Port("Default port for requests that aren't handled by a device.")
