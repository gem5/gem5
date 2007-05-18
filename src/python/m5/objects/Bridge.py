from m5.params import *
from MemObject import MemObject

class Bridge(MemObject):
    type = 'Bridge'
    side_a = Port('Side A port')
    side_b = Port('Side B port')
    req_size_a = Param.Int(16, "The number of requests to buffer")
    req_size_b = Param.Int(16, "The number of requests to buffer")
    resp_size_a = Param.Int(16, "The number of requests to buffer")
    resp_size_b = Param.Int(16, "The number of requests to buffer")
    delay = Param.Latency('0ns', "The latency of this bridge")
    nack_delay = Param.Latency('0ns', "The latency of this bridge")
    write_ack = Param.Bool(False, "Should this bridge ack writes")
    fix_partial_write_a = Param.Bool(False, "Should this bridge fixup partial block writes")
    fix_partial_write_b = Param.Bool(False, "Should this bridge fixup partial block writes")
