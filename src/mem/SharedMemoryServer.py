from m5.SimObject import SimObject
from m5.params import Param
from m5.proxy import Parent


class SharedMemoryServer(SimObject):
    type = "SharedMemoryServer"
    cxx_header = "mem/shared_memory_server.hh"
    cxx_class = "gem5::memory::SharedMemoryServer"

    system = Param.System(
        Parent.any,
        "The system where the target shared memory is actually stored.")
    server_path = Param.String(
        "The unix socket path where the server should be running upon.")
