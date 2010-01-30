from m5.params import *
from Network import RubyNetwork

class BaseGarnetNetwork(RubyNetwork):
    type = 'BaseGarnetNetwork'
    abstract = True
    flit_size = Param.Int(16, "flit size in bytes")
    number_of_pipe_stages = Param.Int(4, "router pipeline stages");
    vcs_per_class = Param.Int(4, "virtual channels per message class");
    buffer_size = Param.Int(4, "buffer size in bytes");
    using_network_testing = Param.Bool(False, "network testing enable");
