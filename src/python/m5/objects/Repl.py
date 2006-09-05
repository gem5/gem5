from m5.SimObject import SimObject
from m5.params import *
class Repl(SimObject):
    type = 'Repl'
    abstract = True

class GenRepl(Repl):
    type = 'GenRepl'
    fresh_res = Param.Int("associativity")
    num_pools = Param.Int("capacity in bytes")
    pool_res = Param.Int("block size in bytes")
