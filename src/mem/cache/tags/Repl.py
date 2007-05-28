from m5.SimObject import SimObject
from m5.params import *
class Repl(SimObject):
    type = 'Repl'
    abstract = True

class GenRepl(Repl):
    type = 'GenRepl'
    fresh_res = Param.Int("Fresh pool residency time")
    num_pools = Param.Int("Number of priority pools")
    pool_res = Param.Int("Pool residency time")
