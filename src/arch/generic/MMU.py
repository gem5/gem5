from m5.params import *
from m5.SimObject import SimObject


class Translator(SimObject):
    type = "NewMMU"
    cxx_header = "arch/generic/new_mmu.hh"
    cxx_class = "gem5::NewMMU"
