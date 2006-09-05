from m5.SimObject import SimObject
from m5.params import *
class Coherence(Enum): vals = ['uni', 'msi', 'mesi', 'mosi', 'moesi']

class CoherenceProtocol(SimObject):
    type = 'CoherenceProtocol'
    do_upgrades = Param.Bool(True, "use upgrade transactions?")
    protocol = Param.Coherence("name of coherence protocol")
