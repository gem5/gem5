from m5.config import *
class Coherence(Enum): vals = ['uni', 'msi', 'mesi', 'mosi', 'moesi']

class CoherenceProtocol(SimObject):
    type = 'CoherenceProtocol'
    do_upgrades = Param.Bool(True, "use upgrade transactions?")
    protocol = Param.Coherence("name of coherence protocol")
