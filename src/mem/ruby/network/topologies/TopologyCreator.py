



from m5.params import *
from m5.objects import *

def instantiateTopology(topology, options, IntLink, ExtLink, Router):

    topo = Topology()
    topo.description = topology.description

    routers, int_links, ext_links = topology.makeTopology(options, IntLink, ExtLink, Router)

    topo.routers = routers
    topo.int_links = int_links
    topo.ext_links = ext_links

    return topo
