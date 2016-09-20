# Copyright (c) 2015-2016 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Curtis Dunham

import sst
import sys
import os

lat="1 ns"
buslat="2 ns"
clockRate = "1GHz"


def getenv(name):
    res = ""
    try:
        res = os.environ[name]
    except KeyError:
        pass
    return res

def debug(d):
    try:
        r = int(getenv(d))
    except ValueError:
        return 0
    return r

baseCacheParams = ({
    "debug" :debug("DEBUG"),
    "debug_level" : 6,
    "coherence_protocol" : "MSI",
    "replacement_policy" : "LRU",
    "cache_line_size" : 64,
    "cache_frequency" : clockRate
    })

l1CacheParams = ({
    "debug" : debug("DEBUG"),
    "debug_level" : 6,
    "L1" : 1,
    "cache_size" : "64 KB",
    "associativity" : 4,
    "access_latency_cycles" : 2,
    "low_network_links" : 1
    })

l2CacheParams = ({
    "debug" : debug("DEBUG"),
    "debug_level" : 6,
    "L1" : 0,
    "cache_size" : "256 KB",
    "associativity" : 8,
    "access_latency_cycles" : 8,
    "high_network_links" : 1,
    "mshr_num_entries" : 4096,
    "low_network_links" : 1
    })


GEM5 = sst.Component("system", "gem5.gem5")
GEM5.addParams({
    "comp_debug" : debug("GEM5_DEBUG"),
    "gem5DebugFlags" : debug("M5_DEBUG"),
    "frequency" : clockRate,
    "cmd" : "configs/example/fs.py --num-cpus 4 --disk-image=vexpress64-openembedded_minimal-armv8_20130623-376.img --root-device=/dev/sda2 --kernel=vmlinux.aarch64.20140821 --dtb-filename=vexpress.aarch64.20140821.dtb --mem-size=256MB --machine-type=VExpress_EMM64 --cpu-type=timing --external-memory-system=sst"
    })

bus = sst.Component("membus", "memHierarchy.Bus")
bus.addParams({
    "bus_frequency": "2GHz",
    "debug" : debug("DEBUG"),
    "debug_level" : 8
    })

def buildL1(name, m5, connector):
    cache = sst.Component(name, "memHierarchy.Cache")
    cache.addParams(baseCacheParams)
    cache.addParams(l1CacheParams)
    link = sst.Link("cpu_%s_link"%name)
    link.connect((m5, connector, lat), (cache, "high_network_0", lat))
    return cache

SysBusConn = buildL1("gem5SystemBus", GEM5, "system.external_memory.port")
bus_port = 0
link = sst.Link("sysbus_bus_link")
link.connect((SysBusConn, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))

bus_port = bus_port + 1
ioCache = buildL1("ioCache", GEM5, "system.iocache.port")
ioCache.addParams({
    "debug" : 0,
    "debug_level" : 6,
    "cache_size" : "16 KB",
    "associativity" : 4
    })
link = sst.Link("ioCache_bus_link")
link.connect((ioCache, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))

def buildCPU(m5, num):
    l1iCache = buildL1("cpu%u.l1iCache" % num, m5, "system.cpu%u.icache.port" % num)
    l1dCache = buildL1("cpu%u.l1dCache" % num, m5, "system.cpu%u.dcache.port" % num)
    itlbCache = buildL1("cpu%u.itlbCache" % num, m5, "system.cpu%u.itb_walker_cache.port" % num)
    dtlbCache = buildL1("cpu%u.dtlbCache" % num, m5, "system.cpu%u.dtb_walker_cache.port" % num)
    l1dCache.addParams({
        "debug" : 0,
        "debug_level" : 10,
        "snoop_l1_invalidations" : 1
    })

    global bus_port
    link = sst.Link("cpu%u.l1iCache_bus_link" % num) ; bus_port = bus_port + 1
    link.connect((l1iCache, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))
    link = sst.Link("cpu%u.l1dCache_bus_link" % num) ; bus_port = bus_port + 1
    link.connect((l1dCache, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))
    link = sst.Link("cpu%u.itlbCache_bus_link" % num) ; bus_port = bus_port + 1
    link.connect((itlbCache, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))
    link = sst.Link("cpu%u.dtlbCache_bus_link" % num) ; bus_port = bus_port + 1
    link.connect((dtlbCache, "low_network_0", buslat), (bus, "high_network_%u" % bus_port, buslat))

buildCPU(GEM5, 0)
buildCPU(GEM5, 1)
buildCPU(GEM5, 2)
buildCPU(GEM5, 3)

l2cache = sst.Component("l2cache", "memHierarchy.Cache")
l2cache.addParams(baseCacheParams)
l2cache.addParams(l2CacheParams)
l2cache.addParams({
      "network_address" : "2"
})

link = sst.Link("l2cache_bus_link")
link.connect((l2cache, "high_network_0", buslat), (bus, "low_network_0", buslat))

memory = sst.Component("memory", "memHierarchy.MemController")
memory.addParams({
    "request_width" : 64,
    "coherence_protocol" : "MSI",
    "access_time" : "25 ns",
    "backend.mem_size" : "256MiB",
    "clock" : "2GHz",
    "debug" : debug("DEBUG"),
    "range_start" : 0, # 2 * (1024 ** 3), # it's behind a directory controller.
    })

comp_chiprtr = sst.Component("chiprtr", "merlin.hr_router")
comp_chiprtr.addParams({
      "xbar_bw" : "16GB/s",
      "link_bw" : "16GB/s",
      "input_buf_size" : "1KB",
      "num_ports" : "3",
      "flit_size" : "72B",
      "output_buf_size" : "1KB",
      "id" : "0",
      "topology" : "merlin.singlerouter"
})
comp_dirctrl = sst.Component("dirctrl", "memHierarchy.DirectoryController")
comp_dirctrl.addParams({
      "coherence_protocol" : "MSI",
      "network_address" : "1",
      "entry_cache_size" : "16384",
      "network_bw" : "1GB/s",
      "addr_range_start" : 2 * (1024 ** 3),
      "addr_range_end" : 2 * (1024 ** 3) + 256 * (1024 ** 2)
})

sst.Link("link_cache_net_0").connect((l2cache, "directory", "10ns"), (comp_chiprtr, "port2", "2ns"))
sst.Link("link_dir_net_0").connect((comp_chiprtr, "port1", "2ns"), (comp_dirctrl, "network", "2ns"))
sst.Link("l2cache_io_link").connect((comp_chiprtr, "port0", "2ns"), (GEM5, "network", buslat))
sst.Link("link_dir_mem_link").connect((comp_dirctrl, "memory", "10ns"), (memory, "direct_link", "10ns"))
