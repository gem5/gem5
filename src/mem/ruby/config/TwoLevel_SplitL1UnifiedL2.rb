#!/usr/bin/ruby
#
#  Creates a homogeneous CMP system with a single unified cache per
#  core and a crossbar network.  Uses the default parameters listed
#  below, which can be overridden using command line args.
#

require "cfg.rb"

# default values

num_cores = 2
L1_ICACHE_SIZE_KB = 32
L1_ICACHE_ASSOC = 8
L1_ICACHE_LATENCY = 1
L1_DCACHE_SIZE_KB = 32
L1_DCACHE_ASSOC = 8
L1_DCACHE_LATENCY = 1
L2_CACHE_SIZE_KB = 2048 # total size (sum of all banks)
L2_CACHE_ASSOC = 16
L2_CACHE_LATENCY = 12
num_l2_banks = num_cores
num_memories = 1
memory_size_mb = 1024
NUM_DMA = 1

protocol = "MOESI_CMP_directory"

# check for overrides

for i in 0..$*.size-1 do
  if $*[i] == "-c" or $*[i] == "--protocol"
    i += 1
    protocol = $*[i]
  elsif $*[i] == "-m"
    num_memories = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-p"
    num_cores = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-s"
    memory_size_mb = $*[i+1].to_i
    i = i + 1
  end
end

net_ports = Array.new
iface_ports = Array.new

assert(protocol == "MOESI_CMP_directory", __FILE__+" cannot be used with protocol "+protocol);

require protocol+".rb"

num_cores.times { |n|
  icache = SetAssociativeCache.new("l1i_"+n.to_s, L1_ICACHE_SIZE_KB, L1_ICACHE_LATENCY, L1_ICACHE_ASSOC, "PSEUDO_LRU")
  dcache = SetAssociativeCache.new("l1d_"+n.to_s, L1_DCACHE_SIZE_KB, L1_DCACHE_LATENCY, L1_DCACHE_ASSOC, "PSEUDO_LRU")
  sequencer = Sequencer.new("Sequencer_"+n.to_s, icache, dcache)
  iface_ports << sequencer
  if protocol == "MOESI_CMP_directory"
    net_ports << MOESI_CMP_directory_L1CacheController.new("L1CacheController_"+n.to_s,
                                                           "L1Cache",
                                                           icache, dcache,
                                                           sequencer,
                                                           num_l2_banks)
  end
}
num_l2_banks.times { |n|
  cache = SetAssociativeCache.new("l2u_"+n.to_s, L2_CACHE_SIZE_KB/num_l2_banks, L2_CACHE_LATENCY, L2_CACHE_ASSOC, "PSEUDO_LRU")
  if protocol == "MOESI_CMP_directory"
    net_ports << MOESI_CMP_directory_L2CacheController.new("L2CacheController_"+n.to_s,
                                                           "L2Cache",
                                                           cache)
  end
}
num_memories.times { |n|
  directory = DirectoryMemory.new("DirectoryMemory_"+n.to_s, memory_size_mb/num_memories)
  memory_control = MemoryControl.new("MemoryControl_"+n.to_s)
  if protocol == "MOESI_CMP_directory"
    net_ports << MOESI_CMP_directory_DirectoryController.new("DirectoryController_"+n.to_s,
                                                             "Directory",
                                                             directory, 
                                                             memory_control)
  end
}
NUM_DMA.times { |n|
  dma_sequencer = DMASequencer.new("DMASequencer_"+n.to_s)
  iface_ports << dma_sequencer
  if protocol == "MOESI_CMP_directory"
    net_ports << MOESI_CMP_directory_DMAController.new("DMAController_"+n.to_s,
                                                       "DMA",
                                                       dma_sequencer)
  end
}

topology = CrossbarTopology.new("theTopology", net_ports)
on_chip_net = Network.new("theNetwork", topology)

RubySystem.init(iface_ports, on_chip_net)
