#!/usr/bin/ruby
#
#  Creates a homogeneous CMP system with a single unified cache per
#  core and a crossbar network.  Uses the default parameters listed
#  below, which can be overridden if a wrapper script sets the hash
#  libruby_args.
#

require "cfg.rb"

# default values

num_cores = 2
L1_CACHE_SIZE_KB = 32
L1_CACHE_ASSOC = 8
L1_CACHE_LATENCY = 1
num_memories = 2
memory_size_mb = 1024
NUM_DMA = 1

# check for overrides

for i in 0..$*.size-1 do
  if $*[i] == "-p"
    num_cores = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-m"
    num_memories = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-s"
    memory_size_mb = $*[i+1].to_i
    i = i + 1
  end
end

net_ports = Array.new
iface_ports = Array.new

num_cores.times { |n|
  cache = SetAssociativeCache.new("l1u_"+n.to_s, L1_CACHE_SIZE_KB, L1_CACHE_LATENCY, L1_CACHE_ASSOC, "PSEUDO_LRU")
  sequencer = Sequencer.new("Sequencer_"+n.to_s, cache, cache)
  iface_ports << sequencer
  net_ports << MI_example_CacheController.new("L1CacheController_"+n.to_s,
                                   "L1Cache",
                                   [cache],
                                   sequencer)
}
num_memories.times { |n|
  directory = DirectoryMemory.new("DirectoryMemory_"+n.to_s, memory_size_mb/num_memories)
  memory_control = MemoryControl.new("MemoryControl_"+n.to_s)
  net_ports << MI_example_DirectoryController.new("DirectoryController_"+n.to_s,
                                       "Directory",
                                       directory, memory_control)
}
NUM_DMA.times { |n|
  dma_sequencer = DMASequencer.new("DMASequencer_"+n.to_s)
  iface_ports << dma_sequencer
  net_ports << DMAController.new("DMAController_"+n.to_s, "DMA", dma_sequencer)
}

topology = CrossbarTopology.new("theTopology", net_ports)
on_chip_net = Network.new("theNetwork", topology)

RubySystem.init(iface_ports, on_chip_net)
