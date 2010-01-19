#!/usr/bin/ruby
#
#  Creates multiple on-chip nodes with three level of cache.
#

require "cfg.rb"

RubySystem.reset

# default values

num_cores = 2
l1_cache_size_bytes = 32768
l1_cache_assoc = 2
l1_cache_latency = 3
l2_cache_size_bytes = 1048576
l2_cache_assoc = 16
l2_cache_latency = 15
num_memories = 2
memory_size_mb = 1024
num_dma = 0
use_map = false
map_levels = 4
protocol = "MOESI_hammer"

# check for overrides


for i in 0..$*.size-1 do
  if $*[i] == "-c"
    protocol = $*[i+1]
    i = i+1
  elsif $*[i] == "-p"
    num_cores = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-m"
    num_memories = $*[i+1].to_i
    i = i+1
  elsif $*[i] == "-s"
    memory_size_mb = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-U"
    use_map = $*[i+1]
    i = i + 1
  elsif $*[i] == "-C"
    l1_cache_size_bytes = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-A"
    l1_cache_assoc = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-M"
    map_levels = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-D"
    num_dma = $*[i+1].to_i
    i = i + 1
  end
end

net_ports = Array.new
iface_ports = Array.new

assert(protocol == "MOESI_hammer", __FILE__ + " cannot be used with protocol " + protocol)

require protocol+".rb"

num_cores.times { |n|
  icache = SetAssociativeCache.new("l1i_"+n.to_s, 
                                   l1_cache_size_bytes, 
                                   l1_cache_latency, 
                                   l1_cache_assoc, 
                                   "PSEUDO_LRU")
  dcache = SetAssociativeCache.new("l1d_"+n.to_s, 
                                   l1_cache_size_bytes, 
                                   l1_cache_latency, 
                                   l1_cache_assoc, 
                                   "PSEUDO_LRU")
  l2cache = SetAssociativeCache.new("l2u_"+n.to_s, 
                                    l2_cache_size_bytes, 
                                    l2_cache_latency, 
                                    l2_cache_assoc, 
                                    "PSEUDO_LRU")
  sequencer = Sequencer.new("Sequencer_"+n.to_s, icache, dcache)
  iface_ports << sequencer
  net_ports << MOESI_hammer_CacheController.new("L1CacheController_"+n.to_s,
                                                "L1Cache",
                                                icache,
                                                dcache,
                                                l2cache,
                                                sequencer)
}
num_memories.times { |n|
  directory = DirectoryMemory.new("DirectoryMemory_"+n.to_s, memory_size_mb/num_memories)
  memory_control = MemoryControl.new("MemoryControl_"+n.to_s)
  net_ports << MOESI_hammer_DirectoryController.new("DirectoryController_"+n.to_s,
                                                    "Directory",
                                                    directory, 
                                                    memory_control)
}
num_dma.times { |n|
  dma_sequencer = DMASequencer.new("DMASequencer_"+n.to_s)
  iface_ports << dma_sequencer
  net_ports << MOESI_hammer_DMAController.new("DMAController_"+n.to_s, "DMA", dma_sequencer)
}

topology = CrossbarTopology.new("theTopology", net_ports)
on_chip_net = Network.new("theNetwork", topology)

RubySystem.init(iface_ports, on_chip_net)
