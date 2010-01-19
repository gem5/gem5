#!/usr/bin/ruby
#
#  Creates a homogeneous CMP system with a single unified cache per
#  core and a crossbar network.  Uses the default parameters listed
#  below, which can be overridden if a wrapper script sets the hash
#  libruby_args.
#

require "cfg.rb"

RubySystem.reset

# default values

num_cores = 2
l1_cache_size_kb = 32768
l1_cache_assoc = 8
l1_cache_latency = 1
num_memories = 2
memory_size_mb = 1024
num_dma = 1
protocol = "MI_example"

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
  elsif $*[i] == "-R"
    if $*[i+1] == "rand"
      RubySystem.random_seed = "rand"
    else
      RubySystem.random_seed = $*[i+1].to_i
    end
    i = i+ 1
  elsif $*[i] == "-s"
    memory_size_mb = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-C"
    l1_cache_size_bytes = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-A"
    l1_cache_assoc = $*[i+1].to_i
    i = i + 1
  elsif $*[i] == "-D"
    num_dma = $*[i+1].to_i
    i = i + 1
  end
end

net_ports = Array.new
iface_ports = Array.new

assert(protocol == "MI_example", __FILE__ + " cannot be used with protocol " + protocol)

require protocol+".rb"

num_cores.times { |n|
  cache = SetAssociativeCache.new("l1u_"+n.to_s, l1_cache_size_bytes, l1_cache_latency, l1_cache_assoc, "PSEUDO_LRU")
  sequencer = Sequencer.new("Sequencer_"+n.to_s, cache, cache)
  iface_ports << sequencer
  net_ports << MI_example_CacheController.new("L1CacheController_"+n.to_s,
                                   "L1Cache",
                                   cache,
                                   sequencer)
}
num_memories.times { |n|
  directory = DirectoryMemory.new("DirectoryMemory_"+n.to_s, memory_size_mb/num_memories)
  memory_control = MemoryControl.new("MemoryControl_"+n.to_s)
  net_ports << MI_example_DirectoryController.new("DirectoryController_"+n.to_s,
                                       "Directory",
                                       directory, memory_control)
}
num_dma.times { |n|
  dma_sequencer = DMASequencer.new("DMASequencer_"+n.to_s)
  iface_ports << dma_sequencer
  net_ports << MI_example_DMAController.new("DMAController_"+n.to_s, "DMA", dma_sequencer)
}

topology = CrossbarTopology.new("theTopology", net_ports)
on_chip_net = Network.new("theNetwork", topology)

RubySystem.init(iface_ports, on_chip_net)
