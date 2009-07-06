#!/usr/bin/ruby



class NetPort < LibRubyObject
  # number of transitions a SLICC state machine can transition per
  # cycle
  default_param :transitions_per_cycle, Integer, 32

  # buffer_size limits the size of all other buffers connecting to
  # SLICC Controllers.  When 0, infinite buffering is used.
  default_param :buffer_size, Integer, 32

  # added by SS for TBE
  default_param :number_of_TBEs, Integer, 128

  default_param :recycle_latency, Integer, 10
end

class Sequencer < IfacePort
  # Maximum number of requests (including prefetches) outstanding from
  # the sequencer
  default_param :max_outstanding_requests, Integer, 16

  # Maximum number of cycles a request is can be outstanding before
  # the Sequencer declares we're in deadlock/livelock
  default_param :deadlock_threshold, Integer, 500000

end

class Debug < LibRubyObject
  # For debugging purposes, one can enable a trace of all the protocol
  # state machine changes. Unfortunately, the code to generate the
  # trace is protocol specific. To enable the code for some of the
  # standard protocols,
  #   1. change protocol_trace = true
  #   2. enable debug in the Ruby Makefile
  #   3. set start_time = 1
  default_param :protocol_trace, Boolean, false

  # a string for filtering debugging output (for all g_debug vars see Debug.h)
  default_param :filter_string, String, "q"

  # filters debugging messages based on priority (low, med, high)
  default_param :verbosity_string, String, "none"

  # filters debugging messages based on a ruby time
  default_param :start_time, Integer, 1

  # sends debugging messages to a output filename
  default_param :output_filename, String, ""
end

class Topology < LibRubyObject
  # The default link latency between all nodes (internal and external)
  # in the toplogy
  default_param :link_latency, Integer, 1

  # the bandwidth from an external network port to it's corresponding
  # internal switch
  default_param :external_bw, Integer, 64

  # the bandwitch between internal switches in the network
  default_param :internal_bw, Integer, 16

  # indicates whether the topology config will be displayed in the
  # stats file
  default_param :print_config, Boolean, true
end

class Network < LibRubyObject
  default_param :endpoint_bandwidth, Integer, 10000
  default_param :adaptive_routing, Boolean, true
  default_param :number_of_virtual_networks, Integer, 6
  default_param :fan_out_degree, Integer, 4

  # default buffer size.  Setting to 0 indicates infinite buffering
  default_param :buffer_size, Integer, 3

  # local memory latency ?? NetworkLinkLatency
  default_param :link_latency, Integer, 1

  # on chip latency
  default_param :on_chip_latency, Integer, 1
end

class GarnetNetwork < Network
  default_param :flit_size, Integer, 16
  default_param :number_of_pipe_stages, Integer, 4
  default_param :vcs_per_class, Integer, 4
  default_param :buffer_size, Integer, 4
  default_param :using_network_testing, Boolean, false
end



#added by SS
class Tracer < LibRubyObject
  default_param :warmup_length, Integer, 1000000
end

#added by SS
class Profiler < LibRubyObject
  default_param :hot_lines, Boolean, false
  default_param :all_instructions, Boolean, false
end

#added by SS
class MI_example_CacheController < CacheController
  default_param :issue_latency, Integer, 2
  default_param :cache_response_latency, Integer, 12
end

class MI_example_DirectoryController < DirectoryController
  default_param :to_mem_ctrl_latency, Integer, 1
  default_param :directory_latency, Integer, 6
  default_param :memory_latency, Integer, 158
end


#added by SS
class MemoryControl < LibRubyObject

  default_param :mem_bus_cycle_multiplier, Integer, 10
  default_param :banks_per_rank, Integer, 8
  default_param :ranks_per_dimm, Integer, 2
  default_param :dimms_per_channel, Integer, 2
  default_param :bank_bit_0, Integer, 8
  default_param :rank_bit_0, Integer, 11
  default_param :dimm_bit_0, Integer, 12
  default_param :bank_queue_size, Integer, 12
  default_param :bank_busy_time, Integer, 11
  default_param :rank_rank_delay, Integer, 1
  default_param :read_write_delay, Integer, 2
  default_param :basic_bus_busy_time, Integer, 2
  default_param :mem_ctl_latency, Integer, 12
  default_param :refresh_period, Integer, 1560
  default_param :tFaw, Integer, 0
  default_param :mem_random_arbitrate, Integer, 0
  default_param :mem_fixed_delay, Integer, 0

end

class RubySystem

  # Random seed used by the simulation. If set to "rand", the seed
  # will be set to the current wall clock at libruby
  # initialization. Otherwise, set this to an integer.
  default_param :random_seed, Object, "rand"

  # When set to true, the simulation will insert random delays on
  # message enqueue times.  Note that even if this is set to false,
  # you can still have a non-deterministic simulation if random seed
  # is set to "rand".  This is because the Ruby swtiches use random
  # link priority elevation
  default_param :randomization, Boolean, false

  # tech_nm is the device size used to calculate latency and area
  # information about system components
  default_param :tech_nm, Integer, 45

  # default frequency for the system
  default_param :freq_mhz, Integer, 3000

  # the default cache block size in the system
  # libruby does not currently support different block sizes
  # among different caches
  # Must be a power of two
  default_param :block_size_bytes, Integer, 64

  # The default debug object. There shouldn't be a reason to ever
  # change this line.  To adjust debug paramters statically, adjust
  # them in the Debug class above.  To adjust these fields
  # dynamically, access this RubySystem object,
  # e.g. RubySystem.debug.protocol_trace = true
  default_param :debug, Debug, Debug.new("dbg0")
  default_param :tracer, Tracer, Tracer.new("tracer0")

  default_param :profiler, Profiler, Profiler.new("profiler0")
end

