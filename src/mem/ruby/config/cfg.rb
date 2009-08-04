#!/usr/bin/ruby

class AssertionFailure < RuntimeError
end

class Boolean
  def self.is_a?(obj)
    return self.name == "Boolean"
  end
end

def assert(condition,message)
  unless condition
    raise AssertionFailure, "Assertion failed: #{message}"
  end
end

class LibRubyObject
  @@all_objs = Array.new
  attr_reader :obj_name
  @@default_params = Hash.new

  def initialize(obj_name)
    assert obj_name.is_a?(String), "Obj_Name must be a string"
    @obj_name = obj_name
    @@all_objs << self
    @params = Hash.new
  end

  def cppClassName()
    raise NotImplementedException
  end

  def self.param(param_name, type)
    idx = self.name.to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    @@default_params[idx][param_name] = nil
    send :define_method, param_name do
      @params[param_name] = @@default_params[idx][param_name] if ! @params.key?(param_name)
      @params[param_name]
    end
    method_name = (param_name.to_s + "=").to_sym
    send :define_method, method_name do |val|
      if val.is_a?(FalseClass) || val.is_a?(TrueClass)
        assert type.is_a?(Boolean), "default value of param \"#{param_name}\" must be either true or false"
      else
        assert val.is_a?(type), "default value of param \"#{param_name}\" does not match type #{type}"
      end
#      assert val.is_a?(type), "#{param_name} must be of type #{type}"
      @params[param_name] = val
    end
  end

  def self.default_param(param_name, type, default)
    idx = self.name.to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    if default.is_a?(FalseClass) || default.is_a?(TrueClass)
      assert type.is_a?(Boolean), "default value of param \"#{param_name}\" must be either true or false"
    else
      assert default.is_a?(type), "default value of param \"#{param_name}\" does not match type #{type}"
    end
    @@default_params[idx][param_name] = default
    send :define_method, param_name do
      @params[param_name] = @@default_params[idx][param_name] if ! @params.key?(param_name)
      @params[param_name]
    end
    method_name = (param_name.to_s + "=").to_sym
    send :define_method, method_name do |val|
      assert val.is_a?(type), "#{param_name} must be of type #{type}"
      @params[param_name] = val
    end
  end

  def applyDefaults()
    idx = self.class.name.to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    @@default_params[idx].each { |key, val|
      @params[key] = val if ! @params.key?(key)
    }
  end

  def argv()
    str = ""

    applyDefaults

    @params.each { |key, val|
      str += key.id2name + " "
      if val.is_a?(LibRubyObject)
        str += val.obj_name + " "
      else
        if val.is_a?(String) and val == ""
          str += "null "
        else
          str += val.to_s + " "
        end
      end
    }
    return str
  end

  def self.printConstructors()
    str = ""
    @@all_objs.each { |obj|
      str += obj.cppClassName + " " + obj.obj_name + " " + obj.argv + "\n"
    }
    return str
  end
  def self.all()
    @@all_objs
  end
end

class IfacePort < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
  end

  def bochsConnType
    raise NotImplementedException
  end
end

class NetPort < LibRubyObject
  attr :mach_type
  attr_reader :version

  @@type_cnt = Hash.new
  @type_id
  def initialize(obj_name, mach_type)
    super(obj_name)
    @mach_type = mach_type
    @@type_cnt[mach_type] ||= 0
    @type_id = @@type_cnt[mach_type]
    @@type_cnt[mach_type] += 1

    idx = "NetPort".to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    @@default_params[idx].each { |key, val|
      @params[key] = val if ! @params.key?(key)
    }
  end

  def port_name
    mach_type
  end
  def port_num
    @type_id
  end
  def cppClassName
    "NetPort"
  end
end

class MemoryVector < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
  end

  def cppClassName
    "MemoryController"
  end
end

class Debug < LibRubyObject
  def initialize *args
    case args.size
      when 1
      super(args[0])
      when 6
      init_params *args[1]
    else
      raise Exception
    end
  end

  def init_params (protocol_trace, filter_string, verbosity_string, start_time, output_filename)
    @params[:protocol_trace] = protocol_trace
    @params[:filter_string] = filter_string
    @params[:verbosity_string] = verbosity_string
    @params[:start_time] = start_time
    @params[:output_filename] = output_filename
  end

  def cppClassName
    "Debug"
  end
end

class RubySystem

  @@params = Hash.new
  @@network = nil

  def self.init(iface_ports, network)
    @@iface_ports = iface_ports
    @@network = network
  end

  def self.default_param(param_name, type, default)
    if default.is_a?(FalseClass) || default.is_a?(TrueClass)
      assert type.is_a?(Boolean), "default value of param \"#{param_name}\" must be either true or false"
    else
      assert default.is_a?(type), "default value of param \"#{param_name}\" does not match type #{type}"
    end
    @@params[param_name] = default
    method_name = (param_name.to_s).to_sym
    instance_eval <<-EOS
       def #{method_name.to_s}
         @@params[:#{param_name.to_s}]
       end
    EOS
    instance_eval <<-EOS
       def #{method_name.to_s}=(val)
         @@params[:#{param_name.to_s}] = val
       end
    EOS
  end

  def self.getConfig()
    # get current time for random seed if set to "rand"
    if @@params[:random_seed] == "rand"
      t = Time.now
      @@params[:random_seed] = t.usec.to_i
    end
    if ! @@params[:random_seed].is_a?(Integer)
      raise TypeException
    end
    str = "System sys0 "+argv+"\n"
    LibRubyObject.all.each { |obj|
      if obj.is_a?(SetAssociativeCache)
        obj.calculateLatency
      end
    }
    str += LibRubyObject.printConstructors
    #puts str.gsub('%',' ').gsub('#','\n')
    return str
  end

  def self.generateConfig()
    puts getConfig
  end

  def self.printIfacePorts()
    @@iface_ports.each { |port|
      print port.obj_name, " "
    }
    puts
  end

  def self.getBochsConnections()
    ports = Hash.new
    @@iface_ports.each { |port|
      ports[port.obj_name] = port.bochsConnType
    }
    return ports
  end

  def self.getMemorySizeMB()
    DirectoryMemory.memorySizeMB
  end

  # override the default accessors (generated by default_param) for random_seed
  def self.random_seed=(seed)
    assert (val.is_a?(Integer) or val == "rand"), "RubySystem.random_seed takes either an integer value or the string \"rand\""
    @@params[:random_seed] = seed
  end

private

  def self.argv()
    str = ""
    @@params.each { |key, val|
      str += key.id2name + " "
      str += val.to_s + " "
    }
    return str
  end

  def self.writeConfig()
    @@network.printTopology
  end

end

#require "defaults.rb"



class CacheController < NetPort
  @@total_cache_controllers = Hash.new

  def initialize(obj_name, mach_type, caches)
    super(obj_name, mach_type)
    caches.each { |cache|
      cache.controller = self
    }

    if !@@total_cache_controllers.has_key?(mach_type)
      @@total_cache_controllers[mach_type] = 0
    end
    @version = @@total_cache_controllers[mach_type]
    @@total_cache_controllers[mach_type] += 1
    
    # call inhereted parameters
    transitions_per_cycle
    buffer_size
    number_of_TBEs
    recycle_latency
  end

  def argv()
    vec = "version "+@version.to_s
    vec += " transitions_per_cycle "+@params[:transitions_per_cycle].to_s
    vec += " buffer_size "+@params[:buffer_size].to_s
    vec += " number_of_TBEs "+@params[:number_of_TBEs].to_s
    vec += " recycle_latency "+@params[:recycle_latency].to_s
  end

  def cppClassName()
    "generated:"+@mach_type
  end
end

class L1CacheController < CacheController
  attr :sequencer

  def initialize(obj_name, mach_type, caches, sequencer)
    super(obj_name, mach_type, caches)

    @sequencer = sequencer
    @sequencer.controller = self
    @sequencer.version = @version
  end

  def argv()
    vec = super()
    vec += " sequencer "+@sequencer.obj_name
  end
end

class DirectoryController < NetPort
  @@total_directory_controllers = 0
  attr :directory
  attr :memory_control

  def initialize(obj_name, mach_type, directory, memory_control)
    super(obj_name, mach_type)

    @directory = directory
    directory.controller = self

    @memory_control = memory_control

    @version = @@total_directory_controllers
    @@total_directory_controllers += 1
    buffer_size()
  end

  def argv()
    "version "+@version.to_s+" directory_name "+@directory.obj_name+" transitions_per_cycle "+@params[:transitions_per_cycle].to_s + " buffer_size "+@params[:buffer_size].to_s + " number_of_TBEs "+@params[:number_of_TBEs].to_s + " memory_controller_name "+@memory_control.obj_name + " recycle_latency "+@params[:recycle_latency].to_s
  end

  def cppClassName()
    "generated:"+@mach_type
  end

end

class DMAController < NetPort
  @@total_dma_controllers = 0
  attr :dma_sequencer
  def initialize(obj_name, mach_type, dma_sequencer)
    super(obj_name, mach_type)
    @dma_sequencer = dma_sequencer
    @version = @@total_dma_controllers
    @@total_dma_controllers += 1
    dma_sequencer.controller = self
    buffer_size
  end

  def argv()
    "version "+@version.to_s+" dma_sequencer "+@dma_sequencer.obj_name+" transitions_per_cycle "+@params[:transitions_per_cycle].to_s + " buffer_size "+@params[:buffer_size].to_s + " number_of_TBEs "+@params[:number_of_TBEs].to_s +  " recycle_latency "+@params[:recycle_latency].to_s
  end

  def cppClassName()
    "generated:"+@mach_type
  end
end

class Cache < LibRubyObject
  attr :size_kb, :latency
  attr_writer :controller
  def initialize(obj_name, size_kb, latency)
    super(obj_name)
    assert size_kb.is_a?(Integer), "Cache size must be an integer"
    @size_kb = size_kb
    @latency = latency
  end

  def args
    "controller "+@controller.obj_name+" size_kb "+@size_kb.to_s+" latency "+@latency.to_s
  end
end

class SetAssociativeCache < Cache
  attr :assoc, :replacement_policy

  # latency can be either an integer, a float, or the string "auto"
  #  when an integer, it represents the number of cycles for a hit
  #  when a float, it represents the cache access time in ns
  #  when set to "auto", libruby will attempt to find a realistic latency by running CACTI
  def initialize(obj_name, size_kb, latency, assoc, replacement_policy)
    super(obj_name, size_kb, latency)
    @assoc = assoc
    @replacement_policy = replacement_policy
  end

  def calculateLatency()
    if @latency == "auto"
      cacti_args = Array.new()
      cacti_args << (@size_kb*1024) <<  RubySystem.block_size_bytes << @assoc
      cacti_args << 1 << 0 << 0 << 0 << 1
      cacti_args << RubySystem.tech_nm << RubySystem.block_size_bytes*8
      cacti_args << 0 << 0 << 0 << 1 << 0 << 0 << 0 << 0 << 1
      cacti_args << 360 << 0 << 0 << 0 << 0 << 1 << 1 << 1 << 1 << 0 << 0
      cacti_args << 50 << 10 << 10 << 0 << 1 << 1

      cacti_cmd = File.dirname(__FILE__) + "/cacti/cacti " + cacti_args.join(" ")

      IO.popen(cacti_cmd) { |pipe|
        str1 = pipe.readline
        str2 = pipe.readline
        results = str2.split(", ")
        if results.size != 61
          print "CACTI ERROR: CACTI produced unexpected output.\n"
          print "Are you using the version shipped with libruby?\n"
          raise Exception
        end
        latency_ns = results[5].to_f
        if (latency_ns == "1e+39")
          print "CACTI ERROR:  CACTI was unable to realistically model the cache ",@obj_name,"\n"
          print "Either change the cache parameters or manually set the latency values\n"
          raise Exception
        end
        clk_period_ns = 1e9 * (1.0 / (RubySystem.freq_mhz * 1e6))
        latency_cycles = (latency_ns / clk_period_ns).ceil
        @latency = latency_cycles
      }
    elsif @latency.is_a?(Float)
      clk_period_ns = 1e9 * (1.0 / (RubySystem.freq_mhz * 1e6))
      latency_cycles = (@latency / clk_period_ns).ceil
      @latency = latency_cycles
    elsif ! @latency.is_a?(Integer)
      raise Exception
    end
  end

  def argv()
    args+" assoc "+@assoc.to_s+" replacement_policy "+@replacement_policy
  end

  def cppClassName()
    "SetAssociativeCache"
  end
end

class DirectoryMemory < LibRubyObject
  attr :size_mb
  attr_writer :controller
  @@total_size_mb = 0

  def initialize(obj_name, size_mb)
    super(obj_name)
    @size_mb = size_mb
    @@total_size_mb += size_mb
  end

  def argv()
    "version "+@controller.version.to_s+" size_mb "+@size_mb.to_s+" controller "+@controller.obj_name
  end

  def cppClassName()
    "DirectoryMemory"
  end

  def self.memorySizeMB()
    @@total_size_mb
  end
end

#added by SS
class MemoryControl < LibRubyObject
  attr :name
  def initialize(obj_name)
    super(obj_name)
    @name = obj_name
  end

  def argv()
    vec = super()
    vec += " mem_bus_cycle_multiplier "+mem_bus_cycle_multiplier.to_s
    vec += " banks_per_rank "+banks_per_rank.to_s
    vec += " ranks_per_dimm "+ranks_per_dimm.to_s
    vec += " dimms_per_channel "+dimms_per_channel.to_s
    vec += " bank_bit_0 "+bank_bit_0.to_s
    vec += " rank_bit_0 "+rank_bit_0.to_s
    vec += " dimm_bit_0 "+dimm_bit_0.to_s
    vec += " bank_queue_size "+bank_queue_size.to_s
    vec += " bank_busy_time "+bank_busy_time.to_s
    vec += " rank_rank_delay "+rank_rank_delay.to_s
    vec += " read_write_delay "+read_write_delay.to_s
    vec += " basic_bus_busy_time "+basic_bus_busy_time.to_s
    vec += " mem_ctl_latency "+mem_ctl_latency.to_s
    vec += " refresh_period "+refresh_period.to_s
    vec += " tFaw "+tFaw.to_s
    vec += " mem_random_arbitrate "+mem_random_arbitrate.to_s
    vec += " mem_fixed_delay "+mem_fixed_delay.to_s
    vec += " memory_controller_name "+@name

  end


  def cppClassName()
    "MemoryControl"
  end
end



class Sequencer < IfacePort

  def cppClassName()
    "Sequencer"
  end

  param :controller, NetPort  # must be set after initialization
  param :icache, Cache
  param :dcache, Cache
  param :version, Integer

  def initialize(obj_name, icache, dcache)
    super(obj_name)
    self.icache=icache
    self.dcache=dcache
  end

  def bochsConnType()
    return "cpu"+version.to_s
  end

end



class DMASequencer < IfacePort
  def initialize(obj_name)
    super(obj_name)
    @params = {
      :controller => nil,
      :version => nil
    }
  end

  def controller=(controller)
    @params[:controller] = controller.obj_name
    @params[:version] = controller.version
  end

  def cppClassName()
    "DMASequencer"
  end

  def bochsConnType()
    return "dma"+@params[:version].to_s
  end
end

class IntNode
  @@num = 0
  def initialize()

  end
end

class Network < LibRubyObject
end

class Topology < LibRubyObject
  attr :net_ports
  param :network, Network
  def initialize(name, net_ports)
    super(name)
    @net_ports = net_ports
  end

  def cppClassName
    "Topology"
  end
end

class Network < LibRubyObject
  param :topology, Topology
  def initialize(name, topo)
    super(name)
    @params[:topology] = topo
    topo.network= self
  end

  def argv()
    vec = super()

    vec += " endpoint_bandwidth "+endpoint_bandwidth.to_s
    vec += " adaptive_routing "+adaptive_routing.to_s
    vec += " number_of_virtual_networks "+number_of_virtual_networks.to_s
    vec += " fan_out_degree "+fan_out_degree.to_s

    vec += " buffer_size "+buffer_size.to_s
    vec += " link_latency "+adaptive_routing.to_s
    vec += " on_chip_latency "+on_chip_latency.to_s

  end

  def printTopology()
    topology().printFile
  end
  def cppClassName()
    "SimpleNetwork"
  end

end

class PtToPtTopology < Topology

  param :connections,String

  def initialize(name, net_ports)
    super(name, net_ports)
    @params[:connections] = ""
    @net_ports.each_index { |idx|
      @params[:connections] << ("ext_node:"+@net_ports[idx].port_name+":"+@net_ports[idx].port_num.to_s)
      @params[:connections] << ("%int_node:"+ idx.to_s+ "%link_latency:"+ link_latency.to_s)
      @params[:connections] << ("%bw_multiplier:"+external_bw.to_s+"#")
    }
    @net_ports.each_index { |outer_idx|
      @net_ports.each_index { |inner_idx|
        if (outer_idx != inner_idx)
          @params[:connections] << ("int_node:"+ outer_idx.to_s+ "%int_node:"+ inner_idx.to_s)
          @params[:connections] << ("%link_latency:"+link_latency.to_s+"%bw_multiplier:"+internal_bw.to_s)
          @params[:connections] << ("%link_weight:"+1.to_s+"#")
        end
      }
    }
    # call the accessors of the parent class to initialize them
    # need to find a better method!!
    print_config
  end

end

class CrossbarTopology < Topology
  param :connections,String

  def initialize(name, net_ports)
    super(name, net_ports)
    @params[:connections] = ""
    crossbar_node = @net_ports.size
    @net_ports.each_index { |idx|
      @params[:connections] << ("ext_node:"+@net_ports[idx].port_name+":"+@net_ports[idx].port_num.to_s)
      @params[:connections] << ("%int_node:"+ idx.to_s+ "%link_latency:"+ link_latency.to_s)
      @params[:connections] << ("%bw_multiplier:"+external_bw.to_s+"#")
    }
    @net_ports.each_index { |idx|
      @params[:connections] << ("int_node:"+idx.to_s+"%int_node:"+crossbar_node.to_s)
      @params[:connections] << ("%link_latency:"+link_latency.to_s+"%bw_multiplier:"+internal_bw.to_s)
      @params[:connections] << ("%link_weight:"+1.to_s+"#")
    }
    print_config
  end
end

#added by SS
class Tracer < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
  end

  def cppClassName()
    "Tracer"
  end

end

class Profiler < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
  end

  def cppClassName()
    "Profiler"
  end

end

#added by SS
class GarnetNetwork < Network
  def initialize(name, topo)
    super(name, topo)
  end
  def argv()
    vec = super()
    vec += " flit_size "+flit_size.to_s
    vec += " number_of_pipe_stages "+number_of_pipe_stages.to_s
    vec += " vcs_per_class "+vcs_per_class.to_s
    vec += " buffer_size "+buffer_size.to_s
    vec += " using_network_testing "+using_network_testing.to_s
  end

end

class GarnetFixedPipeline < GarnetNetwork
  def initialize(name, net_ports)
    super(name, net_ports)
  end

  def argv()
    super()
  end

  def cppClassName()
    "GarnetNetwork_d"
  end
end

class GarnetFlexiblePipeline < GarnetNetwork
  def initialize(name, net_ports)
    super(name, net_ports)
  end

  def argv()
    super()
  end

  def cppClassName()
    "GarnetNetwork"
  end
end

require "defaults.rb"
