#!/usr/bin/ruby

root = File.dirname(File.expand_path(__FILE__))
require root+'/assert.rb'

class Boolean
  def self.is_a?(obj)
    return self.name == "Boolean"
  end
end

class LibRubyObject
  @@all_objs = Array.new
  @@default_params = Hash.new
  @@param_types = Hash.new

  attr_reader :obj_name

  def initialize(obj_name)
    assert obj_name.is_a?(String), "Obj_Name must be a string"
    @obj_name = obj_name
    @@all_objs << self
    @params = Hash.new

    # add all parent parameter accessors if they don't exist
    self.class.ancestors.each { |ancestor|
      if @@default_params.key?(ancestor.name.to_sym)
        @@default_params[ancestor.name.to_sym].each { |p, default|
          p = p.to_sym
          @params[p] = default
          if ! respond_to?(p)
            self.class.send(:define_method, p) {
              @params[p] = @@default_params[ancestor.name.to_sym][p] if ! @params.key?(p)
              return @params[p]
            }
          end
          setter_method_name = (p.to_s + "=").to_sym
          if ! respond_to?(setter_method_name)
            self.class.send(:define_method, setter_method_name) { |val|
              type = @@param_types[ancestor.name.to_sym][p]
              if val.is_a?(FalseClass) || val.is_a?(TrueClass)
                assert type.is_a?(Boolean), "default value of param \"#{p}\" must be either true or false"
              else
                assert val.is_a?(type), "default value of param \"#{p}\", which is of type #{val.class.name} does not match expected type #{type}"
              end
              @params[p] = val
            }
          end
        }
      end
    }
  end

  def cppClassName()
    raise NotImplementedException
  end

  def self.param(param_name, type)
    idx = self.name.to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    @@default_params[idx][param_name] = nil
    @@param_types[idx] = Hash.new if ! @@param_types.key?(idx)
    @@param_types[idx][param_name] = type
  end

  def self.default_param(param_name, type, default)

    if default.is_a?(FalseClass) || default.is_a?(TrueClass)
      assert type.is_a?(Boolean), "default value of param \"#{param_name}\" must be either true or false"
    else
      assert default.is_a?(type), "default value of param \"#{param_name}\" does not match type #{type}"
    end

    idx = self.name.to_sym
    @@default_params[idx] = Hash.new if ! @@default_params.key?(idx)
    @@default_params[idx][param_name] = default
    @@param_types[idx] = Hash.new if ! @@param_types.key?(idx)
    @@param_types[idx][param_name] = type

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
      assert(val != nil, "parameter #{key} is nil")
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
  param :version, Integer

  @@type_cnt = Hash.new
  def initialize(obj_name, mach_type)
    super(obj_name)
    @mach_type = mach_type
    @@type_cnt[mach_type] ||= 0
    self.version= @@type_cnt[mach_type] # sets the version parameter

    @@type_cnt[mach_type] += 1

  end

  def port_name
    mach_type
  end
  def port_num
    version
  end
  def self.totalOfType(mach_type)
    return @@type_cnt[mach_type]
  end
  def cppClassName()
    "generated:"+@mach_type
  end

end

class MemoryVector < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
  end

  def cppClassName
    "MemoryVector"
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
  @@defaults = Hash.new
  @@network = nil

  def self.init(iface_ports, network)
    @@iface_ports = iface_ports
    @@network = network
  end

  def self.reset()
    @@iface_ports = nil
    @@network = nil
    @@params.each { |param_name, param|
      param = @@defaults[param_name]
    }
  end

  def self.default_param(param_name, type, default)
    if default.is_a?(FalseClass) || default.is_a?(TrueClass)
      assert type.is_a?(Boolean), "default value of param \"#{param_name}\" must be either true or false"
    else
      assert default.is_a?(type), "default value of param \"#{param_name}\" does not match type #{type}"
    end
    @@params[param_name] = default
    @@defaults[param_name] = default
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

class CacheController < NetPort

  def initialize(obj_name, mach_type, caches)
    super(obj_name, mach_type)
    caches.each { |cache|
      cache.controller = self
    }
  end

  def cppClassName()
    "generated:"+@mach_type
  end
end

class Sequencer < IfacePort
end

class L1CacheController < CacheController
  param :sequencer, Sequencer

  def initialize(obj_name, mach_type, caches, sequencer)
    super(obj_name, mach_type, caches)

    sequencer.controller = self
    sequencer.version = version
    self.sequencer= sequencer
  end

#  def argv()
#    vec = super()
#    vec += " sequencer "+@sequencer.obj_name
#  end
end

class DirectoryMemory < LibRubyObject
end
class MemoryControl < LibRubyObject
end

class DirectoryController < NetPort
  @@total_directory_controllers = 0
  param :directory, DirectoryMemory
  param :memory_control, MemoryControl

  def initialize(obj_name, mach_type, directory, memory_control)
    super(obj_name, mach_type)

    directory.controller = self
    directory.version = @@total_directory_controllers
    self.directory = directory
    self.memory_control = memory_control

    @version = @@total_directory_controllers
    @@total_directory_controllers += 1
    buffer_size()
  end

  def cppClassName()
    "generated:"+@mach_type
  end

end

class DMASequencer < IfacePort
end

class DMAController < NetPort
  @@total_dma_controllers = 0
  param :dma_sequencer, DMASequencer
  param :version, Integer

  def initialize(obj_name, mach_type, dma_sequencer)
    super(obj_name, mach_type)
    dma_sequencer.controller = self
    dma_sequencer.version = @@total_dma_controllers
    self.dma_sequencer = dma_sequencer

    self.version = @@total_dma_controllers
    @@total_dma_controllers += 1
  end

end

class Cache < LibRubyObject
  param :size_kb, Integer
  param :latency, Integer
  param :controller, NetPort
  def initialize(obj_name, size_kb, latency)
    super(obj_name)
    self.size_kb = size_kb
    self.latency = latency
    # controller must be set manually by the configuration script
    # because there is a cyclic dependence
  end

end

class SetAssociativeCache < Cache
  param :assoc, Integer
  param :replacement_policy, String

  # latency can be either an integer, a float, or the string "auto"
  #  when an integer, it represents the number of cycles for a hit
  #  when a float, it represents the cache access time in ns
  #  when set to "auto", libruby will attempt to find a realistic latency by running CACTI
  def initialize(obj_name, size_kb, latency, assoc, replacement_policy)
    super(obj_name, size_kb, latency)
    self.assoc = assoc
    self.replacement_policy = replacement_policy
  end

  def calculateLatency()
    if self.latency == "auto"
      cacti_args = Array.new()
      cacti_args << (self.size_kb*1024) <<  RubySystem.block_size_bytes << self.assoc
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
        self.latency = latency_cycles
      }
    elsif self.latency.is_a?(Float)
      clk_period_ns = 1e9 * (1.0 / (RubySystem.freq_mhz * 1e6))
      latency_cycles = (self.latency / clk_period_ns).ceil
      self.latency = latency_cycles
    elsif ! self.latency.is_a?(Integer)
      raise Exception
    end
  end

  def cppClassName()
    "SetAssociativeCache"
  end
end

class DirectoryMemory < LibRubyObject
  param :size_mb, Integer
  param :controller, NetPort
  param :version, Integer

  @@total_size_mb = 0

  def initialize(obj_name, size_mb)
    super(obj_name)
    self.size_mb = size_mb
    @@total_size_mb += size_mb
  end

  def cppClassName()
    "DirectoryMemory"
  end

  def self.memorySizeMB()
    @@total_size_mb
  end
end

class MemoryControl < LibRubyObject
  def initialize(obj_name)
    super(obj_name)
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
  param :controller, NetPort
  param :version, Integer

  def initialize(obj_name)
    super(obj_name)
  end

  def cppClassName()
    "DMASequencer"
  end

  def bochsConnType()
    return "dma"+self.version.to_s
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
    topo.network= self
    self.topology = topo
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

class GarnetNetwork < Network
  def initialize(name, topo)
    super(name, topo)
  end
end

class GarnetFixedPipeline < GarnetNetwork
  def initialize(name, net_ports)
    super(name, net_ports)
  end

  def cppClassName()
    "GarnetNetwork_d"
  end
end

class GarnetFlexiblePipeline < GarnetNetwork
  def initialize(name, net_ports)
    super(name, net_ports)
  end

  def cppClassName()
    "GarnetNetwork"
  end
end

require "defaults.rb"
