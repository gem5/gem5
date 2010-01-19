
require "util.rb"

class MOESI_hammer_CacheController < L1CacheController
  attr :cache
  def initialize(obj_name, mach_type, icache, dcache, l2cache, sequencer)
    super(obj_name, mach_type, [icache, dcache, l2cache], sequencer)
    @icache = icache
    @dcache = dcache
    @l2cache = l2cache
  end
  def argv()
    vec = super()
    vec += " icache " + @icache.obj_name
    vec += " dcache " + @dcache.obj_name
    vec += " l2cache " + @l2cache.obj_name
    vec += " issue_latency "+issue_latency.to_s
    vec += " cache_response_latency "+cache_response_latency.to_s
  end

end

class MOESI_hammer_DirectoryController < DirectoryController
  def initialize(obj_name, mach_type, directory, memory_control)
    super(obj_name, mach_type, directory, memory_control)
  end
  def argv()
    vec = super()
    vec += " memory_controller_latency "+memory_controller_latency.to_s
  end
end

class MOESI_hammer_DMAController < DMAController
  def initialize(obj_name, mach_type, dma_sequencer)
    super(obj_name, mach_type, dma_sequencer)
  end
  def argv()
    vec = super
    vec += " request_latency "+request_latency.to_s
  end
end
