
class MI_example_CacheController < L1CacheController
  attr :cache
  def initialize(obj_name, mach_type, cache, sequencer)
    super(obj_name, mach_type, [cache], sequencer)
    @cache = cache
  end
  def argv()
    vec = super()
    vec += " cache " + @cache.obj_name
    vec += " issue_latency "+issue_latency.to_s
    vec += " cache_response_latency "+cache_response_latency.to_s
  end

end

class MI_example_DirectoryController < DirectoryController
  def initialize(obj_name, mach_type, directory, memory_control)
    super(obj_name, mach_type, directory, memory_control)
  end
  def argv()
    vec = super()
    vec += " directory_latency "+directory_latency.to_s
  end
end

class MI_example_DMAController < DMAController
  def initialize(obj_name, mach_type, dma_sequencer)
    super(obj_name, mach_type, dma_sequencer)
  end
  def argv()
    vec = super
    vec += " request_latency "+request_latency.to_s
  end
end
