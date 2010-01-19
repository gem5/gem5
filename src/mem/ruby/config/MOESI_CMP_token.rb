
require "cfg.rb"
require "util.rb"


class MOESI_CMP_token_L1CacheController < L1CacheController
  attr :icache, :dcache
  attr :num_l2_controllers
  attr :n_tokens
  def initialize(obj_name, mach_type, icache, dcache, sequencer, num_l2_controllers, n_tokens)
    super(obj_name, mach_type, [icache, dcache], sequencer)
    @icache = icache
    @dcache = dcache
    @num_l2_controllers = num_l2_controllers
    @n_tokens = n_tokens
  end
  def argv()
    num_select_bits = log_int(num_l2_controllers)
    num_block_bits = log_int(RubySystem.block_size_bytes)

    l2_select_low_bit = num_block_bits

    vec = super()
    vec += " icache " + @icache.obj_name
    vec += " dcache " + @dcache.obj_name
    vec += " l1_request_latency " + l1_request_latency.to_s
    vec += " l1_response_latency " + l1_response_latency.to_s
    vec += " l2_select_low_bit " + l2_select_low_bit.to_s
    vec += " l2_select_num_bits " + num_select_bits.to_s
    vec += " N_tokens " + n_tokens.to_s
    vec += " retry_threshold " + retry_threshold.to_s
    vec += " fixed_timeout_latency " + fixed_timeout_latency.to_s
    vec += " dynamic_timeout_enabled " + dynamic_timeout_enabled.to_s

    return vec
  end
end

class MOESI_CMP_token_L2CacheController < CacheController
  attr :cache
  attr :n_tokens
  def initialize(obj_name, mach_type, cache, n_tokens)
    super(obj_name, mach_type, [cache])
    @cache = cache
    @n_tokens = n_tokens
  end
  def argv()
    vec = super()
    vec += " cache " + @cache.obj_name
    vec += " l2_request_latency " + l2_request_latency.to_s
    vec += " l2_response_latency " + l2_response_latency.to_s
    vec += " N_tokens " + n_tokens.to_s
    vec += " filtering_enabled " + filtering_enabled.to_s
    return vec
  end
end


class MOESI_CMP_token_DirectoryController < DirectoryController
  attr :num_l2_controllers
  def initialize(obj_name, mach_type, directory, memory_control, num_l2_controllers)
    super(obj_name, mach_type, directory, memory_control)
    @num_l2_controllers = num_l2_controllers
  end
  def argv()
    num_select_bits = log_int(num_l2_controllers)
    num_block_bits = log_int(RubySystem.block_size_bytes)

    l2_select_low_bit = num_block_bits

    vec = super()
    vec += " directory_latency "+directory_latency.to_s
    vec += " l2_select_low_bit " + l2_select_low_bit.to_s
    vec += " l2_select_num_bits " + num_select_bits.to_s
    vec += " distributed_persistent "+distributed_persistent.to_s
    vec += " fixed_timeout_latency " + fixed_timeout_latency.to_s
    return vec
  end

end

class MOESI_CMP_token_DMAController < DMAController
  def initialize(obj_name, mach_type, dma_sequencer)
    super(obj_name, mach_type, dma_sequencer)
  end
  def argv()
    vec = super
    vec += " request_latency "+request_latency.to_s
    vec += " response_latency "+response_latency.to_s
    return vec
  end
end
