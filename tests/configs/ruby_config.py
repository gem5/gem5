import os
import subprocess

from os.path import dirname, join as joinpath

import m5
from m5.params import *

def generate(config_file, cores=1, memories=1, memory_size=1024, \
             cache_size=32768, cache_assoc=8, dmas=1,
             ruby_tick='1t', ports_per_cpu=2):
    default = joinpath(dirname(__file__), '../../src/mem/ruby/config')
    ruby_config = os.environ.get('RUBY_CONFIG', default)
    args = [ "ruby", "-I", ruby_config, joinpath(ruby_config, "print_cfg.rb"),
             "-r", joinpath(ruby_config, config_file), "-p", str(cores),
             "-m", str(memories), "-s", str(memory_size), "-C", str(cache_size),
             "-A", str(cache_assoc), "-D", str(dmas)]

    temp_config = joinpath(m5.options.outdir, "ruby.config")
    ret = subprocess.call(args, stdout=file(temp_config, "w"))
    if ret != 0:
        raise RuntimeError, "subprocess failed!"

    return m5.objects.RubyMemory(clock = ruby_tick,
                                 config_file = temp_config,
                                 num_cpus = cores,
                                 range = AddrRange(str(memory_size)+"MB"),
                                 num_dmas = dmas,
                                 ports_per_core = ports_per_cpu)
