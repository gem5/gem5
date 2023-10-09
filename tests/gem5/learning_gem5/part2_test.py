# Copyright (c) 2019 The Regents of the University of California.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from testlib import *

config_path = joinpath(config.base_dir, "configs", "learning_gem5", "part2")
ref_path = joinpath(getcwd(), "ref")
get_verifier = lambda file: verifier.MatchStdout(joinpath(ref_path, file))

gem5_verify_config(
    name="run_simple_test",
    verifiers=(get_verifier("simple"),),
    config=joinpath(config_path, "run_simple.py"),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
)

gem5_verify_config(
    name="hello_goodbye_test",
    verifiers=(get_verifier("hello_goodbye"),),
    config=joinpath(config_path, "hello_goodbye.py"),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
)

gem5_verify_config(
    name="simple_memobj_test",
    verifiers=(verifier.MatchStdoutNoPerf(joinpath(ref_path, "hello")),),
    config=joinpath(config_path, "simple_memobj.py"),
    config_args=[],
    # note: by default the above script uses x86
    valid_isas=(constants.all_compiled_tag,),
)

gem5_verify_config(
    name="simple_cache_test",
    verifiers=(verifier.MatchStdoutNoPerf(joinpath(ref_path, "hello")),),
    config=joinpath(config_path, "simple_cache.py"),
    config_args=[],
    # note: by default the above script uses x86
    valid_isas=(constants.all_compiled_tag,),
)

# Note: for simple memobj and simple cache I want to use the traffic generator
# as well as the scripts above.
