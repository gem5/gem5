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

# Note: This isn't going to work because MSI caches won't be built. Need to
# think more about this. Maybe we should have another parameter to
# gem5_verify_config...

config_path = joinpath(config.base_dir, "configs", "learning_gem5", "part3")
ref_path = joinpath(getcwd(), "ref")

gem5_verify_config(
    name="simple_ruby_test",
    verifiers=(verifier.MatchStdoutNoPerf(joinpath(ref_path, "threads")),),
    config=joinpath(config_path, "simple_ruby.py"),
    config_args=[],
    protocol="MSI",
    # Currently only x86 has the threads test
    valid_isas=(constants.all_compiled_tag,),
    # dynamically linked
    valid_hosts=(constants.x86_tag,),
    length=constants.long_tag,
)

gem5_verify_config(
    name="ruby_test_test",
    verifiers=(verifier.MatchStdout(joinpath(ref_path, "test")),),
    config=joinpath(config_path, "ruby_test.py"),
    config_args=[],
    protocol="MSI",
    # Currently only x86 has the threads test
    valid_isas=(constants.all_compiled_tag,),
    length=constants.long_tag,
)
