# Copyright (c) 2012 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
#
# Authors: Thomas Grass
#          Andreas Hansson
#          Sascha Bischoff

from m5.params import *
from m5.proxy import *
from MemObject import MemObject

# The traffic generator is a master module that generates stimuli for
# the memory system, based on a collection of simple behaviours that
# are either probabilistic or based on traces. It can be used stand
# alone for creating test cases for interconnect and memory
# controllers, or function as a black-box replacement for system
# components that are not yet modelled in detail, e.g. a video engine
# or baseband subsystem in an SoC.
#
# The traffic generator has a single master port that is used to send
# requests, independent of the specific behaviour. The behaviour of
# the traffic generator is specified in a configuration file, and this
# file describes a state transition graph where each state is a
# specific generator behaviour. Examples include idling, generating
# linear address sequences, random sequences and replay of captured
# traces. By describing these behaviours as states, it is straight
# forward to create very complex behaviours, simply by arranging them
# in graphs. The graph transitions can also be annotated with
# probabilities, effectively making it a Markov Chain.
class TrafficGen(MemObject):
    type = 'TrafficGen'
    cxx_header = "cpu/testers/traffic_gen/traffic_gen.hh"

    # Port used for sending requests and receiving responses
    port = MasterPort("Master port")

    # Config file to parse for the state descriptions
    config_file = Param.String("Configuration file describing the behaviour")

    # System used to determine the mode of the memory system
    system = Param.System(Parent.any, "System this generator is part of")

    # Should requests respond to back-pressure or not, if true, the
    # rate of the traffic generator will be slowed down if requests
    # are not immediately accepted
    elastic_req = Param.Bool(False,
                             "Slow down requests in case of backpressure")
