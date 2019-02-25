# Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#  Authors: Myrto Papadopoulou

from __future__ import print_function
from __future__ import absolute_import

def tlb_options(parser):

    #===================================================================
    # TLB Configuration
    #===================================================================

    parser.add_option("--TLB-config", type="string", default="perCU",
            help="Options are: perCU (default), mono, 2CU, or perLane")

    #===================================================================
    #   L1 TLB Options (D-TLB, I-TLB, Dispatcher-TLB)
    #===================================================================

    parser.add_option("--L1TLBentries", type='int', default="32")
    parser.add_option("--L1TLBassoc", type='int', default="32")
    parser.add_option("--L1AccessLatency", type='int', default="1",
                      help="latency in gpu cycles")
    parser.add_option("--L1MissLatency", type='int', default="750",
                      help="latency (in gpu cycles) of a page walk, "
                      "if this is a last level TLB")
    parser.add_option("--L1MaxOutstandingReqs", type='int', default="64")
    parser.add_option("--L1AccessDistanceStat", action="store_true")
    parser.add_option("--tot-L1TLB-size", type="int", default="0")

    #===================================================================
    #   L2 TLB Options
    #===================================================================

    parser.add_option("--L2TLBentries", type='int', default="4096")
    parser.add_option("--L2TLBassoc", type='int', default="32")
    parser.add_option("--L2AccessLatency", type='int', default="69",
                      help="latency in gpu cycles")
    parser.add_option("--L2MissLatency", type='int', default="750",
                      help="latency (in gpu cycles) of a page walk, "
                      "if this is a last level TLB")
    parser.add_option("--L2MaxOutstandingReqs", type='int', default="64")
    parser.add_option("--L2AccessDistanceStat", action="store_true")

    #===================================================================
    #   L3 TLB Options
    #===================================================================

    parser.add_option("--L3TLBentries", type='int', default="8192")
    parser.add_option("--L3TLBassoc", type='int', default="32")
    parser.add_option("--L3AccessLatency", type='int', default="150",
                      help="latency in gpu cycles")
    parser.add_option("--L3MissLatency", type='int', default="750",
                      help="latency (in gpu cycles) of a page walk")
    parser.add_option("--L3MaxOutstandingReqs", type='int', default="64")
    parser.add_option("--L3AccessDistanceStat", action="store_true")

    #===================================================================
    #   L1 TLBCoalescer Options
    #===================================================================

    parser.add_option("--L1ProbesPerCycle", type='int', default="2")
    parser.add_option("--L1CoalescingWindow", type='int', default="1")
    parser.add_option("--L1DisableCoalescing", action="store_true")

    #===================================================================
    #   L2 TLBCoalescer Options
    #===================================================================

    parser.add_option("--L2ProbesPerCycle", type='int', default="2")
    parser.add_option("--L2CoalescingWindow", type='int', default="1")
    parser.add_option("--L2DisableCoalescing", action="store_true")

    #===================================================================
    #   L3 TLBCoalescer Options
    #===================================================================

    parser.add_option("--L3ProbesPerCycle", type='int', default="2")
    parser.add_option("--L3CoalescingWindow", type='int', default="1")
    parser.add_option("--L3DisableCoalescing", action="store_true")
