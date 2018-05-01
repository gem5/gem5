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

from __future__ import print_function
from __future__ import absolute_import

# Configure the TLB hierarchy
# Places which would probably need to be modified if you
# want a different hierarchy are specified by a <Modify here .. >'
# comment
import m5
from m5.objects import *

def TLB_constructor(level):

    constructor_call = "X86GPUTLB(size = options.L%(level)dTLBentries, \
            assoc = options.L%(level)dTLBassoc, \
            hitLatency = options.L%(level)dAccessLatency,\
            missLatency2 = options.L%(level)dMissLatency,\
            maxOutstandingReqs = options.L%(level)dMaxOutstandingReqs,\
            accessDistance = options.L%(level)dAccessDistanceStat,\
            clk_domain = SrcClockDomain(\
                clock = options.gpu_clock,\
                voltage_domain = VoltageDomain(\
                    voltage = options.gpu_voltage)))" % locals()
    return constructor_call

def Coalescer_constructor(level):

    constructor_call = "TLBCoalescer(probesPerCycle = \
                options.L%(level)dProbesPerCycle, \
                coalescingWindow = options.L%(level)dCoalescingWindow,\
                disableCoalescing = options.L%(level)dDisableCoalescing,\
                clk_domain = SrcClockDomain(\
                    clock = options.gpu_clock,\
                    voltage_domain = VoltageDomain(\
                        voltage = options.gpu_voltage)))" % locals()
    return constructor_call

def create_TLB_Coalescer(options, my_level, my_index, tlb_name,
    coalescer_name):
    # arguments: options, TLB level, number of private structures for this
    # Level, TLB name and  Coalescer name
    for i in range(my_index):
        tlb_name.append(eval(TLB_constructor(my_level)))
        coalescer_name.append(eval(Coalescer_constructor(my_level)))

def config_tlb_hierarchy(options, system, shader_idx):
    n_cu = options.cu_per_sa * options.sa_per_complex * \
           options.num_gpu_complexes

    if options.TLB_config == "perLane":
        num_TLBs = 64 * n_cu
    elif options.TLB_config == "mono":
        num_TLBs = 1
    elif options.TLB_config == "perCU":
        num_TLBs = n_cu
    elif options.TLB_config == "2CU":
        num_TLBs = n_cu >> 1
    else:
        print("Bad option for TLB Configuration.")
        sys.exit(1)

    #-------------------------------------------------------------------------
    # A visual representation of the TLB hierarchy
    # for ease of configuration
    # < Modify here the width and the number of levels if you want a different
    # configuration >
    # width is the number of TLBs of the given type (i.e., D-TLB, I-TLB etc)
    # for this level
    L1 = [{'name': 'sqc', 'width': options.num_sqc, 'TLBarray': [],
           'CoalescerArray': []},
          {'name': 'scalar', 'width' : options.num_scalar_cache,
           'TLBarray': [], 'CoalescerArray': []},
          {'name': 'l1', 'width': num_TLBs, 'TLBarray': [],
           'CoalescerArray': []}]

    L2 = [{'name': 'l2', 'width': 1, 'TLBarray': [], 'CoalescerArray': []}]
    L3 = [{'name': 'l3', 'width': 1, 'TLBarray': [], 'CoalescerArray': []}]

    TLB_hierarchy = [L1, L2, L3]

    #-------------------------------------------------------------------------
    # Create the hiearchy
    # Call the appropriate constructors and add objects to the system

    for i in range(len(TLB_hierarchy)):
        hierarchy_level = TLB_hierarchy[i]
        level = i+1
        for TLB_type in hierarchy_level:
            TLB_index = TLB_type['width']
            TLB_array = TLB_type['TLBarray']
            Coalescer_array = TLB_type['CoalescerArray']
            # If the sim calls for a fixed L1 TLB size across CUs,
            # override the TLB entries option
            if options.tot_L1TLB_size:
                options.L1TLBentries = options.tot_L1TLB_size / num_TLBs
                if options.L1TLBassoc > options.L1TLBentries:
                    options.L1TLBassoc = options.L1TLBentries
            # call the constructors for the TLB and the Coalescer
            create_TLB_Coalescer(options, level, TLB_index,\
                TLB_array, Coalescer_array)

            system_TLB_name = TLB_type['name'] + '_tlb'
            system_Coalescer_name = TLB_type['name'] + '_coalescer'

            # add the different TLB levels to the system
            # Modify here if you want to make the TLB hierarchy a child of
            # the shader.
            exec('system.%s = TLB_array' % system_TLB_name)
            exec('system.%s = Coalescer_array' % system_Coalescer_name)

    #===========================================================
    # Specify the TLB hierarchy (i.e., port connections)
    # All TLBs but the last level TLB need to have a memSidePort (master)
    #===========================================================

    # Each TLB is connected with its Coalescer through a single port.
    # There is a one-to-one mapping of TLBs to Coalescers at a given level
    # This won't be modified no matter what the hierarchy looks like.
    for i in range(len(TLB_hierarchy)):
        hierarchy_level = TLB_hierarchy[i]
        level = i+1
        for TLB_type in hierarchy_level:
            name = TLB_type['name']
            for index in range(TLB_type['width']):
                exec('system.%s_coalescer[%d].master[0] = \
                        system.%s_tlb[%d].slave[0]' % \
                        (name, index, name, index))

    # Connect the cpuSidePort (slave) of all the coalescers in level 1
    # < Modify here if you want a different configuration >
    for TLB_type in L1:
        name = TLB_type['name']
        num_TLBs = TLB_type['width']
        if name == 'l1':     # L1 D-TLBs
            tlb_per_cu = num_TLBs // n_cu
            for cu_idx in range(n_cu):
                if tlb_per_cu:
                    for tlb in range(tlb_per_cu):
                        exec('system.cpu[%d].CUs[%d].translation_port[%d] = \
                                system.l1_coalescer[%d].slave[%d]' % \
                                (shader_idx, cu_idx, tlb,
                                    cu_idx*tlb_per_cu+tlb, 0))
                else:
                    exec('system.cpu[%d].CUs[%d].translation_port[%d] = \
                            system.l1_coalescer[%d].slave[%d]' % \
                            (shader_idx, cu_idx, tlb_per_cu,
                                cu_idx / (n_cu / num_TLBs),
                                cu_idx % (n_cu / num_TLBs)))
        elif name == 'sqc': # I-TLB
            for index in range(n_cu):
                sqc_tlb_index = index / options.cu_per_sqc
                sqc_tlb_port_id = index % options.cu_per_sqc
                exec('system.cpu[%d].CUs[%d].sqc_tlb_port = \
                        system.sqc_coalescer[%d].slave[%d]' % \
                        (shader_idx, index, sqc_tlb_index, sqc_tlb_port_id))
        elif name == 'scalar': # Scalar D-TLB
            for index in range(n_cu):
                scalar_tlb_index = index / options.cu_per_scalar_cache
                scalar_tlb_port_id = index % options.cu_per_scalar_cache
                exec('system.cpu[%d].CUs[%d].scalar_tlb_port = \
                        system.scalar_coalescer[%d].slave[%d]' % \
                        (shader_idx, index, scalar_tlb_index,
                         scalar_tlb_port_id))

    # Connect the memSidePorts (masters) of all the TLBs with the
    # cpuSidePorts (slaves) of the Coalescers of the next level
    # < Modify here if you want a different configuration >
    # L1 <-> L2
    l2_coalescer_index = 0
    for TLB_type in L1:
        name = TLB_type['name']
        for index in range(TLB_type['width']):
            exec('system.%s_tlb[%d].master[0] = \
                    system.l2_coalescer[0].slave[%d]' % \
                    (name, index, l2_coalescer_index))
            l2_coalescer_index += 1
    # L2 <-> L3
    system.l2_tlb[0].master[0] = system.l3_coalescer[0].slave[0]

    return system
