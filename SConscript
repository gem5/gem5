# -*- mode:python -*-

# Copyright (c) 2004-2005 The Regents of The University of Michigan
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

import os
import sys
from os.path import isdir

# This file defines how to build a particular configuration of M5
# based on variable settings in the 'env' build environment.

# Import build environment variable from SConstruct.
Import('env')

###################################################
#
# Define needed sources.
#
###################################################

# Base sources used by all configurations.
base_sources = Split('''
	arch/alpha/decoder.cc
        arch/alpha/alpha_o3_exec.cc
	arch/alpha/fast_cpu_exec.cc
	arch/alpha/simple_cpu_exec.cc
	arch/alpha/full_cpu_exec.cc
	arch/alpha/faults.cc
	arch/alpha/isa_traits.cc

	base/circlebuf.cc
	base/copyright.cc
	base/cprintf.cc
        base/embedfile.cc
	base/fast_alloc.cc
	base/fifo_buffer.cc
	base/hostinfo.cc
	base/hybrid_pred.cc
	base/inifile.cc
	base/intmath.cc
	base/match.cc
	base/misc.cc
	base/output.cc
	base/pollevent.cc
	base/range.cc
	base/random.cc
	base/sat_counter.cc
	base/socket.cc
	base/statistics.cc
	base/str.cc
	base/time.cc
	base/trace.cc
	base/traceflags.cc
	base/userinfo.cc
	base/compression/lzss_compression.cc
	base/loader/aout_object.cc
	base/loader/ecoff_object.cc
	base/loader/elf_object.cc
	base/loader/object_file.cc
	base/loader/symtab.cc
	base/stats/events.cc
	base/stats/statdb.cc
	base/stats/visit.cc
	base/stats/text.cc

	cpu/base.cc
        cpu/base_dyn_inst.cc
	cpu/exec_context.cc
	cpu/exetrace.cc
	cpu/pc_event.cc
	cpu/static_inst.cc
        cpu/o3/2bit_local_pred.cc
        cpu/o3/alpha_dyn_inst.cc
        cpu/o3/alpha_cpu.cc
        cpu/o3/alpha_cpu_builder.cc
        cpu/o3/bpred_unit.cc
        cpu/o3/btb.cc
        cpu/o3/commit.cc
        cpu/o3/decode.cc
        cpu/o3/fetch.cc
        cpu/o3/free_list.cc
        cpu/o3/cpu.cc
        cpu/o3/iew.cc
        cpu/o3/inst_queue.cc
        cpu/o3/ldstq.cc
        cpu/o3/mem_dep_unit.cc
        cpu/o3/ras.cc
        cpu/o3/rename.cc
        cpu/o3/rename_map.cc
        cpu/o3/rob.cc
        cpu/o3/sat_counter.cc
        cpu/o3/store_set.cc
        cpu/o3/tournament_pred.cc
	cpu/fast/cpu.cc
        cpu/sampler/sampler.cc
        cpu/simple/cpu.cc
        cpu/trace/reader/mem_trace_reader.cc
        cpu/trace/reader/ibm_reader.cc
        cpu/trace/reader/itx_reader.cc
        cpu/trace/reader/m5_reader.cc
        cpu/trace/opt_cpu.cc
        cpu/trace/trace_cpu.cc

	encumbered/cpu/full/bpred.cc
	encumbered/cpu/full/commit.cc
	encumbered/cpu/full/cpu.cc
	encumbered/cpu/full/create_vector.cc
	encumbered/cpu/full/cv_spec_state.cc
	encumbered/cpu/full/dd_queue.cc
	encumbered/cpu/full/dep_link.cc
	encumbered/cpu/full/dispatch.cc
	encumbered/cpu/full/dyn_inst.cc
	encumbered/cpu/full/execute.cc
	encumbered/cpu/full/fetch.cc
	encumbered/cpu/full/floss_reasons.cc
	encumbered/cpu/full/fu_pool.cc
	encumbered/cpu/full/inst_fifo.cc
	encumbered/cpu/full/instpipe.cc
	encumbered/cpu/full/issue.cc
	encumbered/cpu/full/ls_queue.cc
	encumbered/cpu/full/machine_queue.cc
        encumbered/cpu/full/pc_sample_profile.cc
        encumbered/cpu/full/pipetrace.cc
        encumbered/cpu/full/readyq.cc
        encumbered/cpu/full/reg_info.cc
        encumbered/cpu/full/rob_station.cc
        encumbered/cpu/full/spec_memory.cc
        encumbered/cpu/full/spec_state.cc
        encumbered/cpu/full/storebuffer.cc
        encumbered/cpu/full/writeback.cc
        encumbered/cpu/full/iq/iq_station.cc
        encumbered/cpu/full/iq/iqueue.cc
        encumbered/cpu/full/iq/segmented/chain_info.cc
        encumbered/cpu/full/iq/segmented/chain_wire.cc
        encumbered/cpu/full/iq/segmented/iq_seg.cc
        encumbered/cpu/full/iq/segmented/iq_segmented.cc
        encumbered/cpu/full/iq/segmented/seg_chain.cc
        encumbered/cpu/full/iq/seznec/iq_seznec.cc
        encumbered/cpu/full/iq/standard/iq_standard.cc
	encumbered/mem/functional/main.cc

	mem/base_hier.cc
	mem/base_mem.cc
	mem/hier_params.cc
	mem/mem_cmd.cc
	mem/mem_debug.cc
	mem/mem_req.cc
	mem/memory_interface.cc
	mem/bus/base_interface.cc
	mem/bus/bus.cc
	mem/bus/bus_bridge.cc
	mem/bus/bus_bridge_master.cc
	mem/bus/bus_bridge_slave.cc
	mem/bus/bus_interface.cc
	mem/bus/dma_bus_interface.cc
	mem/bus/dma_interface.cc
	mem/bus/master_interface.cc
	mem/bus/slave_interface.cc
	mem/cache/base_cache.cc
	mem/cache/cache.cc
	mem/cache/cache_builder.cc
	mem/cache/coherence/coherence_protocol.cc
	mem/cache/coherence/uni_coherence.cc
	mem/cache/miss/blocking_buffer.cc
	mem/cache/miss/miss_queue.cc
	mem/cache/miss/mshr.cc
	mem/cache/miss/mshr_queue.cc
        mem/cache/prefetch/base_prefetcher.cc
        mem/cache/prefetch/prefetcher.cc
        mem/cache/prefetch/tagged_prefetcher.cc
	mem/cache/tags/base_tags.cc
	mem/cache/tags/cache_tags.cc
	mem/cache/tags/fa_lru.cc
	mem/cache/tags/iic.cc
	mem/cache/tags/lru.cc
	mem/cache/tags/repl/gen.cc
	mem/cache/tags/repl/repl.cc
	mem/functional/functional.cc
	mem/timing/base_memory.cc
	mem/timing/memory_builder.cc
	mem/timing/simple_mem_bank.cc
        mem/trace/itx_writer.cc
	mem/trace/mem_trace_writer.cc
	mem/trace/m5_writer.cc

        python/pyconfig.cc
        python/embedded_py.cc

	sim/builder.cc
	sim/configfile.cc
	sim/debug.cc
	sim/eventq.cc
	sim/main.cc
	sim/param.cc
	sim/profile.cc
	sim/root.cc
	sim/serialize.cc
	sim/sim_events.cc
	sim/sim_exit.cc
	sim/sim_object.cc
	sim/startup.cc
	sim/stat_context.cc
	sim/stat_control.cc
	sim/trace_context.cc
        ''')

# MySql sources
mysql_sources = Split('''
	base/mysql.cc
	base/stats/mysql.cc
        ''')

# Full-system sources
full_system_sources = Split('''
	arch/alpha/alpha_memory.cc
	arch/alpha/arguments.cc
	arch/alpha/ev5.cc
	arch/alpha/osfpal.cc
	arch/alpha/pseudo_inst.cc
	arch/alpha/vtophys.cc

	base/crc.cc
	base/inet.cc
	base/remote_gdb.cc

	cpu/intr_control.cc

	dev/alpha_console.cc
	dev/baddev.cc
        dev/simconsole.cc
	dev/disk_image.cc
	dev/etherbus.cc
	dev/etherdump.cc
	dev/etherint.cc
	dev/etherlink.cc
	dev/etherpkt.cc
	dev/ethertap.cc
	dev/ide_ctrl.cc
	dev/ide_disk.cc
	dev/io_device.cc
	dev/ns_gige.cc
	dev/pciconfigall.cc
	dev/pcidev.cc
        dev/pcifake.cc
	dev/pktfifo.cc
        dev/platform.cc
	dev/sinic.cc
	dev/simple_disk.cc
	dev/tsunami.cc
	dev/tsunami_cchip.cc
	dev/tsunami_fake.cc
	dev/tsunami_io.cc
	dev/tsunami_pchip.cc
	dev/uart.cc
	dev/uart8250.cc

	kern/kernel_binning.cc
	kern/kernel_stats.cc
	kern/system_events.cc
	kern/freebsd/freebsd_system.cc
        kern/freebsd/freebsd_events.cc
	kern/linux/linux_events.cc
	kern/linux/linux_syscalls.cc
	kern/linux/linux_system.cc
	kern/linux/printk.cc
	kern/tru64/dump_mbuf.cc
	kern/tru64/printf.cc
	kern/tru64/tru64_events.cc
	kern/tru64/tru64_syscalls.cc
	kern/tru64/tru64_system.cc

	mem/functional/memory_control.cc
	mem/functional/physical.cc

	sim/system.cc
        ''')

# turbolaser encumbered sources
turbolaser_sources = Split('''
	encumbered/dev/dma.cc
	encumbered/dev/etherdev.cc
	encumbered/dev/scsi.cc
	encumbered/dev/scsi_ctrl.cc
	encumbered/dev/scsi_disk.cc
	encumbered/dev/scsi_none.cc
	encumbered/dev/tlaser_clock.cc
	encumbered/dev/tlaser_ipi.cc
	encumbered/dev/tlaser_mbox.cc
	encumbered/dev/tlaser_mc146818.cc
	encumbered/dev/tlaser_node.cc
	encumbered/dev/tlaser_pcia.cc
	encumbered/dev/tlaser_pcidev.cc
	encumbered/dev/tlaser_serial.cc
	encumbered/dev/turbolaser.cc
	encumbered/dev/uart8530.cc
        ''')

# Syscall emulation (non-full-system) sources
syscall_emulation_sources = Split('''
	arch/alpha/alpha_common_syscall_emul.cc
	arch/alpha/alpha_linux_process.cc
	arch/alpha/alpha_tru64_process.cc
	cpu/memtest/memtest.cc
	encumbered/eio/eio.cc
	encumbered/eio/exolex.cc
	encumbered/eio/libexo.cc
	sim/process.cc
	sim/syscall_emul.cc
        ''')

targetarch_files = Split('''
        alpha_common_syscall_emul.hh
        alpha_linux_process.hh
        alpha_memory.hh
        alpha_tru64_process.hh
        aout_machdep.h
        arguments.hh
        byte_swap.hh
        ecoff_machdep.h
        ev5.hh
        faults.hh
        isa_fullsys_traits.hh
        isa_traits.hh
        osfpal.hh
        pseudo_inst.hh
        vptr.hh
        vtophys.hh
        ''')

for f in targetarch_files:
    env.Command('targetarch/' + f, 'arch/alpha/' + f,
                '''echo '#include "arch/alpha/%s"' > $TARGET''' % f)


# Set up complete list of sources based on configuration.
sources = base_sources

if env['FULL_SYSTEM']:
    sources += full_system_sources
    if env['ALPHA_TLASER']:
        sources += turbolaser_sources
else:
    sources += syscall_emulation_sources

extra_libraries = []
env.Append(LIBS=['z'])
if isdir('/usr/lib64/mysql') or isdir('/usr/lib/mysql') or \
   isdir('/usr/local/lib/mysql'):
    print 'Compiling with MySQL support!'
    env.Append(LIBPATH=['/usr/lib64/mysql', '/usr/local/lib/mysql/',
                        '/usr/lib/mysql'])
    env.Append(CPPPATH=['/usr/local/include/mysql', '/usr/include/mysql'])
    sources += mysql_sources
    env.Append(CPPDEFINES = 'USE_MYSQL')
    env.Append(CPPDEFINES = 'STATS_BINNING')
    env.Append(LIBS=['mysqlclient'])

###################################################
#
# Special build rules.
#
###################################################

# base/traceflags.{cc,hh} are generated from base/traceflags.py.
# $TARGET.base will expand to "<build-dir>/base/traceflags".
env.Command(Split('base/traceflags.hh base/traceflags.cc'),
            'base/traceflags.py',
            'python $SOURCE $TARGET.base')

# several files are generated from arch/$TARGET_ISA/isa_desc.
env.Command(Split('''arch/alpha/decoder.cc
		     arch/alpha/decoder.hh
                     arch/alpha/alpha_o3_exec.cc
		     arch/alpha/fast_cpu_exec.cc
                     arch/alpha/simple_cpu_exec.cc
                     arch/alpha/full_cpu_exec.cc'''),
            Split('''arch/alpha/isa_desc
		     arch/isa_parser.py'''),
            '$SRCDIR/arch/isa_parser.py $SOURCE $TARGET.dir arch/alpha')


# libelf build is described in its own SConscript file.
# SConscript-local is the per-config build, which just copies some
# header files into a place where they can be found.
SConscript('libelf/SConscript-local', exports = 'env', duplicate=0)
SConscript('python/SConscript', exports = ['env'], duplicate=0)

# This function adds the specified sources to the given build
# environment, and returns a list of all the corresponding SCons
# Object nodes (including an extra one for date.cc).  We explicitly
# add the Object nodes so we can set up special dependencies for
# date.cc.
def make_objs(sources, env):
    objs = [env.Object(s) for s in sources]
    # make date.cc depend on all other objects so it always gets
    # recompiled whenever anything else does
    date_obj = env.Object('base/date.cc')
    env.Depends(date_obj, objs)
    objs.append(date_obj)
    objs.extend(extra_libraries)
    return objs

###################################################
#
# Define binaries.  Each different build type (debug, opt, etc.) gets
# a slightly different build environment.
#
###################################################

# Include file paths are rooted in this directory.  SCons will
# automatically expand '.' to refer to both the source directory and
# the corresponding build directory to pick up generated include
# files.
env.Append(CPPPATH='.')

# Debug binary
debug = env.Copy(OBJSUFFIX='.do')
debug.Append(CCFLAGS=Split('-g -gstabs+ -O0'))
debug.Append(CPPDEFINES='DEBUG')
debug.Program(target = 'm5.debug', source = make_objs(sources, debug))

# Optimized binary
opt = env.Copy()
opt.Append(CCFLAGS=Split('-g -O5'))
opt.Program(target = 'm5.opt', source = make_objs(sources, opt))

# "Fast" binary
fast = env.Copy(OBJSUFFIX='.fo')
fast.Append(CCFLAGS=Split('-O5'))
fast.Append(CPPDEFINES='NDEBUG')
fast.Program(target = 'm5.fast.unstripped', source = make_objs(sources, fast))
fast.Command(target = 'm5.fast', source = 'm5.fast.unstripped',
             action = 'strip $SOURCE -o $TARGET')

# Profiled binary
prof = env.Copy(OBJSUFFIX='.po')
prof.Append(CCFLAGS=Split('-O5 -g -pg'), LINKFLAGS='-pg')
prof.Program(target = 'm5.prof', source = make_objs(sources, prof))
