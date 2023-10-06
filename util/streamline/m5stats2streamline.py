#!/usr/bin/env python3
# Copyright (c) 2012, 2014 ARM Limited
# All rights reserved
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
# Author: Dam Sunwoo
#
# This script converts gem5 output to ARM DS-5 Streamline .apc project file
# (Requires the gem5 runs to be run with ContextSwitchStatsDump enabled and
# some patches applied to target Linux kernel.)
#
# Usage:
# m5stats2streamline.py <stat_config.ini> <gem5 run folder> <dest .apc folder>
#
# <stat_config.ini>: .ini file that describes which stats to be included
#                    in conversion. Sample .ini files can be found in
#                    util/streamline.
#                    NOTE: this is NOT the gem5 config.ini file.
#
# <gem5 run folder>: Path to gem5 run folder (must contain config.ini,
#                    stats.txt[.gz], and system.tasks.txt.)
#
# <dest .apc folder>: Destination .apc folder path
#
# APC project generation based on Gator v17 (DS-5 v5.17)
# Subsequent versions should be backward compatible
import argparse
import gzip
import os
import re
import shutil
import sys
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
import zlib
from configparser import ConfigParser

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="""
        Converts gem5 runs to ARM DS-5 Streamline .apc project file.
        (NOTE: Requires gem5 runs to be run with ContextSwitchStatsDump
        enabled and some patches applied to the target Linux kernel.)

        APC project generation based on Gator v17 (DS-5 v5.17)
        Subsequent versions should be backward compatible
        """,
)

parser.add_argument(
    "stat_config_file",
    metavar="<stat_config.ini>",
    help=".ini file that describes which stats to be included \
                    in conversion. Sample .ini files can be found in \
                    util/streamline. NOTE: this is NOT the gem5 config.ini \
                    file.",
)

parser.add_argument(
    "input_path",
    metavar="<gem5 run folder>",
    help="Path to gem5 run folder (must contain config.ini, \
                    stats.txt[.gz], and system.tasks.txt.)",
)

parser.add_argument(
    "output_path",
    metavar="<dest .apc folder>",
    help="Destination .apc folder path",
)

parser.add_argument(
    "--num-events",
    action="store",
    type=int,
    default=1000000,
    help="Maximum number of scheduling (context switch) \
                    events to be processed. Set to truncate early. \
                    Default=1000000",
)

parser.add_argument(
    "--gzipped-bmp-not-supported",
    action="store_true",
    help="Do not use gzipped .bmp files for visual annotations. \
                    This option is only required when using Streamline versions \
                    older than 5.14",
)

parser.add_argument(
    "--verbose",
    action="store_true",
    help="Enable verbose output",
)

args = parser.parse_args()

if not re.match(r"(.*)\.apc", args.output_path):
    print("ERROR: <dest .apc folder> should end with '.apc'!")
    sys.exit(1)

# gzipped BMP files for visual annotation is supported in Streamline 5.14.
# Setting this to True will significantly compress the .apc binary file that
# includes frame buffer snapshots.
gzipped_bmp_supported = not args.gzipped_bmp_not_supported

ticks_in_ns = -1

# Default max # of events. Increase this for longer runs.
num_events = args.num_events

start_tick = -1
end_tick = -1


# Parse gem5 config.ini file to determine some system configurations.
# Number of CPUs, L2s, etc.
def parseConfig(config_file):
    global num_cpus, num_l2

    print("\n===============================")
    print("Parsing gem5 config.ini file...")
    print(config_file)
    print("===============================\n")
    config = ConfigParser()
    if not config.read(config_file):
        print("ERROR: config file '", config_file, "' not found")
        sys.exit(1)

    if config.has_section("system.cpu"):
        num_cpus = 1
    else:
        num_cpus = 0
        while config.has_section("system.cpu" + str(num_cpus)):
            num_cpus += 1

    if config.has_section("system.l2_cache"):
        num_l2 = 1
    else:
        num_l2 = 0
        while config.has_section("system.l2_cache" + str(num_l2)):
            num_l2 += 1

    print("Num CPUs:", num_cpus)
    print("Num L2s:", num_l2)
    print("")

    return (num_cpus, num_l2)


process_dict = {}
thread_dict = {}

process_list = []

idle_uid = -1
kernel_uid = -1


class Task:
    def __init__(self, uid, pid, tgid, task_name, is_process, tick):
        if pid == 0:  # Idle
            self.uid = 0
        elif pid == -1:  # Kernel
            self.uid = 0
        else:
            self.uid = uid
        self.pid = pid
        self.tgid = tgid
        self.is_process = is_process
        self.task_name = task_name
        self.children = []
        self.tick = tick  # time this task first appeared


class Event:
    def __init__(self, tick, task):
        self.tick = tick
        self.task = task


############################################################
# Types used in APC Protocol
#  - packed32, packed64
#  - int32
#  - string
############################################################


def packed32(x):
    ret = []
    more = True
    while more:
        x = int(x)
        b = x & 0x7F
        x = x >> 7
        if ((x == 0) and ((b & 0x40) == 0)) or (
            (x == -1) and ((b & 0x40) != 0)
        ):
            more = False
        else:
            b = b | 0x80
        ret.append(b)
    return ret


# For historical reasons, 32/64-bit versions of functions are presevered
def packed64(x):
    return packed32(x)


#  variable length packed 4-byte signed value
def unsigned_packed32(x):
    ret = []
    if (x & 0xFFFFFF80) == 0:
        ret.append(x & 0x7F)
    elif (x & 0xFFFFC000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append((x >> 7) & 0x7F)
    elif (x & 0xFFE00000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append((x >> 14) & 0x7F)
    elif (x & 0xF0000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append((x >> 21) & 0x7F)
    else:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append((x >> 28) & 0x0F)
    return ret


#  variable length packed 8-byte signed value
def unsigned_packed64(x):
    ret = []
    if (x & 0xFFFFFFFFFFFFFF80) == 0:
        ret.append(x & 0x7F)
    elif (x & 0xFFFFFFFFFFFFC000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append((x >> 7) & 0x7F)
    elif (x & 0xFFFFFFFFFFE00000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append((x >> 14) & 0x7F)
    elif (x & 0xFFFFFFFFF0000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append((x >> 21) & 0x7F)
    elif (x & 0xFFFFFFF800000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append((x >> 28) & 0x7F)
    elif (x & 0xFFFFFC0000000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append(((x >> 28) | 0x80) & 0xFF)
        ret.append((x >> 35) & 0x7F)
    elif (x & 0xFFFE000000000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append(((x >> 28) | 0x80) & 0xFF)
        ret.append(((x >> 35) | 0x80) & 0xFF)
        ret.append((x >> 42) & 0x7F)
    elif (x & 0xFF00000000000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append(((x >> 28) | 0x80) & 0xFF)
        ret.append(((x >> 35) | 0x80) & 0xFF)
        ret.append(((x >> 42) | 0x80) & 0xFF)
        ret.append((x >> 49) & 0x7F)
    elif (x & 0x8000000000000000) == 0:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append(((x >> 28) | 0x80) & 0xFF)
        ret.append(((x >> 35) | 0x80) & 0xFF)
        ret.append(((x >> 42) | 0x80) & 0xFF)
        ret.append(((x >> 49) | 0x80) & 0xFF)
        ret.append((x >> 56) & 0x7F)
    else:
        ret.append((x | 0x80) & 0xFF)
        ret.append(((x >> 7) | 0x80) & 0xFF)
        ret.append(((x >> 14) | 0x80) & 0xFF)
        ret.append(((x >> 21) | 0x80) & 0xFF)
        ret.append(((x >> 28) | 0x80) & 0xFF)
        ret.append(((x >> 35) | 0x80) & 0xFF)
        ret.append(((x >> 42) | 0x80) & 0xFF)
        ret.append(((x >> 49) | 0x80) & 0xFF)
        ret.append(((x >> 56) | 0x80) & 0xFF)
        ret.append((x >> 63) & 0x7F)
    return ret


# 4-byte signed little endian
def int32(x):
    ret = []
    ret.append(x & 0xFF)
    ret.append((x >> 8) & 0xFF)
    ret.append((x >> 16) & 0xFF)
    ret.append((x >> 24) & 0xFF)
    return ret


# 2-byte signed little endian
def int16(x):
    ret = []
    ret.append(x & 0xFF)
    ret.append((x >> 8) & 0xFF)
    return ret


# a packed32 length followed by the specified number of characters
def stringList(x):
    ret = []
    ret += packed32(len(x))
    for i in x:
        ret.append(i)
    return ret


def utf8StringList(x):
    ret = []
    for i in x:
        ret.append(ord(i))
    return ret


# packed64 time value in nanoseconds relative to the uptime from the
# Summary message.
def timestampList(x):
    ret = packed64(x)
    return ret


############################################################
# Write binary
############################################################


def writeBinary(outfile, binary_list):
    for i in binary_list:
        if isinstance(i, str):
            byteVal = bytes(i, "utf-8")
        elif isinstance(i, int):
            byteVal = bytes([i])
        else:
            byteVal = i
        outfile.write(byteVal)


############################################################
# APC Protocol Frame Types
############################################################


def addFrameHeader(frame_type, body, core):
    ret = []

    if frame_type == "Summary":
        code = 1
    elif frame_type == "Backtrace":
        code = 2
    elif frame_type == "Name":
        code = 3
    elif frame_type == "Counter":
        code = 4
    elif frame_type == "Block Counter":
        code = 5
    elif frame_type == "Annotate":
        code = 6
    elif frame_type == "Sched Trace":
        code = 7
    elif frame_type == "GPU Trace":
        code = 8
    elif frame_type == "Idle":
        code = 9
    else:
        print("ERROR: Unknown frame type:", frame_type)
        sys.exit(1)

    packed_code = packed32(code)

    packed_core = packed32(core)

    length = int32(len(packed_code) + len(packed_core) + len(body))

    ret = length + packed_code + packed_core + body
    return ret


# Summary frame
#  - timestamp: packed64
#  - uptime: packed64
def summaryFrame(timestamp, uptime):
    frame_type = "Summary"
    newline_canary = stringList("1\n2\r\n3\r4\n\r5")
    monotonic_delta = packed64(0)
    end_of_attr = stringList("")
    body = newline_canary + packed64(timestamp) + packed64(uptime)
    body += monotonic_delta + end_of_attr
    ret = addFrameHeader(frame_type, body, 0)
    return ret


# Backtrace frame
#  - not implemented yet
def backtraceFrame():
    pass


# Cookie name message
#  - cookie: packed32
#  - name: string
def cookieNameFrame(cookie, name):
    frame_type = "Name"
    packed_code = packed32(1)
    body = packed_code + packed32(cookie) + stringList(name)
    ret = addFrameHeader(frame_type, body, 0)
    return ret


# Thread name message
#  - timestamp: timestamp
#  - thread id: packed32
#  - name: string
def threadNameFrame(timestamp, thread_id, name):
    frame_type = "Name"
    packed_code = packed32(2)
    body = (
        packed_code
        + timestampList(timestamp)
        + packed32(thread_id)
        + stringList(name)
    )
    ret = addFrameHeader(frame_type, body, 0)
    return ret


# Core name message
#  - name: string
#  - core_id: packed32
#  - cpuid: packed32
def coreNameFrame(name, core_id, cpuid):
    frame_type = "Name"
    packed_code = packed32(3)
    body = packed_code + packed32(core_id) + packed32(cpuid) + stringList(name)
    ret = addFrameHeader(frame_type, body, 0)
    return ret


# IRQ Cookie name message
#  - cookie: packed32
#  - name: string
#  - irq: packed32
def irqCookieNameFrame(cookie, name, irq):
    frame_type = "Name"
    packed_code = packed32(5)
    body = packed_code + packed32(cookie) + stringList(name) + packed32(irq)
    ret = addFrameHeader(frame_type, body, 0)
    return ret


# Counter frame message
#  - timestamp: timestamp
#  - core: packed32
#  - key: packed32
#  - value: packed64
def counterFrame(timestamp, core, key, value):
    frame_type = "Counter"
    body = (
        timestampList(timestamp)
        + packed32(core)
        + packed32(key)
        + packed64(value)
    )
    ret = addFrameHeader(frame_type, body, core)
    return ret


# Block Counter frame message
#  - key: packed32
#  - value: packed64
def blockCounterFrame(core, key, value):
    frame_type = "Block Counter"
    body = packed32(key) + packed64(value)
    ret = addFrameHeader(frame_type, body, core)
    return ret


# Annotate frame messages
#  - core: packed32
#  - tid: packed32
#  - timestamp: timestamp
#  - size: packed32
#  - body
def annotateFrame(core, tid, timestamp, size, userspace_body):
    frame_type = "Annotate"
    body = (
        packed32(core)
        + packed32(tid)
        + timestampList(timestamp)
        + packed32(size)
        + userspace_body
    )
    ret = addFrameHeader(frame_type, body, core)
    return ret


# Scheduler Trace frame messages
# Sched Switch
#  - Code: 1
#  - timestamp: timestamp
#  - pid: packed32
#  - tid: packed32
#  - cookie: packed32
#  - state: packed32
def schedSwitchFrame(core, timestamp, pid, tid, cookie, state):
    frame_type = "Sched Trace"
    body = (
        packed32(1)
        + timestampList(timestamp)
        + packed32(pid)
        + packed32(tid)
        + packed32(cookie)
        + packed32(state)
    )
    ret = addFrameHeader(frame_type, body, core)
    return ret


# Sched Thread Exit
#  - Code: 2
#  - timestamp: timestamp
#  - tid: packed32
def schedThreadExitFrame(core, timestamp, pid, tid, cookie, state):
    frame_type = "Sched Trace"
    body = packed32(2) + timestampList(timestamp) + packed32(tid)
    ret = addFrameHeader(frame_type, body, core)
    return ret


# GPU Trace frame messages
#  - Not implemented yet
def gpuTraceFrame():
    pass


# Idle frame messages
# Enter Idle
#  - code: 1
#  - timestamp: timestamp
#  - core: packed32
def enterIdleFrame(timestamp, core):
    frame_type = "Idle"
    body = packed32(1) + timestampList(timestamp) + packed32(core)
    ret = addFrameHeader(frame_type, body, core)
    return ret


# Exit Idle
#  - code: 2
#  - timestamp: timestamp
#  - core: packed32
def exitIdleFrame(timestamp, core):
    frame_type = "Idle"
    body = packed32(2) + timestampList(timestamp) + packed32(core)
    ret = addFrameHeader(frame_type, body, core)
    return ret


####################################################################
def parseProcessInfo(task_file):
    print("\n===============================")
    print("Parsing Task file...")
    print(task_file)
    print("===============================\n")

    global start_tick, end_tick, num_cpus
    global process_dict, thread_dict, process_list
    global event_list, unified_event_list
    global idle_uid, kernel_uid

    event_list = []
    unified_event_list = []
    for cpu in range(num_cpus):
        event_list.append([])

    uid = 1  # uid 0 is reserved for idle

    # Dummy Tasks for frame buffers and system diagrams
    process = Task(uid, 9999, 9999, "framebuffer", True, 0)
    process_list.append(process)
    uid += 1
    thread = Task(uid, 9999, 9999, "framebuffer", False, 0)
    process.children.append(thread)
    uid += 1
    process = Task(uid, 9998, 9998, "System", True, 0)
    process_list.append(process)
    # if we don't find the real kernel, use this to keep things going
    kernel_uid = uid
    uid += 1
    thread = Task(uid, 9998, 9998, "System", False, 0)
    process.children.append(thread)
    uid += 1

    ext = os.path.splitext(task_file)[1]

    try:
        if ext == ".gz":
            process_file = gzip.open(task_file, "rb")
        else:
            process_file = open(task_file, "rb")
    except:
        print("ERROR opening task file:", task_file)
        print("Make sure context switch task dumping is enabled in gem5.")
        sys.exit(1)

    process_re = re.compile(
        r"tick=(\d+)\s+(\d+)\s+cpu_id=(\d+)\s+"
        + r"next_pid=([-\d]+)\s+next_tgid=([-\d]+)\s+next_task=(.*)",
    )

    task_name_failure_warned = False

    for line in process_file:
        match = re.match(process_re, line.decode())
        if match:
            tick = int(match.group(1))
            if start_tick < 0:
                start_tick = tick
            cpu_id = int(match.group(3))
            pid = int(match.group(4))
            tgid = int(match.group(5))
            task_name = match.group(6)

            if not task_name_failure_warned:
                if task_name == "FailureIn_curTaskName":
                    print("-------------------------------------------------")
                    print("WARNING: Task name not set correctly!")
                    print(
                        "Process/Thread info will not be displayed correctly",
                    )
                    print("Perhaps forgot to apply m5struct.patch to kernel?")
                    print("-------------------------------------------------")
                    task_name_failure_warned = True

            if not tgid in process_dict:
                if tgid == pid:
                    # new task is parent as well
                    if args.verbose:
                        print("new process", uid, pid, tgid, task_name)
                    if tgid == 0:
                        # new process is the "idle" task
                        process = Task(uid, pid, tgid, "idle", True, tick)
                        idle_uid = 0
                    else:
                        process = Task(uid, pid, tgid, task_name, True, tick)
                else:
                    if tgid == 0:
                        process = Task(uid, tgid, tgid, "idle", True, tick)
                        idle_uid = 0
                    else:
                        # parent process name not known yet
                        process = Task(
                            uid,
                            tgid,
                            tgid,
                            "_Unknown_",
                            True,
                            tick,
                        )
                if tgid == -1:  # kernel
                    kernel_uid = 0
                uid += 1
                process_dict[tgid] = process
                process_list.append(process)
            else:
                if tgid == pid:
                    if process_dict[tgid].task_name == "_Unknown_":
                        if args.verbose:
                            print(
                                "new process",
                                process_dict[tgid].uid,
                                pid,
                                tgid,
                                task_name,
                            )
                        process_dict[tgid].task_name = task_name
                    if process_dict[tgid].task_name != task_name and tgid != 0:
                        process_dict[tgid].task_name = task_name

            if not pid in thread_dict:
                if args.verbose:
                    print(
                        "new thread",
                        uid,
                        process_dict[tgid].uid,
                        pid,
                        tgid,
                        task_name,
                    )
                thread = Task(uid, pid, tgid, task_name, False, tick)
                uid += 1
                thread_dict[pid] = thread
                process_dict[tgid].children.append(thread)
            else:
                if thread_dict[pid].task_name != task_name:
                    thread_dict[pid].task_name = task_name

            if args.verbose:
                print(tick, uid, cpu_id, pid, tgid, task_name)

            task = thread_dict[pid]
            event = Event(tick, task)
            event_list[cpu_id].append(event)
            unified_event_list.append(event)

            if len(unified_event_list) == num_events:
                print("Truncating at", num_events, "events!")
                break
    print(f"Found {len(unified_event_list)} events.")

    for process in process_list:
        if process.pid > 9990:  # fix up framebuffer ticks
            process.tick = start_tick
        print(
            process.uid,
            process.pid,
            process.tgid,
            process.task_name,
            str(process.tick),
        )
        for thread in process.children:
            if thread.pid > 9990:
                thread.tick = start_tick
            print(
                "\t",
                thread.uid,
                thread.pid,
                thread.tgid,
                thread.task_name,
                str(thread.tick),
            )

    end_tick = tick

    print("Start tick:", start_tick)
    print("End tick:  ", end_tick)
    print("")

    return


def initOutput(output_path):
    if not os.path.exists(output_path):
        os.mkdir(output_path)


def ticksToNs(tick):
    if ticks_in_ns < 0:
        print("ticks_in_ns not set properly!")
        sys.exit(1)

    return tick / ticks_in_ns


def writeXmlFile(xml, filename):
    f = open(filename, "w")
    txt = ET.tostring(xml)
    f.write(minidom.parseString(txt).toprettyxml())
    f.close()


# StatsEntry that contains individual statistics
class StatsEntry:
    def __init__(self, name, group, group_index, per_cpu, key):
        # Full name of statistics
        self.name = name

        # Streamline group name that statistic will belong to
        self.group = group

        # Index of statistics within group (used to change colors within groups)
        self.group_index = group_index

        # Shorter name with "system" stripped off
        # and symbols converted to alphanumerics
        self.short_name = re.sub(r"system\.", "", name)
        self.short_name = re.sub(":", "_", name)

        # Regex for this stat (string version used to construct union regex)
        self.regex_string = "^" + name + r"\s+([\d\.]+)"
        self.regex = re.compile(
            "^" + name + r"\s+([\d\.e\-]+)\s+# (.*)$",
            re.M,
        )
        self.description = ""

        # Whether this stat is use per CPU or not
        self.per_cpu = per_cpu

        # Key used in .apc protocol (as described in captured.xml)
        self.key = key

        # List of values of stat per timestamp
        self.values = []

        # Whether this stat has been found for the current timestamp
        self.found = False

        # Whether this stat has been found at least once
        # (to suppress too many warnings)
        self.not_found_at_least_once = False

        # Field used to hold ElementTree subelement for this stat
        self.ET_element = None

        # Create per-CPU stat name and regex, etc.
        if self.per_cpu:
            self.per_cpu_regex_string = []
            self.per_cpu_regex = []
            self.per_cpu_name = []
            self.per_cpu_found = []
            for i in range(num_cpus):
                if num_cpus > 1:
                    per_cpu_name = re.sub("#", str(i), self.name)
                else:
                    per_cpu_name = re.sub("#", "", self.name)

                self.per_cpu_name.append(per_cpu_name)
                print("\t", per_cpu_name)

                self.per_cpu_regex_string.append(
                    "^" + per_cpu_name + r"\s+[\d\.]+",
                )
                self.per_cpu_regex.append(
                    re.compile(
                        "^" + per_cpu_name + r"\s+([\d\.e\-]+)\s+# (.*)$",
                        re.M,
                    ),
                )
                self.values.append([])
                self.per_cpu_found.append(False)

    def append_value(self, val, per_cpu_index=None):
        if self.per_cpu:
            self.values[per_cpu_index].append(str(val))
        else:
            self.values.append(str(val))


# Global stats object that contains the list of stats entries
# and other utility functions
class Stats:
    def __init__(self):
        self.stats_list = []
        self.tick_list = []
        self.next_key = 1

    def register(self, name, group, group_index, per_cpu):
        print("registering stat:", name, "group:", group, group_index)
        self.stats_list.append(
            StatsEntry(name, group, group_index, per_cpu, self.next_key),
        )
        self.next_key += 1

    # Union of all stats to accelerate parsing speed
    def createStatsRegex(self):
        regex_strings = []
        print("\nnum entries in stats_list", len(self.stats_list))
        for entry in self.stats_list:
            if entry.per_cpu:
                for i in range(num_cpus):
                    regex_strings.append(entry.per_cpu_regex_string[i])
            else:
                regex_strings.append(entry.regex_string)

        self.regex = re.compile("|".join(regex_strings))


def registerStats(config_file):
    print("===============================")
    print("Parsing stats config.ini file...")
    print(config_file)
    print("===============================")

    config = ConfigParser()
    if not config.read(config_file):
        print("ERROR: config file '", config_file, "' not found!")
        sys.exit(1)

    print("\nRegistering Stats...")

    stats = Stats()

    per_cpu_stat_groups = config.options("PER_CPU_STATS")
    for group in per_cpu_stat_groups:
        i = 0
        per_cpu_stats_list = config.get("PER_CPU_STATS", group).split("\n")
        for item in per_cpu_stats_list:
            if item:
                stats.register(item, group, i, True)
                i += 1

    per_l2_stat_groups = config.options("PER_L2_STATS")
    for group in per_l2_stat_groups:
        i = 0
        per_l2_stats_list = config.get("PER_L2_STATS", group).split("\n")
        for item in per_l2_stats_list:
            if item:
                for l2 in range(num_l2):
                    if num_l2 > 1:
                        name = re.sub("#", str(l2), item)
                    else:
                        name = re.sub("#", "", item)
                    stats.register(name, group, i, False)
                i += 1

    other_stat_groups = config.options("OTHER_STATS")
    for group in other_stat_groups:
        i = 0
        other_stats_list = config.get("OTHER_STATS", group).split("\n")
        for item in other_stats_list:
            if item:
                stats.register(item, group, i, False)
                i += 1

    stats.createStatsRegex()

    return stats


# Parse and read in gem5 stats file
# Streamline counters are organized per CPU
def readGem5Stats(stats, gem5_stats_file):
    print("\n===============================")
    print("Parsing gem5 stats file...")
    print(gem5_stats_file)
    print("===============================\n")
    ext = os.path.splitext(gem5_stats_file)[1]

    window_start_regex = re.compile(
        "^---------- Begin Simulation Statistics ----------",
    )
    window_end_regex = re.compile(
        "^---------- End Simulation Statistics   ----------",
    )
    final_tick_regex = re.compile(r"^final_tick\s+(\d+)")

    global ticks_in_ns
    sim_freq_regex = re.compile(r"^sim_freq\s+(\d+)")
    sim_freq = -1

    try:
        if ext == ".gz":
            f = gzip.open(gem5_stats_file, "r")
        else:
            f = open(gem5_stats_file)
    except:
        print("ERROR opening stats file", gem5_stats_file, "!")
        sys.exit(1)

    stats_not_found_list = stats.stats_list[:]
    window_num = 0

    while True:
        error = False
        try:
            line = f.readline()
        except OSError:
            print("")
            print("WARNING: IO error in stats file")
            print("(gzip stream not closed properly?)...continuing for now")
            error = True
        if not line:
            break

        # Find out how many gem5 ticks in 1ns
        if sim_freq < 0:
            m = sim_freq_regex.match(line)
            if m:
                sim_freq = int(m.group(1))  # ticks in 1 sec
                ticks_in_ns = int(sim_freq / 1e9)
                print(
                    f"Simulation frequency found! 1 tick == {1.0 / sim_freq:e} sec\n",
                )

        # Final tick in gem5 stats: current absolute timestamp
        m = final_tick_regex.match(line)
        if m:
            tick = int(m.group(1))
            if tick > end_tick:
                break
            stats.tick_list.append(tick)

        if window_end_regex.match(line) or error:
            if args.verbose:
                print("new window")
            for stat in stats.stats_list:
                if stat.per_cpu:
                    for i in range(num_cpus):
                        if not stat.per_cpu_found[i]:
                            if not stat.not_found_at_least_once:
                                print(
                                    "WARNING: stat not found in window #",
                                    window_num,
                                    ":",
                                    stat.per_cpu_name[i],
                                )
                                print(
                                    "suppressing further warnings for "
                                    + "this stat",
                                )
                                stat.not_found_at_least_once = True
                            stat.values[i].append(str(0))
                        stat.per_cpu_found[i] = False
                else:
                    if not stat.found:
                        if not stat.not_found_at_least_once:
                            print(
                                "WARNING: stat not found in window #",
                                window_num,
                                ":",
                                stat.name,
                            )
                            print("suppressing further warnings for this stat")
                            stat.not_found_at_least_once = True
                        stat.values.append(str(0))
                    stat.found = False
            stats_not_found_list = stats.stats_list[:]
            window_num += 1
            if error:
                break

        # Do a single regex of the union of all stats first for speed
        if stats.regex.match(line):
            # Then loop through only the stats we haven't seen in this window
            for stat in stats_not_found_list[:]:
                if stat.per_cpu:
                    for i in range(num_cpus):
                        m = stat.per_cpu_regex[i].match(line)
                        if m:
                            if stat.name == "ipc":
                                value = str(int(float(m.group(1)) * 1000))
                            else:
                                value = str(int(float(m.group(1))))
                            if args.verbose:
                                print(stat.per_cpu_name[i], value)
                            stat.values[i].append(value)
                            stat.per_cpu_found[i] = True
                            all_found = True
                            for j in range(num_cpus):
                                if not stat.per_cpu_found[j]:
                                    all_found = False
                            if all_found:
                                stats_not_found_list.remove(stat)
                            if stat.description == "":
                                stat.description = m.group(2)
                else:
                    m = stat.regex.match(line)
                    if m:
                        value = str(int(float(m.group(1))))
                        if args.verbose:
                            print(stat.name, value)
                        stat.values.append(value)
                        stat.found = True
                        stats_not_found_list.remove(stat)
                        if stat.description == "":
                            stat.description = m.group(2)
    f.close()


# Create session.xml file in .apc folder
def doSessionXML(output_path):
    session_file = output_path + "/session.xml"

    xml = ET.Element("session")

    xml.set("version", "1")
    xml.set("call_stack_unwinding", "no")
    xml.set("parse_debug_info", "no")
    xml.set("high_resolution", "yes")
    xml.set("buffer_mode", "streaming")
    xml.set("sample_rate", "low")

    # Setting duration to zero for now. Doesn't affect visualization.
    xml.set("duration", "0")

    xml.set("target_host", "")
    xml.set("target_port", "8080")

    writeXmlFile(xml, session_file)


# Create captured.xml file in .apc folder
def doCapturedXML(output_path, stats):
    captured_file = output_path + "/captured.xml"

    xml = ET.Element("captured")
    xml.set("version", "1")
    xml.set("protocol", "17")
    xml.set("backtrace_processing", "none")

    target = ET.SubElement(xml, "target")
    target.set("name", "gem5")
    target.set("sample_rate", "1000")
    target.set("cores", str(num_cpus))

    counters = ET.SubElement(xml, "counters")
    for stat in stats.stats_list:
        s = ET.SubElement(counters, "counter")
        stat_name = re.sub(r"\.", "_", stat.short_name)
        stat_name = re.sub("#", "", stat_name)
        s.set("title", stat.group)
        s.set("name", stat_name)
        s.set("color", "0x00000000")
        s.set("key", f"0x{stat.key:08x}")
        s.set("type", stat_name)
        s.set("event", "0x00000000")
        if stat.per_cpu:
            s.set("per_cpu", "yes")
        else:
            s.set("per_cpu", "no")
        s.set("display", "")
        s.set("units", "")
        s.set("average_selection", "no")
        s.set("description", stat.description)

    writeXmlFile(xml, captured_file)


# Writes out Streamline cookies (unique IDs per process/thread)
def writeCookiesThreads(blob):
    thread_list = []
    for process in process_list:
        if process.uid > 0:
            print("cookie", process.task_name, process.uid)
            writeBinary(blob, cookieNameFrame(process.uid, process.task_name))

        # pid and tgid need to be positive values -- no longer true?
        for thread in process.children:
            thread_list.append(thread)

    # Threads need to be sorted in timestamp order
    thread_list.sort(key=lambda x: x.tick)
    for thread in thread_list:
        print(
            "thread",
            thread.task_name,
            (ticksToNs(thread.tick)),
            thread.tgid,
            thread.pid,
        )
        writeBinary(
            blob,
            threadNameFrame(
                ticksToNs(thread.tick),
                thread.pid,
                thread.task_name,
            ),
        )


# Writes context switch info as Streamline scheduling events
def writeSchedEvents(blob):
    for cpu in range(num_cpus):
        for event in event_list[cpu]:
            timestamp = ticksToNs(event.tick)
            pid = event.task.tgid
            tid = event.task.pid
            if event.task.tgid in process_dict:
                cookie = process_dict[event.task.tgid].uid
            else:
                cookie = 0

            # State:
            #   0: waiting on other event besides I/O
            #   1: Contention/pre-emption
            #   2: Waiting on I/O
            #   3: Waiting on mutex
            # Hardcoding to 0 for now. Other states not implemented yet.
            state = 0

            if args.verbose:
                print(cpu, timestamp, pid, tid, cookie)

            writeBinary(
                blob,
                schedSwitchFrame(cpu, timestamp, pid, tid, cookie, state),
            )


# Writes selected gem5 statistics as Streamline counters
def writeCounters(blob, stats):
    timestamp_list = []
    for tick in stats.tick_list:
        if tick > end_tick:
            break
        timestamp_list.append(ticksToNs(tick))

    for stat in stats.stats_list:
        if stat.per_cpu:
            stat_length = len(stat.values[0])
        else:
            stat_length = len(stat.values)

    for n in range(len(timestamp_list)):
        for stat in stats.stats_list:
            if stat.per_cpu:
                for i in range(num_cpus):
                    writeBinary(
                        blob,
                        counterFrame(
                            timestamp_list[n],
                            i,
                            stat.key,
                            int(float(stat.values[i][n])),
                        ),
                    )
            else:
                writeBinary(
                    blob,
                    counterFrame(
                        timestamp_list[n],
                        0,
                        stat.key,
                        int(float(stat.values[n])),
                    ),
                )


# Streamline can display LCD frame buffer dumps (gzipped bmp)
# This function converts the frame buffer dumps to the Streamline format
def writeVisualAnnotations(blob, input_path, output_path):
    frame_path = input_path + "/frames_system.vncserver"
    if not os.path.exists(frame_path):
        return

    frame_count = 0
    file_list = os.listdir(frame_path)
    file_list.sort()
    re_fb = re.compile(r"fb\.(\d+)\.(\d+)\.bmp.gz")

    # Use first non-negative pid to tag visual annotations
    annotate_pid = -1
    for e in unified_event_list:
        pid = e.task.pid
        if pid >= 0:
            annotate_pid = pid
            break

    for fn in file_list:
        m = re_fb.match(fn)
        if m:
            seq = m.group(1)
            tick = int(m.group(2))
            if tick > end_tick:
                break
            frame_count += 1

            userspace_body = []
            userspace_body += packed32(0x1C)  # escape code
            userspace_body += packed32(0x04)  # visual code

            text_annotation = "image_" + str(ticksToNs(tick)) + ".bmp.gz"
            userspace_body += int16(len(text_annotation))
            userspace_body += utf8StringList(text_annotation)

            if gzipped_bmp_supported:
                # copy gzipped bmp directly
                bytes_read = open(frame_path + "/" + fn, "rb").read()
            else:
                # copy uncompressed bmp
                bytes_read = gzip.open(frame_path + "/" + fn, "rb").read()

            userspace_body += int32(len(bytes_read))
            userspace_body += bytes_read

            writeBinary(
                blob,
                annotateFrame(
                    0,
                    annotate_pid,
                    ticksToNs(tick),
                    len(userspace_body),
                    userspace_body,
                ),
            )

    print("\nfound", frame_count, "frames for visual annotation.\n")


def createApcProject(input_path, output_path, stats):
    initOutput(output_path)

    blob = open(output_path + "/0000000000", "wb")

    # Summary frame takes current system time and system uptime.
    # Filling in with random values for now.
    writeBinary(blob, summaryFrame(1234, 5678))

    writeCookiesThreads(blob)

    print("writing Events")
    writeSchedEvents(blob)

    print("writing Counters")
    writeCounters(blob, stats)

    print("writing Visual Annotations")
    writeVisualAnnotations(blob, input_path, output_path)

    doSessionXML(output_path)
    doCapturedXML(output_path, stats)

    blob.close()


#######################
# Main Routine

input_path = args.input_path
output_path = args.output_path

####
# Make sure input path exists
####
if not os.path.exists(input_path):
    print(f"ERROR: Input path {input_path} does not exist!")
    sys.exit(1)

####
# Parse gem5 configuration file to find # of CPUs and L2s
####
(num_cpus, num_l2) = parseConfig(input_path + "/config.ini")

####
# Parse task file to find process/thread info
####
parseProcessInfo(input_path + "/system.tasks.txt")

####
# Parse stat config file and register stats
####
stat_config_file = args.stat_config_file
stats = registerStats(stat_config_file)

####
# Parse gem5 stats
####
# Check if both stats.txt and stats.txt.gz exist and warn if both exist
if os.path.exists(input_path + "/stats.txt") and os.path.exists(
    input_path + "/stats.txt.gz",
):
    print(
        "WARNING: Both stats.txt.gz and stats.txt exist. \
            Using stats.txt.gz by default.",
    )

gem5_stats_file = input_path + "/stats.txt.gz"
if not os.path.exists(gem5_stats_file):
    gem5_stats_file = input_path + "/stats.txt"
if not os.path.exists(gem5_stats_file):
    print(f"ERROR: stats.txt[.gz] file does not exist in {input_path}!")
    sys.exit(1)

readGem5Stats(stats, gem5_stats_file)

####
# Create Streamline .apc project folder
####
createApcProject(input_path, output_path, stats)

print("All done!")
