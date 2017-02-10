#!/usr/bin/env python2

# Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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

"""
SYNOPSIS

    ./regression/verify_output.py <McPAT output>

DESCRIPTION

    Verify the output from McPAT. In particular, ensure that the values in the
    file sum up hierarchically.

AUTHORS

    Joel Hestness <hestness@cs.wisc.edu> (while interning at AMD)
    Yasuko Eckert <yasuko.eckert@amd.com>

"""

import os
import sys
import optparse
import re

root = None
curr_node = None

optionsparser = optparse.OptionParser(
        formatter = optparse.TitledHelpFormatter(),
        usage = globals()['__doc__'])
optionsparser.add_option(
        "-v", "--verbose", action = "store_true", default = False,
        help = "verbose output")
(options, args) = optionsparser.parse_args()

def warning(msg):
    global options
    if options.verbose:
        print "WARNING: %s" %(msg)

def toNumber(value):
    try:
        to_return = float(value)
    except:
        warning("Value, %s, is not a number" % value)
        to_return = value

    return to_return

def withinTolerance(reference, calculated, tolerance = 0.001):
    if tolerance > 1:
        warning("Tolernance is too large: %s" % tolerance)
    upper_bound = reference * (1 + tolerance)
    lower_bound = reference * (1 - tolerance)
    return calculated <= upper_bound and calculated >= lower_bound

class Component:
    def __init__(self):
        self.parent = None
        self.name = None
        self.area = None
        self.peak_dynamic_power = None
        self.subthreshold_leakage = None
        self.gate_leakage = None
        self.runtime_dynamic_power = None
        self.runtime_dynamic_energy = None
        self.total_runtime_energy = None
        self.children = []
        self.hierarchy_level = None

    def print_data(self):
        print "%s:" % self.name
        print "  Area = %s" % self.area
        print "  Peak Dynamic Power = %s" % self.peak_dynamic_power
        print "  Subthreshold Leakage = %s" % self.subthreshold_leakage
        print "  Gate Leakage = %s" % self.gate_leakage
        print "  Runtime Dynamic Power = %s" % self.runtime_dynamic_power
        print "  Runtime Dynamic Energy = %s" % self.runtime_dynamic_energy
        print "  Total Runtime Energy = %s" % self.total_runtime_energy

    def set_name_and_level(self, name_string):
        self.name = name_string.lstrip().rstrip(":")
        self.hierarchy_level = (len(re.match(r"\s*", name_string).group()) - 2) / 4

    def verify_values(self):
        if len(self.children) == 0:
            return
        temp_node = Component()
        temp_node.area = 0
        temp_node.peak_dynamic_power = 0
        temp_node.subthreshold_leakage = 0
        temp_node.gate_leakage = 0
        temp_node.runtime_dynamic_power = 0
        temp_node.runtime_dynamic_energy = 0
        temp_node.total_runtime_energy = 0
        for child in self.children:
            if child != self:
                temp_node.area += child.area
                temp_node.peak_dynamic_power += child.peak_dynamic_power
                temp_node.subthreshold_leakage += child.subthreshold_leakage
                temp_node.gate_leakage += child.gate_leakage
                temp_node.runtime_dynamic_power += child.runtime_dynamic_power
                temp_node.runtime_dynamic_energy += child.runtime_dynamic_energy
                temp_node.total_runtime_energy += child.total_runtime_energy
                child.verify_values()

        if not withinTolerance(self.area, temp_node.area):
            print "WRONG: %s.area = %s != %s" % \
                    (self.name, self.area, temp_node.area)

        if not withinTolerance(
                self.peak_dynamic_power, temp_node.peak_dynamic_power):
            print "WRONG: %s.peak_dynamic_power = %s != %s" % \
                    (self.name, self.peak_dynamic_power,
                     temp_node.peak_dynamic_power)

        if not withinTolerance(
                self.subthreshold_leakage, temp_node.subthreshold_leakage):
            print "WRONG: %s.subthreshold_leakage = %s != %s" % \
                    (self.name, self.subthreshold_leakage,
                     temp_node.subthreshold_leakage)

        if not withinTolerance(self.gate_leakage, temp_node.gate_leakage):
            print "WRONG: %s.gate_leakage = %s != %s" % \
                    (self.name, self.gate_leakage, temp_node.gate_leakage)

        if not withinTolerance(
                self.runtime_dynamic_power, temp_node.runtime_dynamic_power):
            print "WRONG: %s.runtime_dynamic_power = %s != %s" % \
                    (self.name, self.runtime_dynamic_power,
                     temp_node.runtime_dynamic_power)

        if not withinTolerance(
                self.runtime_dynamic_energy, temp_node.runtime_dynamic_energy):
            print "WRONG: %s.runtime_dynamic_energy = %s != %s" % \
                    (self.name, self.runtime_dynamic_energy,
                     temp_node.runtime_dynamic_energy)

        if not withinTolerance(
                self.total_runtime_energy, temp_node.total_runtime_energy):
            print "WRONG: %s.total_runtime_energy = %s != %s" % \
                    (self.name, self.total_runtime_energy,
                     temp_node.total_runtime_energy)

if len(args) < 1:
    print "ERROR: Must specify a McPAT output file to verify"
    exit(0)

# check params
mcpat_output = args[0];
if not os.path.exists(mcpat_output):
    print "ERROR: Output file does not exist: %s" % mcpat_output
    exit(0)

output_file_handle = open(mcpat_output, 'r')
for line in output_file_handle:
    line = line.rstrip()
    if ":" in line:
        # Start a new component
        new_node = Component()
        if root is None:
            root = new_node
            curr_node = new_node
        else:
            if ((curr_node.area is None) or
                    (curr_node.peak_dynamic_power is None) or
                    (curr_node.subthreshold_leakage is None) or
                    (curr_node.gate_leakage is None) or
                    (curr_node.runtime_dynamic_power is None) or
                    (curr_node.runtime_dynamic_energy is None) or
                    (curr_node.total_runtime_energy is None)):
                print "ERROR: Some value is not specified for %s" % curr_node.name
                curr_node.print_data()
                exit(0)

        new_node.set_name_and_level(line)
        while (
                (new_node.hierarchy_level <= curr_node.hierarchy_level) and
                not curr_node is root):
            curr_node = curr_node.parent
        new_node.parent = curr_node
        curr_node.children.append(new_node)
        curr_node = new_node

    elif line is not "":
        tokens = line.split()
        if "Area" in line:
            curr_node.area = toNumber(tokens[2])
        elif "Peak Dynamic Power" in line:
            curr_node.peak_dynamic_power = toNumber(tokens[4])
        elif "Peak Dynamic" in line:
            curr_node.peak_dynamic_power = toNumber(tokens[3])
        elif "Subthreshold Leakage Power" in line:
            curr_node.subthreshold_leakage = toNumber(tokens[4])
        elif "Subthreshold Leakage" in line:
            curr_node.subthreshold_leakage = toNumber(tokens[3])
        elif "Gate Leakage Power" in line:
            curr_node.gate_leakage = toNumber(tokens[4])
        elif "Gate Leakage" in line:
            curr_node.gate_leakage = toNumber(tokens[3])
        elif "Runtime Dynamic Power" in line:
            curr_node.runtime_dynamic_power = toNumber(tokens[4])
        elif "Runtime Dynamic Energy" in line:
            curr_node.runtime_dynamic_energy = toNumber(tokens[4])
        elif "Total Runtime Energy" in line:
            curr_node.total_runtime_energy = toNumber(tokens[4])
        else:
            warning("ERROR: Line not matched: %s" % line)

curr_node = root

curr_node.verify_values()
