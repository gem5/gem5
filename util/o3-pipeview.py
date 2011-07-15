#! /usr/bin/env python

# Copyright (c) 2011 ARM Limited
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
# Authors: Giacomo Gabrielli

# Pipeline activity viewer for the O3 CPU model.

import optparse
import os
import sys


def process_trace(trace, outfile, cycle_time, width, color, timestamps,
                  start_tick, stop_tick, start_sn, stop_sn):
    line = None
    fields = None
    # Skip lines up to region of interest
    if start_tick != 0:
        while True:
            line = trace.readline()
            if not line: return
            fields = line.split(':')
            if fields[0] != 'O3PipeView': continue
            if int(fields[2]) >= start_tick: break
    elif start_sn != 0:
        while True:
            line = trace.readline()
            if not line: return
            fields = line.split(':')
            if fields[0] != 'O3PipeView': continue
            if fields[1] == 'fetch' and int(fields[5]) >= start_sn: break
    else:
        line = trace.readline()
        if not line: return
        fields = line.split(':')
    # Skip lines up to next instruction fetch
    while fields[0] != 'O3PipeView' or fields[1] != 'fetch':
        line = trace.readline()
        if not line: return
        fields = line.split(':')
    # Print header
    outfile.write('// f = fetch, d = decode, n = rename, p = dispatch, '
                  'i = issue, c = complete, r = retire\n\n')
    outfile.write(' ' + 'timeline'.center(width) +
                  '   ' + 'tick'.center(15) +
                  '  ' + 'pc.upc'.center(12) +
                  '  ' + 'disasm'.ljust(25) +
                  '  ' + 'seq_num'.center(15))
    if timestamps:
        outfile.write('timestamps'.center(25))
    outfile.write('\n')
    # Region of interest
    curr_inst = {}
    while True:
        if fields[0] == 'O3PipeView':
            curr_inst[fields[1]] = int(fields[2])
            if fields[1] == 'fetch':
                if ((stop_tick > 0 and int(fields[2]) > stop_tick) or
                    (stop_sn > 0 and int(fields[5]) > stop_sn)):
                    return
                (curr_inst['pc'], curr_inst['upc']) = fields[3:5]
                curr_inst['sn'] = int(fields[5])
                curr_inst['disasm'] = ' '.join(fields[6][:-1].split())
            elif fields[1] == 'retire':
                print_inst(outfile, curr_inst, cycle_time, width, color,
                           timestamps)
        line = trace.readline()
        if not line: return
        fields = line.split(':')


def print_inst(outfile, inst, cycle_time, width, color, timestamps):
    if color:
        from m5.util.terminal import termcap
    else:
        from m5.util.terminal import no_termcap as termcap
    # Pipeline stages
    stages = [{'name': 'fetch',
               'color': termcap.Blue + termcap.Reverse,
               'shorthand': 'f'},
              {'name': 'decode',
               'color': termcap.Yellow + termcap.Reverse,
               'shorthand': 'd'},
              {'name': 'rename',
               'color': termcap.Magenta + termcap.Reverse,
               'shorthand': 'n'},
              {'name': 'dispatch',
               'color': termcap.Green + termcap.Reverse,
               'shorthand': 'p'},
              {'name': 'issue',
               'color': termcap.Red + termcap.Reverse,
               'shorthand': 'i'},
              {'name': 'complete',
               'color': termcap.Cyan + termcap.Reverse,
               'shorthand': 'c'},
              {'name': 'retire',
               'color': termcap.Blue + termcap.Reverse,
               'shorthand': 'r'}]
    # Print
    time_width = width * cycle_time
    base_tick = (inst['fetch'] / time_width) * time_width
    num_lines = ((inst['retire'] - inst['fetch']) / time_width) + 1
    curr_color = termcap.Normal
    for i in range(num_lines):
        start_tick = base_tick + i * time_width
        end_tick = start_tick + time_width
        if num_lines == 1:  # compact form
            end_tick += (inst['fetch'] - base_tick)
        events = []
        for stage_idx in range(len(stages)):
            tick = inst[stages[stage_idx]['name']]
            if tick >= start_tick and tick < end_tick:
                events.append((tick % time_width,
                               stages[stage_idx]['name'],
                               stage_idx))
        events.sort()
        outfile.write('[')
        pos = 0
        if num_lines == 1 and events[0][2] != 0:  # event is not fetch
            curr_color = stages[events[0][2] - 1]['color']
        for event in events:
            if (stages[event[2]]['name'] == 'dispatch' and
                inst['dispatch'] == inst['issue']):
                continue
            outfile.write(curr_color + '.' * ((event[0] / cycle_time) - pos))
            outfile.write(stages[event[2]]['color'] +
                          stages[event[2]]['shorthand'])
            if event[2] != len(stages) - 1:  # event is not retire
                curr_color = stages[event[2]]['color']
            else:
                curr_color = termcap.Normal
            pos = (event[0] / cycle_time) + 1
        outfile.write(curr_color + '.' * (width - pos) + termcap.Normal +
                      ']-(' + str(base_tick + i * time_width).rjust(15) + ') ')
        if i == 0:
            outfile.write('%s.%s  %s [%s]' % (
                    inst['pc'].rjust(10),
                    inst['upc'],
                    inst['disasm'].ljust(25),
                    str(inst['sn']).rjust(15)))
            if timestamps:
                outfile.write('  f=%s, r=%s' % (inst['fetch'], inst['retire']))
            outfile.write('\n')
        else:
            outfile.write('...'.center(12) + '\n')


def validate_range(my_range):
    my_range = [int(i) for i in my_range.split(':')]
    if (len(my_range) != 2 or
        my_range[0] < 0 or
        my_range[1] > 0 and my_range[0] >= my_range[1]):
        return None
    return my_range


def main():
    # Parse options
    usage = ('%prog [OPTION]... TRACE_FILE')
    parser = optparse.OptionParser(usage=usage)
    parser.add_option(
        '-o',
        dest='outfile',
        default=os.path.join(os.getcwd(), 'o3-pipeview.out'),
        help="output file (default: '%default')")
    parser.add_option(
        '-t',
        dest='tick_range',
        default='0:-1',
        help="tick range (default: '%default'; -1 == inf.)")
    parser.add_option(
        '-i',
        dest='inst_range',
        default='0:-1',
        help="instruction range (default: '%default'; -1 == inf.)")
    parser.add_option(
        '-w',
        dest='width',
        type='int', default=80,
        help="timeline width (default: '%default')")
    parser.add_option(
        '--color',
        action='store_true', default=False,
        help="enable colored output (default: '%default')")
    parser.add_option(
        '-c', '--cycle-time',
        type='int', default=1000,
        help="CPU cycle time in ticks (default: '%default')")
    parser.add_option(
        '--timestamps',
        action='store_true', default=False,
        help="print fetch and retire timestamps (default: '%default')")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error('incorrect number of arguments')
        sys.exit(1)
    tick_range = validate_range(options.tick_range)
    if not tick_range:
        parser.error('invalid range')
        sys.exit(1)
    inst_range = validate_range(options.inst_range)
    if not inst_range:
        parser.error('invalid range')
        sys.exit(1)
    # Process trace
    print 'Processing trace... ',
    with open(args[0], 'r') as trace:
        with open(options.outfile, 'w') as out:
            process_trace(trace, out, options.cycle_time, options.width,
                          options.color, options.timestamps,
                          *(tick_range + inst_range))
    print 'done!'


if __name__ == '__main__':
    sys.path.append(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', 'src', 'python'))
    main()
