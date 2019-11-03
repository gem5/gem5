# Copyright (c) 2017, TU Dresden
# Copyright (c) 2017, University of Kaiserslautern
# All rights reserved.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Authors: Christian Menard
#          Matthias Jung

Import('systemc', 'SystemCSource')

SystemCSource(
    'sc_clock.cpp',
    'sc_event_finder.cpp',
    'sc_event_queue.cpp',
    'sc_export.cpp',
    'sc_interface.cpp',
    'sc_mutex.cpp',
    'sc_port.cpp',
    'sc_prim_channel.cpp',
    'sc_semaphore.cpp',
    'sc_signal.cpp',
    'sc_signal_ports.cpp',
    'sc_signal_resolved.cpp',
    'sc_signal_resolved_ports.cpp',
)
