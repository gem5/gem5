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
    'sc_attribute.cpp',
    'sc_cthread_process.cpp',
    'sc_event.cpp',
    'sc_except.cpp',
    'sc_join.cpp',
    'sc_main.cpp',
    'sc_main_main.cpp',
    'sc_method_process.cpp',
    'sc_module.cpp',
    'sc_module_name.cpp',
    'sc_module_registry.cpp',
    'sc_name_gen.cpp',
    'sc_object.cpp',
    'sc_object_manager.cpp',
    'sc_phase_callback_registry.cpp',
    'sc_process.cpp',
    'sc_reset.cpp',
    'sc_sensitive.cpp',
    'sc_simcontext.cpp',
    'sc_spawn_options.cpp',
    'sc_thread_process.cpp',
    'sc_time.cpp',
    'sc_ver.cpp',
    'sc_wait.cpp',
    'sc_wait_cthread.cpp',
)

coroutine_lib = systemc['COROUTINE_LIB']
if coroutine_lib == 'qt':
    SystemCSource('sc_cor_qt.cpp')
elif coroutine_lib == 'pthreads':
    systemc.Append(CXXFLAGS=['-pthread'])
    systemc.Append(CFLAGS=['-pthread'])
    SystemCSource('sc_cor_pthread.cpp')
elif coroutine_lib == 'fiber':
    SystemCSource('sc_cor_fiber.cpp')
else:
    print('Unrecognized threading implementation \'%s\'' % coroutine_lib)
    Exit(1)
