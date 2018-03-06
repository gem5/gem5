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

from __future__ import print_function

import os

Import('systemc', 'SystemCSource')

if systemc['COROUTINE_LIB'] == 'qt':
    SystemCSource('qt.c')

    qt_arch = systemc.get('QT_ARCH', None)
    if not qt_arch:
        print('No architecture selected for the QT coroutine library.')
        Exit(1)

    if qt_arch in ('i386', 'iX86_64'):
        SystemCSource(os.path.join('md', qt_arch + '.s'))
    else:
        print('Don\'t know what to do for QT arch %s.' % qt_arch)
        Exit(1)
