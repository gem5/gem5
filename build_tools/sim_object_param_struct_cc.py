# Copyright 2021 Google, Inc.
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

import argparse
import importlib
import os.path
import sys

import importer

from code_formatter import code_formatter

parser = argparse.ArgumentParser()
parser.add_argument('modpath', help='module the simobject belongs to')
parser.add_argument('param_cc', help='parameter cc file to generate')
parser.add_argument('use_python',
        help='whether python is enabled in gem5 (True or False)')

args = parser.parse_args()

use_python = args.use_python.lower()
if use_python == 'true':
    use_python = True
elif use_python == 'false':
    use_python = False
else:
    print(f'Unrecognized "use_python" value {use_python}', file=sys.stderr)
    sys.exit(1)

basename = os.path.basename(args.param_cc)
no_ext = os.path.splitext(basename)[0]
sim_object_name = '_'.join(no_ext.split('_')[1:])

importer.install()
module = importlib.import_module(args.modpath)
sim_object = getattr(module, sim_object_name)

code = code_formatter()
sim_object.params_create_decl(code, use_python)
code.write(args.param_cc)
