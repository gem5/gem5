# Copyright (c) 2005 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert
#          Steve Reinhardt

import sys, os, time

import __main__

briefCopyright = '''
Copyright (c) 2001-2006
The Regents of The University of Michigan
All Rights Reserved
'''

fullCopyright = '''
Copyright (c) 2001-2006
The Regents of The University of Michigan
All Rights Reserved

Permission is granted to use, copy, create derivative works and
redistribute this software and such derivative works for any purpose,
so long as the copyright notice above, this grant of permission, and
the disclaimer below appear in all copies made; and so long as the
name of The University of Michigan is not used in any advertising or
publicity pertaining to the use or distribution of this software
without specific, written prior authorization.

THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
'''

def sayHello(f):
    print >> f, "M5 Simulator System"
    print >> f, briefCopyright
    print >> f, "M5 compiled on", __main__.compileDate
    hostname = os.environ.get('HOSTNAME')
    if not hostname:
        hostname = os.environ.get('HOST')
    if hostname:
        print >> f, "M5 executing on", hostname
    print >> f, "M5 simulation started", time.ctime()

sayHello(sys.stderr)

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

def m5execfile(f, global_dict):
    # copy current sys.path
    oldpath = sys.path[:]
    # push file's directory onto front of path
    sys.path.insert(0, os.path.abspath(os.path.dirname(f)))
    execfile(f, global_dict)
    # restore original path
    sys.path = oldpath

# Prepend given directory to system module search path.
def AddToPath(path):
    # if it's a relative path and we know what directory the current
    # python script is in, make the path relative to that directory.
    if not os.path.isabs(path) and sys.path[0]:
        path = os.path.join(sys.path[0], path)
    path = os.path.realpath(path)
    # sys.path[0] should always refer to the current script's directory,
    # so place the new dir right after that.
    sys.path.insert(1, path)

# find the m5 compile options: must be specified as a dict in
# __main__.m5_build_env.
import __main__
if not hasattr(__main__, 'm5_build_env'):
    panic("__main__ must define m5_build_env")

# make a SmartDict out of the build options for our local use
import smartdict
build_env = smartdict.SmartDict()
build_env.update(__main__.m5_build_env)

# make a SmartDict out of the OS environment too
env = smartdict.SmartDict()
env.update(os.environ)

# import the main m5 config code
from config import *

# import the built-in object definitions
from objects import *


args_left = sys.argv[1:]
configfile_found = False

while args_left:
    arg = args_left.pop(0)
    if arg.startswith('--'):
        # if arg starts with '--', parse as a special python option
        # of the format --<python var>=<string value>
        try:
            (var, val) = arg.split('=', 1)
        except ValueError:
            panic("Could not parse configuration argument '%s'\n"
                  "Expecting --<variable>=<value>\n" % arg);
        eval("%s = %s" % (var, repr(val)))
    elif arg.startswith('-'):
        # if the arg starts with '-', it should be a simulator option
        # with a format similar to getopt.
        optchar = arg[1]
        if len(arg) > 2:
            args_left.insert(0, arg[2:])
        if optchar == 'd':
            outdir = args_left.pop(0)
        elif optchar == 'h':
            showBriefHelp(sys.stderr)
            sys.exit(1)
        elif optchar == 'E':
            env_str = args_left.pop(0)
            split_result = env_str.split('=', 1)
            var = split_result[0]
            if len(split_result == 2):
                val = split_result[1]
            else:
                val = True
            env[var] = val
        elif optchar == 'I':
            AddToPath(args_left.pop(0))
        elif optchar == 'P':
            eval(args_left.pop(0))
        else:
            showBriefHelp(sys.stderr)
            panic("invalid argument '%s'\n" % arg_str)
    else:
        # In any other case, treat the option as a configuration file
        # name and load it.
        if not arg.endswith('.py'):
            panic("Config file '%s' must end in '.py'\n" % arg)
        configfile_found = True
        m5execfile(arg, globals())


if not configfile_found:
    panic("no configuration file specified!")

if globals().has_key('root') and isinstance(root, Root):
    sys.stdout = file('config.ini', 'w')
    instantiate(root)
else:
    print 'Instantiation skipped: no root object found.'

