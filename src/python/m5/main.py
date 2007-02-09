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

import code, optparse, os, socket, sys
from datetime import datetime
from attrdict import attrdict

__all__ = [ 'options', 'arguments', 'main' ]

usage="%prog [m5 options] script.py [script options]"
version="%prog 2.0"
brief_copyright='''
Copyright (c) 2001-2006
The Regents of The University of Michigan
All Rights Reserved
'''

# there's only one option parsing done, so make it global and add some
# helper functions to make it work well.
parser = optparse.OptionParser(usage=usage, version=version,
                               description=brief_copyright,
                               formatter=optparse.TitledHelpFormatter())
parser.disable_interspersed_args()

# current option group
group = None

def set_group(*args, **kwargs):
    '''set the current option group'''
    global group
    if not args and not kwargs:
        group = None
    else:
        group = parser.add_option_group(*args, **kwargs)

class splitter(object):
    def __init__(self, split):
        self.split = split
    def __call__(self, option, opt_str, value, parser):
        getattr(parser.values, option.dest).extend(value.split(self.split))

def add_option(*args, **kwargs):
    '''add an option to the current option group, or global none set'''

    # if action=split, but allows the option arguments
    # themselves to be lists separated by the split variable'''

    if kwargs.get('action', None) == 'append' and 'split' in kwargs:
        split = kwargs.pop('split')
        kwargs['default'] = []
        kwargs['type'] = 'string'
        kwargs['action'] = 'callback'
        kwargs['callback'] = splitter(split)

    if group:
        return group.add_option(*args, **kwargs)

    return parser.add_option(*args, **kwargs)

def bool_option(name, default, help):
    '''add a boolean option called --name and --no-name.
    Display help depending on which is the default'''

    tname = '--%s' % name
    fname = '--no-%s' % name
    dest = name.replace('-', '_')
    if default:
        thelp = optparse.SUPPRESS_HELP
        fhelp = help
    else:
        thelp = help
        fhelp = optparse.SUPPRESS_HELP

    add_option(tname, action="store_true", default=default, help=thelp)
    add_option(fname, action="store_false", dest=dest, help=fhelp)

# Help options
add_option('-A', "--authors", action="store_true", default=False,
    help="Show author information")
add_option('-C', "--copyright", action="store_true", default=False,
    help="Show full copyright information")
add_option('-R', "--readme", action="store_true", default=False,
    help="Show the readme")
add_option('-N', "--release-notes", action="store_true", default=False,
    help="Show the release notes")

# Options for configuring the base simulator
add_option('-d', "--outdir", metavar="DIR", default=".",
    help="Set the output directory to DIR [Default: %default]")
add_option('-i', "--interactive", action="store_true", default=False,
    help="Invoke the interactive interpreter after running the script")
add_option("--pdb", action="store_true", default=False,
    help="Invoke the python debugger before running the script")
add_option('-p', "--path", metavar="PATH[:PATH]", action='append', split=':',
    help="Prepend PATH to the system path when invoking the script")
add_option('-q', "--quiet", action="count", default=0,
    help="Reduce verbosity")
add_option('-v', "--verbose", action="count", default=0,
    help="Increase verbosity")

# Statistics options
set_group("Statistics Options")
add_option("--stats-file", metavar="FILE", default="m5stats.txt",
    help="Sets the output file for statistics [Default: %default]")

# Debugging options
set_group("Debugging Options")
add_option("--debug-break", metavar="TIME[,TIME]", action='append', split=',',
    help="Cycle to create a breakpoint")

# Tracing options
set_group("Trace Options")
add_option("--trace-flags", metavar="FLAG[,FLAG]", action='append', split=',',
    help="Sets the flags for tracing")
add_option("--trace-start", metavar="TIME", default='0s',
    help="Start tracing at TIME (must have units)")
add_option("--trace-file", metavar="FILE", default="cout",
    help="Sets the output file for tracing [Default: %default]")
add_option("--trace-circlebuf", metavar="SIZE", type="int", default=0,
    help="If SIZE is non-zero, turn on the circular buffer with SIZE lines")
add_option("--no-trace-circlebuf", action="store_const", const=0,
    dest='trace_circlebuf', help=optparse.SUPPRESS_HELP)
bool_option("trace-dumponexit", default=False,
    help="Dump trace buffer on exit")
add_option("--trace-ignore", metavar="EXPR", action='append', split=':',
    help="Ignore EXPR sim objects")

# Execution Trace options
set_group("Execution Trace Options")
bool_option("speculative", default=True,
    help="Don't capture speculative instructions")
bool_option("print-cycle", default=True,
    help="Don't print cycle numbers in trace output")
bool_option("print-symbol", default=True,
    help="Disable PC symbols in trace output")
bool_option("print-opclass", default=True,
    help="Don't print op class type in trace output")
bool_option("print-thread", default=True,
    help="Don't print thread number in trace output")
bool_option("print-effaddr", default=True,
    help="Don't print effective address in trace output")
bool_option("print-data", default=True,
    help="Don't print result data in trace output")
bool_option("print-iregs", default=False,
    help="Print fetch sequence numbers in trace output")
bool_option("print-fetch-seq", default=False,
    help="Print fetch sequence numbers in trace output")
bool_option("print-cpseq", default=False,
    help="Print correct path sequence numbers in trace output")
#bool_option("print-reg-delta", default=False,
#    help="Print which registers changed to what in trace output")
bool_option("legion-lock", default=False,
    help="Compare simulator state with Legion simulator every cycle")

options = attrdict()
arguments = []

def usage(exitcode=None):
    parser.print_help()
    if exitcode is not None:
        sys.exit(exitcode)

def parse_args():
    _opts,args = parser.parse_args()
    opts = attrdict(_opts.__dict__)

    # setting verbose and quiet at the same time doesn't make sense
    if opts.verbose > 0 and opts.quiet > 0:
        usage(2)

    # store the verbosity in a single variable.  0 is default,
    # negative numbers represent quiet and positive values indicate verbose
    opts.verbose -= opts.quiet

    del opts.quiet

    options.update(opts)
    arguments.extend(args)
    return opts,args

def main():
    import defines
    import info
    import internal

    parse_args()

    done = False
    if options.copyright:
        done = True
        print info.LICENSE
        print

    if options.authors:
        done = True
        print 'Author information:'
        print
        print info.AUTHORS
        print

    if options.readme:
        done = True
        print 'Readme:'
        print
        print info.README
        print

    if options.release_notes:
        done = True
        print 'Release Notes:'
        print
        print info.RELEASE_NOTES
        print

    if done:
        sys.exit(0)

    if options.verbose >= 0:
        print "M5 Simulator System"
        print brief_copyright
        print
        print "M5 compiled %s" % internal.main.cvar.compileDate;
        print "M5 started %s" % datetime.now().ctime()
        print "M5 executing on %s" % socket.gethostname()
        print "command line:",
        for argv in sys.argv:
            print argv,
        print

    # check to make sure we can find the listed script
    if not arguments or not os.path.isfile(arguments[0]):
        if arguments and not os.path.isfile(arguments[0]):
            print "Script %s not found" % arguments[0]
        usage(2)

    # tell C++ about output directory
    internal.main.setOutputDir(options.outdir)

    # update the system path with elements from the -p option
    sys.path[0:0] = options.path

    import objects

    # set stats options
    objects.Statistics.text_file = options.stats_file

    # set debugging options
    for when in options.debug_break:
        internal.debug.schedBreakCycle(int(when))

    for flag in options.trace_flags:
        internal.trace.setFlag(flag)

    if options.trace_start is not None:
        internal.trace.enabled = False
        def enable_trace():
            internal.event.enabled = True
        internal.event.create(enable_trace, options.trace_start)

    if options.trace_file is not None:
        internal.trace.file(options.trace_file)

    if options.trace_bufsize is not None:
        internal.trace.buffer_size(options.bufsize)

    #if options.trace_dumponexit:
    #    internal.trace.dumpOnExit = True

    for ignore in options.trace_ignore:
        internal.trace.ignore(ignore)

    # set execution trace options
    objects.ExecutionTrace.speculative = options.speculative
    objects.ExecutionTrace.print_cycle = options.print_cycle
    objects.ExecutionTrace.pc_symbol = options.print_symbol
    objects.ExecutionTrace.print_opclass = options.print_opclass
    objects.ExecutionTrace.print_thread = options.print_thread
    objects.ExecutionTrace.print_effaddr = options.print_effaddr
    objects.ExecutionTrace.print_data = options.print_data
    objects.ExecutionTrace.print_iregs = options.print_iregs
    objects.ExecutionTrace.print_fetchseq = options.print_fetch_seq
    objects.ExecutionTrace.print_cpseq = options.print_cpseq
    #objects.ExecutionTrace.print_reg_delta = options.print_reg_delta
    objects.ExecutionTrace.legion_lockstep = options.legion_lock

    sys.argv = arguments
    sys.path = [ os.path.dirname(sys.argv[0]) ] + sys.path

    scope = { '__file__' : sys.argv[0],
              '__name__' : '__m5_main__' }

    # we want readline if we're doing anything interactive
    if options.interactive or options.pdb:
        exec "import readline" in scope

    # if pdb was requested, execfile the thing under pdb, otherwise,
    # just do the execfile normally
    if options.pdb:
        from pdb import Pdb
        debugger = Pdb()
        debugger.run('execfile("%s")' % sys.argv[0], scope)
    else:
        execfile(sys.argv[0], scope)

    # once the script is done
    if options.interactive:
        interact = code.InteractiveConsole(scope)
        interact.interact("M5 Interactive Console")

if __name__ == '__main__':
    from pprint import pprint

    parse_args()

    print 'opts:'
    pprint(options, indent=4)
    print

    print 'args:'
    pprint(arguments, indent=4)
