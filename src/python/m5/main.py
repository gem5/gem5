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

import code
import datetime
import os
import socket
import sys

__all__ = [ 'options', 'arguments', 'main' ]

usage="%prog [gem5 options] script.py [script options]"
version="%prog 2.0"
brief_copyright=\
    "gem5 is copyrighted software; use the --copyright option for details."

def parse_options():
    import config
    from options import OptionParser

    options = OptionParser(usage=usage, version=version,
                           description=brief_copyright)
    option = options.add_option
    group = options.set_group

    # Help options
    option('-B', "--build-info", action="store_true", default=False,
        help="Show build information")
    option('-C', "--copyright", action="store_true", default=False,
        help="Show full copyright information")
    option('-R', "--readme", action="store_true", default=False,
        help="Show the readme")

    # Options for configuring the base simulator
    option('-d', "--outdir", metavar="DIR", default="m5out",
        help="Set the output directory to DIR [Default: %default]")
    option('-r', "--redirect-stdout", action="store_true", default=False,
        help="Redirect stdout (& stderr, without -e) to file")
    option('-e', "--redirect-stderr", action="store_true", default=False,
        help="Redirect stderr to file")
    option("--stdout-file", metavar="FILE", default="simout",
        help="Filename for -r redirection [Default: %default]")
    option("--stderr-file", metavar="FILE", default="simerr",
        help="Filename for -e redirection [Default: %default]")
    option('-i', "--interactive", action="store_true", default=False,
        help="Invoke the interactive interpreter after running the script")
    option("--pdb", action="store_true", default=False,
        help="Invoke the python debugger before running the script")
    option('-p', "--path", metavar="PATH[:PATH]", action='append', split=':',
        help="Prepend PATH to the system path when invoking the script")
    option('-q', "--quiet", action="count", default=0,
        help="Reduce verbosity")
    option('-v', "--verbose", action="count", default=0,
        help="Increase verbosity")

    # Statistics options
    group("Statistics Options")
    option("--stats-file", metavar="FILE", default="stats.txt",
        help="Sets the output file for statistics [Default: %default]")

    # Configuration Options
    group("Configuration Options")
    option("--dump-config", metavar="FILE", default="config.ini",
        help="Dump configuration output file [Default: %default]")
    option("--json-config", metavar="FILE", default="config.json",
        help="Create JSON output of the configuration [Default: %default]")
    option("--dot-config", metavar="FILE", default="config.dot",
        help="Create DOT & pdf outputs of the configuration [Default: %default]")
    option("--dot-dvfs-config", metavar="FILE", default=None,
        help="Create DOT & pdf outputs of the DVFS configuration" + \
             " [Default: %default]")

    # Debugging options
    group("Debugging Options")
    option("--debug-break", metavar="TICK[,TICK]", action='append', split=',',
        help="Create breakpoint(s) at TICK(s) " \
             "(kills process if no debugger attached)")
    option("--debug-help", action='store_true',
        help="Print help on debug flags")
    option("--debug-flags", metavar="FLAG[,FLAG]", action='append', split=',',
        help="Sets the flags for debug output (-FLAG disables a flag)")
    option("--debug-start", metavar="TICK", type='int',
        help="Start debug output at TICK")
    option("--debug-end", metavar="TICK", type='int',
        help="End debug output at TICK")
    option("--debug-file", metavar="FILE", default="cout",
        help="Sets the output file for debug [Default: %default]")
    option("--debug-ignore", metavar="EXPR", action='append', split=':',
        help="Ignore EXPR sim objects")
    option("--remote-gdb-port", type='int', default=7000,
        help="Remote gdb base port (set to 0 to disable listening)")

    # Help options
    group("Help Options")
    option("--list-sim-objects", action='store_true', default=False,
        help="List all built-in SimObjects, their params and default values")

    # load the options.py config file to allow people to set their own
    # default options
    options_file = config.get('options.py')
    if options_file:
        scope = { 'options' : options }
        execfile(options_file, scope)

    arguments = options.parse_args()
    return options,arguments

def interact(scope):
    banner = "gem5 Interactive Console"

    ipshell = None
    prompt_in1 = "gem5 \\#> "
    prompt_out = "gem5 \\#: "

    # Is IPython version 0.10 or earlier available?
    try:
        from IPython.Shell import IPShellEmbed
        ipshell = IPShellEmbed(argv=["-prompt_in1", prompt_in1,
                                     "-prompt_out", prompt_out],
                               banner=banner, user_ns=scope)
    except ImportError:
        pass

    # Is IPython version 0.11 or later available?
    if not ipshell:
        try:
            import IPython
            from IPython.config.loader import Config
            from IPython.frontend.terminal.embed import InteractiveShellEmbed

            cfg = Config()
            cfg.PromptManager.in_template = prompt_in1
            cfg.PromptManager.out_template = prompt_out
            ipshell = InteractiveShellEmbed(config=cfg, user_ns=scope,
                                            banner1=banner)
        except ImportError:
            pass

    if ipshell:
        ipshell()
    else:
        # Use the Python shell in the standard library if IPython
        # isn't available.
        code.InteractiveConsole(scope).interact(banner)

def main(*args):
    import m5

    import core
    import debug
    import defines
    import event
    import info
    import stats
    import trace

    from util import fatal

    if len(args) == 0:
        options, arguments = parse_options()
    elif len(args) == 2:
        options, arguments = args
    else:
        raise TypeError, "main() takes 0 or 2 arguments (%d given)" % len(args)

    m5.options = options

    def check_tracing():
        if defines.TRACING_ON:
            return

        fatal("Tracing is not enabled.  Compile with TRACING_ON")

    # Set the main event queue for the main thread.
    event.mainq = event.getEventQueue(0)
    event.setEventQueue(event.mainq)

    if not os.path.isdir(options.outdir):
        os.makedirs(options.outdir)

    # These filenames are used only if the redirect_std* options are set
    stdout_file = os.path.join(options.outdir, options.stdout_file)
    stderr_file = os.path.join(options.outdir, options.stderr_file)

    # Print redirection notices here before doing any redirection
    if options.redirect_stdout and not options.redirect_stderr:
        print "Redirecting stdout and stderr to", stdout_file
    else:
        if options.redirect_stdout:
            print "Redirecting stdout to", stdout_file
        if options.redirect_stderr:
            print "Redirecting stderr to", stderr_file

    # Now redirect stdout/stderr as desired
    if options.redirect_stdout:
        redir_fd = os.open(stdout_file, os. O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(redir_fd, sys.stdout.fileno())
        if not options.redirect_stderr:
            os.dup2(redir_fd, sys.stderr.fileno())

    if options.redirect_stderr:
        redir_fd = os.open(stderr_file, os. O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(redir_fd, sys.stderr.fileno())

    done = False

    if options.build_info:
        done = True
        print 'Build information:'
        print
        print 'compiled %s' % defines.compileDate;
        print 'build options:'
        keys = defines.buildEnv.keys()
        keys.sort()
        for key in keys:
            val = defines.buildEnv[key]
            print '    %s = %s' % (key, val)
        print

    if options.copyright:
        done = True
        print info.COPYING
        print

    if options.readme:
        done = True
        print 'Readme:'
        print
        print info.README
        print

    if options.debug_help:
        done = True
        check_tracing()
        debug.help()

    if options.list_sim_objects:
        import SimObject
        done = True
        print "SimObjects:"
        objects = SimObject.allClasses.keys()
        objects.sort()
        for name in objects:
            obj = SimObject.allClasses[name]
            print "    %s" % obj
            params = obj._params.keys()
            params.sort()
            for pname in params:
                param = obj._params[pname]
                default = getattr(param, 'default', '')
                print "        %s" % pname
                if default:
                    print "            default: %s" % default
                print "            desc: %s" % param.desc
                print
            print

    if done:
        sys.exit(0)

    # setting verbose and quiet at the same time doesn't make sense
    if options.verbose > 0 and options.quiet > 0:
        options.usage(2)

    verbose = options.verbose - options.quiet
    if verbose >= 0:
        print "gem5 Simulator System.  http://gem5.org"
        print brief_copyright
        print

        print "gem5 compiled %s" % defines.compileDate;

        print "gem5 started %s" % \
            datetime.datetime.now().strftime("%b %e %Y %X")
        print "gem5 executing on %s, pid %d" % \
            (socket.gethostname(), os.getpid())

        # in Python 3 pipes.quote() is moved to shlex.quote()
        import pipes
        print "command line:", " ".join(map(pipes.quote, sys.argv))
        print

    # check to make sure we can find the listed script
    if not arguments or not os.path.isfile(arguments[0]):
        if arguments and not os.path.isfile(arguments[0]):
            print "Script %s not found" % arguments[0]

        options.usage(2)

    # tell C++ about output directory
    core.setOutputDir(options.outdir)

    # update the system path with elements from the -p option
    sys.path[0:0] = options.path

    # set stats options
    stats.initText(options.stats_file)

    # set debugging options
    debug.setRemoteGDBPort(options.remote_gdb_port)
    for when in options.debug_break:
        debug.schedBreak(int(when))

    if options.debug_flags:
        check_tracing()

        on_flags = []
        off_flags = []
        for flag in options.debug_flags:
            off = False
            if flag.startswith('-'):
                flag = flag[1:]
                off = True

            if flag not in debug.flags:
                print >>sys.stderr, "invalid debug flag '%s'" % flag
                sys.exit(1)

            if off:
                debug.flags[flag].disable()
            else:
                debug.flags[flag].enable()

    if options.debug_start:
        check_tracing()
        e = event.create(trace.enable, event.Event.Debug_Enable_Pri)
        event.mainq.schedule(e, options.debug_start)
    else:
        trace.enable()

    if options.debug_end:
        check_tracing()
        e = event.create(trace.disable, event.Event.Debug_Enable_Pri)
        event.mainq.schedule(e, options.debug_end)

    trace.output(options.debug_file)

    for ignore in options.debug_ignore:
        check_tracing()
        trace.ignore(ignore)

    sys.argv = arguments
    sys.path = [ os.path.dirname(sys.argv[0]) ] + sys.path

    filename = sys.argv[0]
    filedata = file(filename, 'r').read()
    filecode = compile(filedata, filename, 'exec')
    scope = { '__file__' : filename,
              '__name__' : '__m5_main__' }

    # we want readline if we're doing anything interactive
    if options.interactive or options.pdb:
        exec "import readline" in scope

    # if pdb was requested, execfile the thing under pdb, otherwise,
    # just do the execfile normally
    if options.pdb:
        import pdb
        import traceback

        pdb = pdb.Pdb()
        try:
            pdb.run(filecode, scope)
        except SystemExit:
            print "The program exited via sys.exit(). Exit status: ",
            print sys.exc_info()[1]
        except:
            traceback.print_exc()
            print "Uncaught exception. Entering post mortem debugging"
            t = sys.exc_info()[2]
            while t.tb_next is not None:
                t = t.tb_next
                pdb.interaction(t.tb_frame,t)
    else:
        exec filecode in scope

    # once the script is done
    if options.interactive:
        interact(scope)

if __name__ == '__main__':
    from pprint import pprint

    options, arguments = parse_options()

    print 'opts:'
    pprint(options, indent=4)
    print

    print 'args:'
    pprint(arguments, indent=4)
