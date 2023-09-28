# Copyright (c) 2016, 2019 Arm Limited
# All rights reserved.
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

import code
import datetime
import os
import socket
import sys

__all__ = ["options", "arguments", "main"]

usage = "%prog [gem5 options] script.py [script options]"
brief_copyright = (
    "gem5 is copyrighted software; use the --copyright option for details."
)


def _stats_help(option, opt, value, parser):
    import m5

    print("A stat file can either be specified as a URI or a plain")
    print("path. When specified as a path, gem5 uses the default text ")
    print("format.")
    print()
    print("The following stat formats are supported:")
    print()
    m5.stats.printStatVisitorTypes()
    sys.exit(0)


def parse_options():
    from .options import OptionParser

    options = OptionParser(usage=usage, description=brief_copyright)
    option = options.add_option
    group = options.set_group

    listener_modes = ("on", "off", "auto")

    # Help options
    option(
        "-B",
        "--build-info",
        action="store_true",
        default=False,
        help="Show build information",
    )
    option(
        "-C",
        "--copyright",
        action="store_true",
        default=False,
        help="Show full copyright information",
    )
    option(
        "-R",
        "--readme",
        action="store_true",
        default=False,
        help="Show the readme",
    )

    # Options for configuring the base simulator
    option(
        "-d",
        "--outdir",
        metavar="DIR",
        default="m5out",
        help="Set the output directory to DIR [Default: %default]",
    )
    option(
        "-r",
        "--redirect-stdout",
        action="store_true",
        default=False,
        help="Redirect stdout (& stderr, without -e) to file",
    )
    option(
        "-e",
        "--redirect-stderr",
        action="store_true",
        default=False,
        help="Redirect stderr to file",
    )
    option(
        "--silent-redirect",
        action="store_true",
        default=False,
        help="Suppress printing a message when redirecting stdout or stderr",
    )
    option(
        "--stdout-file",
        metavar="FILE",
        default="simout.txt",
        help="Filename for -r redirection [Default: %default]",
    )
    option(
        "--stderr-file",
        metavar="FILE",
        default="simerr.txt",
        help="Filename for -e redirection [Default: %default]",
    )
    option(
        "--listener-mode",
        metavar="{on,off,auto}",
        choices=listener_modes,
        default="auto",
        help="Port (e.g., gdb) listener mode (auto: Enable if running "
        "interactively) [Default: %default]",
    )
    option(
        "--allow-remote-connections",
        action="store_true",
        default=False,
        help="Port listeners will accept connections from anywhere (0.0.0.0). "
        "Default is only localhost.",
    )
    option(
        "-i",
        "--interactive",
        action="store_true",
        default=False,
        help="Invoke the interactive interpreter after running the script",
    )
    option(
        "--pdb",
        action="store_true",
        default=False,
        help="Invoke the python debugger before running the script",
    )
    option(
        "-p",
        "--path",
        metavar="PATH[:PATH]",
        action="append",
        split=":",
        help="Prepend PATH to the system path when invoking the script",
    )
    option("-q", "--quiet", action="count", default=0, help="Reduce verbosity")
    option(
        "-v", "--verbose", action="count", default=0, help="Increase verbosity"
    )

    # To make gem5 mimic python better. After `-c` we should consume all other
    # arguments and add those to argv.
    def collect_args(option, opt_str, value, parser):
        extra_args = parser.rargs[:]
        del parser.rargs[:]
        setattr(parser.values, option.dest, (value, extra_args))

    option(
        "-c",
        type=str,
        help="program passed in as string (terminates option list)",
        default="",
        metavar="cmd",
        action="callback",
        callback=collect_args,
    )

    option(
        "-P",
        action="store_true",
        default=False,
        help="Don't prepend the script directory to the system path. "
        "Mimics Python 3's `-P` option.",
    )

    option(
        "-s",
        action="store_true",
        help="IGNORED, only for compatibility with python. don't"
        "add user site directory to sys.path; also PYTHONNOUSERSITE",
    )

    # Statistics options
    group("Statistics Options")
    option(
        "--stats-file",
        metavar="FILE",
        default="stats.txt",
        help="Sets the output file for statistics [Default: %default]",
    )
    option(
        "--stats-help",
        action="callback",
        callback=_stats_help,
        help="Display documentation for available stat visitors",
    )

    # Configuration Options
    group("Configuration Options")
    option(
        "--dump-config",
        metavar="FILE",
        default="config.ini",
        help="Dump configuration output file [Default: %default]",
    )
    option(
        "--json-config",
        metavar="FILE",
        default="config.json",
        help="Create JSON output of the configuration [Default: %default]",
    )
    option(
        "--dot-config",
        metavar="FILE",
        default="config.dot",
        help="Create DOT & pdf outputs of the configuration [Default: %default]",
    )
    option(
        "--dot-dvfs-config",
        metavar="FILE",
        default=None,
        help="Create DOT & pdf outputs of the DVFS configuration"
        + " [Default: %default]",
    )

    # Debugging options
    group("Debugging Options")
    option(
        "--debug-break",
        metavar="TICK[,TICK]",
        action="append",
        split=",",
        help="Create breakpoint(s) at TICK(s) "
        "(kills process if no debugger attached)",
    )
    option(
        "--debug-help", action="store_true", help="Print help on debug flags"
    )
    option(
        "--debug-flags",
        metavar="FLAG[,FLAG]",
        action="append",
        split=",",
        help="Sets the flags for debug output (-FLAG disables a flag)",
    )
    option(
        "--debug-start",
        metavar="TICK",
        type="int",
        help="Start debug output at TICK",
    )
    option(
        "--debug-end",
        metavar="TICK",
        type="int",
        help="End debug output at TICK",
    )
    option(
        "--debug-file",
        metavar="FILE",
        default="cout",
        help="Sets the output file for debug. Append '.gz' to the name for it"
        " to be compressed automatically [Default: %default]",
    )
    option(
        "--debug-activate",
        metavar="EXPR[,EXPR]",
        action="append",
        split=",",
        help="Activate EXPR sim objects",
    )
    option(
        "--debug-ignore",
        metavar="EXPR",
        action="append",
        split=":",
        help="Ignore EXPR sim objects",
    )
    option(
        "--remote-gdb-port",
        type="int",
        default=7000,
        help="Remote gdb base port (set to 0 to disable listening)",
    )

    # Help options
    group("Help Options")
    option(
        "--list-sim-objects",
        action="store_true",
        default=False,
        help="List all built-in SimObjects, their params and default values",
    )

    arguments = options.parse_args()
    return options, arguments


def interact(scope):
    banner = "gem5 Interactive Console"

    ipshell = None
    prompt_in1 = "gem5 \\#> "
    prompt_out = "gem5 \\#: "

    try:
        import IPython
        from IPython.config.loader import Config
        from IPython.terminal.embed import InteractiveShellEmbed

        cfg = Config()
        cfg.PromptManager.in_template = prompt_in1
        cfg.PromptManager.out_template = prompt_out
        ipshell = InteractiveShellEmbed(
            config=cfg, user_ns=scope, banner1=banner
        )
    except ImportError:
        pass

    if ipshell:
        ipshell()
    else:
        # Use the Python shell in the standard library if IPython
        # isn't available.
        import readline  # if this is imported, then the up arrow works

        code.InteractiveConsole(scope).interact(banner)


def _check_tracing():
    import _m5.core

    from .util import fatal

    if _m5.core.TRACING_ON:
        return

    fatal("Tracing is not enabled.  Compile with TRACING_ON")


def main():
    import m5
    import _m5.core

    from . import core
    from . import debug
    from . import defines
    from . import event
    from . import info
    from . import stats
    from . import trace

    from .util import inform, panic, isInteractive
    from m5.util.terminal_formatter import TerminalFormatter

    options, arguments = parse_options()

    m5.options = options

    # Set the main event queue for the main thread.
    event.mainq = event.getEventQueue(0)
    event.setEventQueue(event.mainq)

    if not os.path.isdir(options.outdir):
        os.makedirs(options.outdir)

    # These filenames are used only if the redirect_std* options are set
    stdout_file = os.path.join(options.outdir, options.stdout_file)
    stderr_file = os.path.join(options.outdir, options.stderr_file)

    if not options.silent_redirect:
        # Print redirection notices here before doing any redirection
        if options.redirect_stdout and not options.redirect_stderr:
            print("Redirecting stdout and stderr to", stdout_file)
        else:
            if options.redirect_stdout:
                print("Redirecting stdout to", stdout_file)
            if options.redirect_stderr:
                print("Redirecting stderr to", stderr_file)

    # Now redirect stdout/stderr as desired
    if options.redirect_stdout:
        redir_fd = os.open(stdout_file, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(redir_fd, sys.stdout.fileno())
        if not options.redirect_stderr:
            os.dup2(redir_fd, sys.stderr.fileno())

    if options.redirect_stderr:
        redir_fd = os.open(stderr_file, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(redir_fd, sys.stderr.fileno())

    done = False

    if options.build_info:
        done = True
        print("Build information:")
        print()
        print(f"gem5 version {defines.gem5Version}")
        print(f"compiled {defines.compileDate}")
        print("build options:")
        keys = list(defines.buildEnv.keys())
        keys.sort()
        for key in keys:
            val = defines.buildEnv[key]
            print(f"    {key} = {val}")
        print()

    if options.copyright:
        done = True
        print(info.COPYING)
        print()

    if options.readme:
        done = True
        print("Readme:")
        print()
        print(info.README)
        print()

    if options.debug_help:
        done = True
        _check_tracing()
        debug.help()

    if options.list_sim_objects:
        from . import SimObject

        done = True
        print("SimObjects:")
        objects = list(SimObject.allClasses.keys())
        objects.sort()
        terminal_formatter = TerminalFormatter()
        for name in objects:
            obj = SimObject.allClasses[name]
            print(terminal_formatter.format_output(str(obj), indent=4))
            params = list(obj._params.keys())
            params.sort()
            for pname in params:
                param = obj._params[pname]
                default = getattr(param, "default", "")
                print(terminal_formatter.format_output(pname, indent=8))
                if default:
                    print(
                        terminal_formatter.format_output(
                            str(default), label="default: ", indent=21
                        )
                    )
                print(
                    terminal_formatter.format_output(
                        param.desc, label="desc: ", indent=21
                    )
                )
                print()
            print()

    if done:
        sys.exit(0)

    # setting verbose and quiet at the same time doesn't make sense
    if options.verbose > 0 and options.quiet > 0:
        options.usage(2)

    verbose = options.verbose - options.quiet
    if verbose >= 0:
        print("gem5 Simulator System.  https://www.gem5.org")
        print(brief_copyright)
        print()

        print(f"gem5 version {_m5.core.gem5Version}")
        print(f"gem5 compiled {_m5.core.compileDate}")

        print(
            f"gem5 started {datetime.datetime.now().strftime('%b %e %Y %X')}"
        )
        print(
            "gem5 executing on %s, pid %d"
            % (socket.gethostname(), os.getpid())
        )

        def quote(arg: str) -> str:
            """Quotes a string for printing in a shell. In addition to Unix,
            this is designed to handle the problematic Windows cases where
            'shlex.quote' doesn't work"""

            if os.name == "nt" and os.sep == "\\":
                # If a Windows machine, we manually quote the string.
                arg = arg.replace('"', '\\"')
                if re.search("\s", args):
                    # We quote args which have whitespace.
                    arg = '"' + arg + '"'
                return arg
            import shlex

            return shlex.quote(arg)

        print("command line:", " ".join(map(quote, sys.argv)))
        print()

    # check to make sure we can find the listed script
    if not options.c and (not arguments or not os.path.isfile(arguments[0])):
        if arguments and not os.path.isfile(arguments[0]):
            print(f"Script {arguments[0]} not found")

        options.usage(2)

    # tell C++ about output directory
    core.setOutputDir(options.outdir)

    # update the system path with elements from the -p option
    sys.path[0:0] = options.path

    # set stats options
    stats.addStatVisitor(options.stats_file)

    # Disable listeners unless running interactively or explicitly
    # enabled
    if options.listener_mode == "off":
        m5.disableAllListeners()
    elif options.listener_mode == "auto":
        if not isInteractive():
            inform("Standard input is not a terminal, disabling listeners.")
            m5.disableAllListeners()
    elif options.listener_mode == "on":
        pass
    else:
        panic(f"Unhandled listener mode: {options.listener_mode}")

    if not options.allow_remote_connections:
        m5.listenersLoopbackOnly()

    for when in options.debug_break:
        debug.schedBreak(int(when))

    if options.debug_flags:
        _check_tracing()

        on_flags = []
        off_flags = []
        for flag in options.debug_flags:
            off = False
            if flag.startswith("-"):
                flag = flag[1:]
                off = True

            if flag not in debug.flags:
                print(f"invalid debug flag '{flag}'", file=sys.stderr)
                sys.exit(1)

            if off:
                debug.flags[flag].disable()
            else:
                debug.flags[flag].enable()

    if options.debug_start:
        _check_tracing()
        e = event.create(trace.enable, event.Event.Debug_Enable_Pri)
        event.mainq.schedule(e, options.debug_start)
    else:
        trace.enable()

    if options.debug_end:
        _check_tracing()
        e = event.create(trace.disable, event.Event.Debug_Enable_Pri)
        event.mainq.schedule(e, options.debug_end)

    trace.output(options.debug_file)

    for activate in options.debug_activate:
        _check_tracing()
        trace.activate(activate)

    for ignore in options.debug_ignore:
        _check_tracing()
        trace.ignore(ignore)

    sys.argv = arguments

    if options.c:
        filedata = options.c[0]
        filecode = compile(filedata, "<string>", "exec")
        sys.argv = ["-c"] + options.c[1]
        scope = {"__name__": "__m5_main__"}
    else:
        # If `-P` was used (`options.P == true`), don't prepend the script
        # directory to the `sys.path`. This mimics Python 3's `-P` option
        # (https://docs.python.org/3/using/cmdline.html#cmdoption-P).
        if not options.P:
            sys.path = [os.path.dirname(sys.argv[0])] + sys.path
        filename = sys.argv[0]
        filedata = open(filename, "r").read()
        filecode = compile(filedata, filename, "exec")
        scope = {"__file__": filename, "__name__": "__m5_main__"}

    # if pdb was requested, execfile the thing under pdb, otherwise,
    # just do the execfile normally
    if options.pdb:
        import pdb
        import traceback

        pdb = pdb.Pdb()
        try:
            pdb.run(filecode, scope)
        except SystemExit:
            print("The program exited via sys.exit(). Exit status: ", end=" ")
            print(sys.exc_info()[1])
        except:
            traceback.print_exc()
            print("Uncaught exception. Entering post mortem debugging")
            t = sys.exc_info()[2]
            while t.tb_next is not None:
                t = t.tb_next
                pdb.interaction(t.tb_frame, t)
    else:
        exec(filecode, scope)

    # once the script is done
    if options.interactive:
        interact(scope)
