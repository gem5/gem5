# -*- mode:python -*-

# Copyright (c) 2013, 2015-2020 ARM Limited
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
# Copyright (c) 2011 Advanced Micro Devices, Inc.
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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

###################################################
#
# SCons top-level build description (SConstruct) file.
#
# While in this directory ('gem5'), just type 'scons' to build the default
# configuration (see below), or type 'scons build/<CONFIG>/<binary>'
# to build some other configuration (e.g., 'build/X86/gem5.opt' for
# the optimized X86 version).
#
# You can build gem5 in a different directory as long as there is a
# 'build/<CONFIG>' somewhere along the target path.  The build system
# expects that all configs under the same build directory are being
# built for the same host system.
#
# Examples:
#
#   The following two commands are equivalent.  The '-u' option tells
#   scons to search up the directory tree for this SConstruct file.
#   % cd <path-to-src>/gem5 ; scons build/X86/gem5.debug
#   % cd <path-to-src>/gem5/build/X86; scons -u gem5.debug
#
#   The following two commands are equivalent and demonstrate building
#   in a directory outside of the source tree.  The '-C' option tells
#   scons to chdir to the specified directory to find this SConstruct
#   file.
#   % cd <path-to-src>/gem5 ; scons /local/foo/build/X86/gem5.debug
#   % cd /local/foo/build/X86; scons -C <path-to-src>/gem5 gem5.debug
#
# You can use 'scons -H' to print scons options.  If you're in this
# 'gem5' directory (or use -u or -C to tell scons where to find this
# file), you can use 'scons -h' to print all the gem5-specific build
# options as well.
#
###################################################

# Global Python imports
import atexit
import os
import sys

from os import mkdir, remove, environ
from os.path import abspath, dirname, expanduser
from os.path import isdir, isfile
from os.path import join, split

import logging
logging.basicConfig()

# SCons imports
import SCons
import SCons.Node
import SCons.Node.FS
import SCons.Tool

if getattr(SCons, '__version__', None) in ('3.0.0', '3.0.1'):
    # Monkey patch a fix which appears in version 3.0.2, since we only
    # require version 3.0.0
    def __hash__(self):
        return hash(self.lstr)
    import SCons.Subst
    SCons.Subst.Literal.__hash__ = __hash__


########################################################################
#
# Command line options.
#
########################################################################

linker_options = ('bfd', 'gold', 'lld', 'mold')

AddOption('--no-colors', dest='use_colors', action='store_false',
          help="Don't add color to abbreviated scons output")
AddOption('--with-cxx-config', action='store_true',
          help="Build with support for C++-based configuration")
AddOption('--default',
          help='Override which build_opts file to use for defaults')
AddOption('--ignore-style', action='store_true',
          help='Disable style checking hooks')
AddOption('--linker', action='store', default=None, choices=linker_options,
          help=f'Select which linker to use ({", ".join(linker_options)})')
AddOption('--gold-linker', action='store_const', const='gold', dest='linker',
          help='Use the gold linker. Deprecated: Use --linker=gold')
AddOption('--no-compress-debug', action='store_true',
          help="Don't compress debug info in build files")
AddOption('--with-lto', action='store_true',
          help='Enable Link-Time Optimization')
AddOption('--verbose', action='store_true',
          help='Print full tool command lines')
AddOption('--without-python', action='store_true',
          help='Build without Python configuration support')
AddOption('--without-tcmalloc', action='store_true',
          help='Disable linking against tcmalloc')
AddOption('--with-ubsan', action='store_true',
          help='Build with Undefined Behavior Sanitizer if available')
AddOption('--with-asan', action='store_true',
          help='Build with Address Sanitizer if available')
AddOption('--with-systemc-tests', action='store_true',
          help='Build systemc tests')
AddOption('--install-hooks', action='store_true',
          help='Install revision control hooks non-interactively')
AddOption('--gprof', action='store_true',
          help='Enable support for the gprof profiler')
AddOption('--pprof', action='store_true',
          help='Enable support for the pprof profiler')

# Inject the built_tools directory into the python path.
sys.path[1:1] = [ Dir('#build_tools').abspath ]

# Imports of gem5_scons happen here since it depends on some options which are
# declared above.
from gem5_scons import error, warning, summarize_warnings, parse_build_path
from gem5_scons import TempFileSpawn, EnvDefaults, MakeAction, MakeActionTool
import gem5_scons
from gem5_scons.builders import ConfigFile, AddLocalRPATH, SwitchingHeaders
from gem5_scons.builders import Blob
from gem5_scons.sources import TagImpliesTool
from gem5_scons.util import compareVersions, readCommand

# Disable warnings when targets can be built with multiple environments but
# with the same actions. This can happen intentionally if, for instance, a
# generated source file is used to build object files in different ways in
# different environments, but generating the source file itself is exactly the
# same. This can be re-enabled from the command line if desired.
SetOption('warn', 'no-duplicate-environment')

Export('MakeAction')

########################################################################
#
# Set up the main build environment.
#
########################################################################

main = Environment(tools=[
        'default', 'git', TempFileSpawn, EnvDefaults, MakeActionTool,
        ConfigFile, AddLocalRPATH, SwitchingHeaders, TagImpliesTool, Blob
    ])

main.Tool(SCons.Tool.FindTool(['gcc', 'clang'], main))
main.Tool(SCons.Tool.FindTool(['g++', 'clang++'], main))

Export('main')

from gem5_scons.util import get_termcap
termcap = get_termcap()

# Check that we have a C/C++ compiler
if not ('CC' in main and 'CXX' in main):
    error("No C++ compiler installed (package g++ on Ubuntu and RedHat)")

# Find default configuration & binary.
Default(environ.get('M5_DEFAULT_BINARY', 'build/ARM/gem5.debug'))


########################################################################
#
# Figure out which configurations to set up based on the path(s) of
# the target(s).
#
########################################################################

# helper function: find last occurrence of element in list
def rfind(l, elt, offs = -1):
    for i in range(len(l)+offs, 0, -1):
        if l[i] == elt:
            return i
    raise ValueError("element not found")

# Take a list of paths (or SCons Nodes) and return a list with all
# paths made absolute and ~-expanded.  Paths will be interpreted
# relative to the launch directory unless a different root is provided
def makePathListAbsolute(path_list, root=GetLaunchDir()):
    return [abspath(os.path.join(root, expanduser(str(p))))
            for p in path_list]

# Each target must have 'build' in the interior of the path; the
# directory below this will determine the build parameters.  For
# example, for target 'foo/bar/build/X86/arch/x86/blah.do' we
# recognize that X86 specifies the configuration because it
# follow 'build' in the build path.

# The funky assignment to "[:]" is needed to replace the list contents
# in place rather than reassign the symbol to a new list, which
# doesn't work (obviously!).
BUILD_TARGETS[:] = makePathListAbsolute(BUILD_TARGETS)

# Generate a list of the unique build roots and configs that the
# collected targets reference.
variant_paths = set()
build_root = None
for t in BUILD_TARGETS:
    this_build_root, variant = parse_build_path(t)

    # Make sure all targets use the same build root.
    if not build_root:
        build_root = this_build_root
    elif this_build_root != build_root:
        error("build targets not under same build root\n  %s\n  %s" %
            (build_root, this_build_root))

    # Collect all the variants into a set.
    variant_paths.add(os.path.join('/', build_root, variant))

# Make sure build_root exists (might not if this is the first build there)
if not isdir(build_root):
    mkdir(build_root)
main['BUILDROOT'] = build_root


########################################################################
#
# Set up various paths.
#
########################################################################

base_dir = Dir('#src').abspath
Export('base_dir')

# the ext directory should be on the #includes path
main.Append(CPPPATH=[Dir('ext')])

# Add shared top-level headers
main.Prepend(CPPPATH=Dir('include'))


########################################################################
#
# Set command line options based on the configuration of the host and
# build settings.
#
########################################################################

# Initialize the Link-Time Optimization (LTO) flags
main['LTO_CCFLAGS'] = []
main['LTO_LINKFLAGS'] = []

# According to the readme, tcmalloc works best if the compiler doesn't
# assume that we're using the builtin malloc and friends. These flags
# are compiler-specific, so we need to set them after we detect which
# compiler we're using.
main['TCMALLOC_CCFLAGS'] = []

CXX_version = readCommand([main['CXX'], '--version'], exception=False)

main['GCC'] = CXX_version and CXX_version.find('g++') >= 0
main['CLANG'] = CXX_version and CXX_version.find('clang') >= 0
if main['GCC'] + main['CLANG'] > 1:
    error('Two compilers enabled at once?')


########################################################################
#
# Detect and configure external dependencies.
#
########################################################################

main['USE_PYTHON'] = not GetOption('without_python')

def config_embedded_python(env):
    # Find Python include and library directories for embedding the
    # interpreter. We rely on python-config to resolve the appropriate
    # includes and linker flags. If you want to link in an alternate version
    # of python, override the PYTHON_CONFIG variable.

    python_config = env.Detect(env['PYTHON_CONFIG'])
    if python_config is None:
        error("Can't find a suitable python-config, tried "
              f"{env['PYTHON_CONFIG']}")

    print(f"Info: Using Python config: {python_config}")

    cmd = [python_config, '--ldflags', '--includes']

    # Starting in Python 3.8 the --embed flag is required. Use it if supported.
    with gem5_scons.Configure(env) as conf:
        if conf.TryAction(f'@{python_config} --embed')[0]:
            cmd.append('--embed')

    def flag_filter(env, cmd_output, unique=True):
        # Since this function does not use the `unique` param, one should not
        # pass any value to this param.
        assert(unique==True)
        flags = cmd_output.split()
        prefixes = ('-l', '-L', '-I')
        is_useful = lambda x: any(x.startswith(prefix) for prefix in prefixes)
        useful_flags = list(filter(is_useful, flags))
        env.MergeFlags(' '.join(useful_flags))

    env.ParseConfig(cmd, flag_filter)

    env.Prepend(CPPPATH=Dir('ext/pybind11/include/'))

    with gem5_scons.Configure(env) as conf:
        # verify that this stuff works
        if not conf.CheckHeader('Python.h', '<>'):
            error("Check failed for Python.h header.\n",
                  "Two possible reasons:\n"
                  "1. Python headers are not installed (You can install the "
                  "package python-dev on Ubuntu and RedHat)\n"
                  "2. SCons is using a wrong C compiler. This can happen if "
                  "CC has the wrong value.\n"
                  f"CC = {env['CC']}")
        py_version = conf.CheckPythonLib()
        if not py_version:
            error("Can't find a working Python installation")

    # Found a working Python installation. Check if it meets minimum
    # requirements.
    ver_string = '.'.join(map(str, py_version))
    if py_version[0] < 3 or (py_version[0] == 3 and py_version[1] < 6):
        error('Embedded python library 3.6 or newer required, found '
              f'{ver_string}.')
    elif py_version[0] > 3:
        warning('Embedded python library too new. '
                f'Python 3 expected, found {ver_string}.')


########################################################################
#
# Define build environments for required variants.
#
########################################################################

for variant_path in variant_paths:
    # Make a copy of the build-root environment to use for this config.
    env = main.Clone()
    env['BUILDDIR'] = variant_path

    gem5_build = os.path.join(build_root, variant_path, 'gem5.build')
    env['GEM5BUILD'] = gem5_build
    Execute(Mkdir(gem5_build))

    env.SConsignFile(os.path.join(gem5_build, 'sconsign'))

    # Set up default C++ compiler flags
    if env['GCC'] or env['CLANG']:
        # As gcc and clang share many flags, do the common parts here
        env.Append(CCFLAGS=['-pipe'])
        env.Append(CCFLAGS=['-fno-strict-aliasing'])

        # Enable -Wall and -Wextra and then disable the few warnings that
        # we consistently violate
        env.Append(CCFLAGS=['-Wall', '-Wundef', '-Wextra',
                            '-Wno-sign-compare', '-Wno-unused-parameter'])

        # We always compile using C++17
        env.Append(CXXFLAGS=['-std=c++17'])

        if sys.platform.startswith('freebsd'):
            env.Append(CCFLAGS=['-I/usr/local/include'])
            env.Append(CXXFLAGS=['-I/usr/local/include'])
            # On FreeBSD we need libthr.
            env.Append(LIBS=['thr'])

        with gem5_scons.Configure(env) as conf:
            conf.CheckLinkFlag('-Wl,--as-needed')

        linker = GetOption('linker')
        if linker:
            with gem5_scons.Configure(env) as conf:
                if not conf.CheckLinkFlag(f'-fuse-ld={linker}'):
                    # check mold support for gcc older than 12.1.0
                    if linker == 'mold' and \
                       (env['GCC'] and \
                           compareVersions(env['CXXVERSION'],
                                           "12.1.0") < 0) and \
                       ((isdir('/usr/libexec/mold') and \
                           conf.CheckLinkFlag('-B/usr/libexec/mold')) or \
                       (isdir('/usr/local/libexec/mold') and \
                           conf.CheckLinkFlag('-B/usr/local/libexec/mold'))):
                        pass # support mold
                    else:
                        error(f'Linker "{linker}" is not supported')
                if linker == 'gold' and not GetOption('with_lto'):
                    # Tell the gold linker to use threads. The gold linker
                    # segfaults if both threads and LTO are enabled.
                    conf.CheckLinkFlag('-Wl,--threads')
                    conf.CheckLinkFlag(
                            '-Wl,--thread-count=%d' % GetOption('num_jobs'))

        # Treat warnings as errors but white list some warnings that we
        # want to allow (e.g., deprecation warnings).
        env.Append(CCFLAGS=['-Werror',
                             '-Wno-error=deprecated-declarations',
                             '-Wno-error=deprecated',
                            ])

    else:
        error('\n'.join((
              "Don't know what compiler options to use for your compiler.",
              "compiler: " + env['CXX'],
              "version: " + CXX_version.replace('\n', '<nl>') if
                    CXX_version else 'COMMAND NOT FOUND!',
              "If you're trying to use a compiler other than GCC",
              "or clang, there appears to be something wrong with your",
              "environment.",
              "",
              "If you are trying to use a compiler other than those listed",
              "above you will need to ease fix SConstruct and ",
              "src/SConscript to support that compiler.")))

    if env['GCC']:
        if compareVersions(env['CXXVERSION'], "7") < 0:
            error('gcc version 7 or newer required.\n'
                  'Installed version:', env['CXXVERSION'])

        with gem5_scons.Configure(env) as conf:
            # This warning has a false positive in the systemc in g++ 11.1.
            conf.CheckCxxFlag('-Wno-free-nonheap-object')

        # Add the appropriate Link-Time Optimization (LTO) flags if
        # `--with-lto` is set.
        if GetOption('with_lto'):
            # g++ uses "make" to parallelize LTO. The program can be overriden
            # with the environment variable "MAKE", but we currently make no
            # attempt to plumb that variable through.
            parallelism = ''
            if env.Detect('make'):
                parallelism = '=%d' % GetOption('num_jobs')
            else:
                warning('"make" not found, link time optimization will be '
                        'single threaded.')

            for var in 'LTO_CCFLAGS', 'LTO_LINKFLAGS':
                # Use the same amount of jobs for LTO as scons.
                env[var] = ['-flto%s' % parallelism]

        env.Append(TCMALLOC_CCFLAGS=[
            '-fno-builtin-malloc', '-fno-builtin-calloc',
            '-fno-builtin-realloc', '-fno-builtin-free'])

    elif env['CLANG']:
        if compareVersions(env['CXXVERSION'], "6") < 0:
            error('clang version 6 or newer required.\n'
                  'Installed version:', env['CXXVERSION'])

        # Set the Link-Time Optimization (LTO) flags if enabled.
        if GetOption('with_lto'):
            for var in 'LTO_CCFLAGS', 'LTO_LINKFLAGS':
                env[var] = ['-flto']

        # clang has a few additional warnings that we disable.
        with gem5_scons.Configure(env) as conf:
            conf.CheckCxxFlag('-Wno-c99-designator')
            conf.CheckCxxFlag('-Wno-defaulted-function-deleted')

        env.Append(TCMALLOC_CCFLAGS=['-fno-builtin'])

        # On Mac OS X/Darwin we need to also use libc++ (part of XCode) as
        # opposed to libstdc++, as the later is dated.
        if sys.platform == "darwin":
            env.Append(CXXFLAGS=['-stdlib=libc++'])
            env.Append(LIBS=['c++'])

    # Add sanitizers flags
    sanitizers=[]
    if GetOption('with_ubsan'):
        sanitizers.append('undefined')
    if GetOption('with_asan'):
        # Available for gcc >= 5 or llvm >= 3.1 both a requirement
        # by the build system
        sanitizers.append('address')
        suppressions_file = Dir('util').File('lsan-suppressions').get_abspath()
        suppressions_opt = 'suppressions=%s' % suppressions_file
        suppressions_opts = ':'.join([suppressions_opt,
                                      'print_suppressions=0'])
        env['ENV']['LSAN_OPTIONS'] = suppressions_opts
        print()
        warning('To suppress false positive leaks, set the LSAN_OPTIONS '
                'environment variable to "%s" when running gem5' %
                suppressions_opts)
        warning('LSAN_OPTIONS=%s' % suppressions_opts)
        print()
    if sanitizers:
        sanitizers = ','.join(sanitizers)
        if env['GCC'] or env['CLANG']:
            env.Append(CCFLAGS=['-fsanitize=%s' % sanitizers,
                                 '-fno-omit-frame-pointer'],
                        LINKFLAGS='-fsanitize=%s' % sanitizers)
        else:
            warning("Don't know how to enable %s sanitizer(s) for your "
                    "compiler." % sanitizers)

    if sys.platform == 'cygwin':
        # cygwin has some header file issues...
        env.Append(CCFLAGS=["-Wno-uninitialized"])


    if not GetOption('no_compress_debug'):
        with gem5_scons.Configure(env) as conf:
            if not conf.CheckCxxFlag('-gz'):
                warning("Can't enable object file debug section compression")
            if not conf.CheckLinkFlag('-gz'):
                warning("Can't enable executable debug section compression")

    if env['USE_PYTHON']:
        config_embedded_python(env)
        gem5py_env = env.Clone()
    else:
        gem5py_env = env.Clone()
        config_embedded_python(gem5py_env)

    # Bare minimum environment that only includes python
    gem5py_env.Append(CCFLAGS=['${GEM5PY_CCFLAGS_EXTRA}'])
    gem5py_env.Append(LINKFLAGS=['${GEM5PY_LINKFLAGS_EXTRA}'])

    if GetOption('gprof') and GetOption('pprof'):
        error('Only one type of profiling should be enabled at a time')
    if GetOption('gprof'):
        env.Append(CCFLAGS=['-g', '-pg'], LINKFLAGS=['-pg'])
    if GetOption('pprof'):
        env.Append(CCFLAGS=['-g'],
                LINKFLAGS=['-Wl,--no-as-needed', '-lprofiler',
                    '-Wl,--as-needed'])

    env['HAVE_PKG_CONFIG'] = env.Detect('pkg-config')

    with gem5_scons.Configure(env) as conf:
        # On Solaris you need to use libsocket for socket ops
        if not conf.CheckLibWithHeader(
                [None, 'socket'], 'sys/socket.h', 'C++', 'accept(0,0,0);'):
           error("Can't find library with socket calls (e.g. accept()).")

        if not conf.CheckLibWithHeader('z', 'zlib.h', 'C++','zlibVersion();'):
            error('Did not find needed zlib compression library '
                  'and/or zlib.h header file.\n'
                  'Please install zlib and try again.')

    if not GetOption('without_tcmalloc'):
        with gem5_scons.Configure(env) as conf:
            if conf.CheckLib('tcmalloc'):
                conf.env.Append(CCFLAGS=conf.env['TCMALLOC_CCFLAGS'])
            elif conf.CheckLib('tcmalloc_minimal'):
                conf.env.Append(CCFLAGS=conf.env['TCMALLOC_CCFLAGS'])
            else:
                warning("You can get a 12% performance improvement by "
                        "installing tcmalloc (libgoogle-perftools-dev package "
                        "on Ubuntu or RedHat).")

    if not GetOption('silent'):
        print("Building in", variant_path)

    # variant_dir is the tail component of build path, and is used to
    # determine the build parameters (e.g., 'X86')
    (build_root, variant_dir) = os.path.split(variant_path)

    ####################################################################
    #
    # Read and process SConsopts files. These can add new settings which
    # affect each variant directory independently.
    #
    ####################################################################

    # Register a callback to call after all SConsopts files have been read.
    after_sconsopts_callbacks = []
    def AfterSConsopts(cb):
        after_sconsopts_callbacks.append(cb)
    Export('AfterSConsopts')

    # Sticky variables get saved in the variables file so they persist from
    # one invocation to the next (unless overridden, in which case the new
    # value becomes sticky).
    sticky_vars = Variables(args=ARGUMENTS)
    Export('sticky_vars')

    # EXTRAS is special since it affects what SConsopts need to be read.
    sticky_vars.Add(('EXTRAS', 'Add extra directories to the compilation', ''))

    # Set env variables according to the build directory config.
    sticky_vars.files = []
    # Variables for $BUILD_ROOT/$VARIANT_DIR are stored in
    # $BUILD_ROOT/$VARIANT_DIR/gem5.build/variables

    gem5_build_vars = os.path.join(gem5_build, 'variables')
    build_root_vars = os.path.join(build_root, 'variables', variant_dir)
    current_vars_files = [gem5_build_vars, build_root_vars]
    existing_vars_files = list(filter(isfile, current_vars_files))
    if existing_vars_files:
        sticky_vars.files.extend(existing_vars_files)
        if not GetOption('silent'):
            print('Using saved variables file(s) %s' %
                    ', '.join(existing_vars_files))
    else:
        # Variant specific variables file doesn't exist.

        # Get default build variables from source tree.  Variables are
        # normally determined by name of $VARIANT_DIR, but can be
        # overridden by '--default=' arg on command line.
        default = GetOption('default')
        opts_dir = Dir('#build_opts').abspath
        if default:
            default_vars_files = [
                    gem5_build_vars,
                    build_root_vars,
                    os.path.join(opts_dir, default)
                ]
        else:
            default_vars_files = [os.path.join(opts_dir, variant_dir)]
        existing_default_files = list(filter(isfile, default_vars_files))
        if existing_default_files:
            default_vars_file = existing_default_files[0]
            sticky_vars.files.append(default_vars_file)
            print("Variables file(s) %s not found,\n  using defaults in %s" %
                    (' or '.join(current_vars_files), default_vars_file))
        else:
            error("Cannot find variables file(s) %s or default file(s) %s" %
                    (' or '.join(current_vars_files),
                     ' or '.join(default_vars_files)))
            Exit(1)

    # Apply current settings for EXTRAS to env.
    sticky_vars.Update(env)

    # Parse EXTRAS variable to build list of all directories where we're
    # look for sources etc.  This list is exported as extras_dir_list.
    if env['EXTRAS']:
        extras_dir_list = makePathListAbsolute(env['EXTRAS'].split(':'))
    else:
        extras_dir_list = []

    Export('extras_dir_list')

    # Variables which were determined with Configure.
    env['CONF'] = {}

    # Walk the tree and execute all SConsopts scripts that wil add to the
    # above variables
    if GetOption('verbose'):
        print("Reading SConsopts")

    def trySConsopts(dir):
        sconsopts_path = os.path.join(dir, 'SConsopts')
        if not isfile(sconsopts_path):
            return
        if GetOption('verbose'):
            print("Reading", sconsopts_path)
        SConscript(sconsopts_path, exports={'main': env})

    trySConsopts(Dir('#').abspath)
    for bdir in [ base_dir ] + extras_dir_list:
        if not isdir(bdir):
            error("Directory '%s' does not exist." % bdir)
        for root, dirs, files in os.walk(bdir):
            trySConsopts(root)

    # Call any callbacks which the SConsopts files registered.
    for cb in after_sconsopts_callbacks:
        cb()

    # Update env for new variables added by the SConsopts.
    sticky_vars.Update(env)

    Help('''
Build variables for {dir}:
{help}
'''.format(dir=variant_dir, help=sticky_vars.GenerateHelpText(env)),
         append=True)

    # If the old vars file exists, delete it to avoid confusion/stale values.
    if isfile(build_root_vars):
        warning(f'Deleting old variant variables file "{build_root_vars}"')
        remove(build_root_vars)
    # Save sticky variables back to the gem5.build variant variables file.
    sticky_vars.Save(gem5_build_vars, env)

    # Pull all the sticky variables into the CONF dict.
    env['CONF'].update({key: env[key] for key in sticky_vars.keys()})

    # Do this after we save setting back, or else we'll tack on an
    # extra 'qdo' every time we run scons.
    if env['CONF']['BATCH']:
        env['CC']     = env['CONF']['BATCH_CMD'] + ' ' + env['CC']
        env['CXX']    = env['CONF']['BATCH_CMD'] + ' ' + env['CXX']
        env['AS']     = env['CONF']['BATCH_CMD'] + ' ' + env['AS']
        env['AR']     = env['CONF']['BATCH_CMD'] + ' ' + env['AR']
        env['RANLIB'] = env['CONF']['BATCH_CMD'] + ' ' + env['RANLIB']

    # Cache build files in the supplied directory.
    if env['CONF']['M5_BUILD_CACHE']:
        print('Using build cache located at', env['CONF']['M5_BUILD_CACHE'])
        CacheDir(env['CONF']['M5_BUILD_CACHE'])


    env.Append(CCFLAGS='$CCFLAGS_EXTRA')
    env.Append(LINKFLAGS='$LINKFLAGS_EXTRA')

    exports=['env', 'gem5py_env']

    ext_dir = Dir('#ext').abspath
    variant_ext = os.path.join(variant_path, 'ext')
    for root, dirs, files in os.walk(ext_dir):
        if 'SConscript' in files:
            build_dir = os.path.relpath(root, ext_dir)
            SConscript(os.path.join(root, 'SConscript'),
                       variant_dir=os.path.join(variant_ext, build_dir),
                       exports=exports)

    # The src/SConscript file sets up the build rules in 'env' according
    # to the configured variables.  It returns a list of environments,
    # one for each variant build (debug, opt, etc.)
    SConscript('src/SConscript', variant_dir=variant_path, exports=exports)

atexit.register(summarize_warnings)
