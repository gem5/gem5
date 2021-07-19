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

from os import mkdir, environ
from os.path import abspath, dirname, expanduser
from os.path import isdir, isfile
from os.path import join, split

# SCons imports
import SCons
import SCons.Node
import SCons.Node.FS
import SCons.Tool


########################################################################
#
# Command line options.
#
########################################################################

AddOption('--no-colors', dest='use_colors', action='store_false',
          help="Don't add color to abbreviated scons output")
AddOption('--with-cxx-config', action='store_true',
          help="Build with support for C++-based configuration")
AddOption('--default',
          help='Override which build_opts file to use for defaults')
AddOption('--ignore-style', action='store_true',
          help='Disable style checking hooks')
AddOption('--gold-linker', action='store_true', help='Use the gold linker')
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

# Imports of gem5_scons happen here since it depends on some options which are
# declared above.
from gem5_scons import error, warning, summarize_warnings, parse_build_path
from gem5_scons import TempFileSpawn, EnvDefaults, MakeAction, MakeActionTool
import gem5_scons
from gem5_scons.builders import ConfigFile, AddLocalRPATH, SwitchingHeaders
from gem5_scons.util import compareVersions, readCommand

Export('MakeAction')

########################################################################
#
# Set up the main build environment.
#
########################################################################

main = Environment(tools=[
        'default', 'git', TempFileSpawn, EnvDefaults, MakeActionTool,
        ConfigFile, AddLocalRPATH, SwitchingHeaders
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

main.SConsignFile(os.path.join(build_root, "sconsign"))


########################################################################
#
# Set up global sticky variables... these are common to an entire build
# tree (not specific to a particular build like X86)
#
########################################################################

global_vars_file = os.path.join(build_root, 'variables.global')

global_vars = Variables(global_vars_file, args=ARGUMENTS)

global_vars.AddVariables(
    ('CC', 'C compiler', environ.get('CC', main['CC'])),
    ('CXX', 'C++ compiler', environ.get('CXX', main['CXX'])),
    ('CCFLAGS_EXTRA', 'Extra C and C++ compiler flags', ''),
    ('LDFLAGS_EXTRA', 'Extra linker flags', ''),
    ('MARSHAL_CCFLAGS_EXTRA', 'Extra C and C++ marshal compiler flags', ''),
    ('MARSHAL_LDFLAGS_EXTRA', 'Extra marshal linker flags', ''),
    ('PYTHON_CONFIG', 'Python config binary to use',
     [ 'python3-config', 'python-config']
    ),
    ('PROTOC', 'protoc tool', environ.get('PROTOC', 'protoc')),
    ('BATCH', 'Use batch pool for build and tests', False),
    ('BATCH_CMD', 'Batch pool submission command name', 'qdo'),
    ('M5_BUILD_CACHE', 'Cache built objects in this directory', False),
    ('EXTRAS', 'Add extra directories to the compilation', '')
    )

# Update main environment with values from ARGUMENTS & global_vars_file
global_vars.Update(main)
Help('''
Global build variables:
{help}
'''.format(help=global_vars.GenerateHelpText(main)), append=True)

# Save sticky variable settings back to current variables file
global_vars.Save(global_vars_file, main)


########################################################################
#
# Set up various paths.
#
########################################################################

# Parse EXTRAS variable to build list of all directories where we're
# look for sources etc.  This list is exported as extras_dir_list.
base_dir = Dir('#src').abspath
if main['EXTRAS']:
    extras_dir_list = makePathListAbsolute(main['EXTRAS'].split(':'))
else:
    extras_dir_list = []

Export('base_dir')
Export('extras_dir_list')

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
main['LTO_LDFLAGS'] = []

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

# Set up default C++ compiler flags
if main['GCC'] or main['CLANG']:
    # As gcc and clang share many flags, do the common parts here
    main.Append(CCFLAGS=['-pipe'])
    main.Append(CCFLAGS=['-fno-strict-aliasing'])

    # Enable -Wall and -Wextra and then disable the few warnings that
    # we consistently violate
    main.Append(CCFLAGS=['-Wall', '-Wundef', '-Wextra',
                         '-Wno-sign-compare', '-Wno-unused-parameter'])

    # We always compile using C++17
    main.Append(CXXFLAGS=['-std=c++17'])

    if sys.platform.startswith('freebsd'):
        main.Append(CCFLAGS=['-I/usr/local/include'])
        main.Append(CXXFLAGS=['-I/usr/local/include'])
        # On FreeBSD we need libthr.
        main.Append(LIBS=['thr'])

    with gem5_scons.Configure(main) as conf:
        conf.CheckLinkFlag('-Wl,--as-needed')
    if GetOption('gold_linker'):
        main.Append(LINKFLAGS='-fuse-ld=gold')

else:
    error('\n'.join((
          "Don't know what compiler options to use for your compiler.",
          "compiler: " + main['CXX'],
          "version: " + CXX_version.replace('\n', '<nl>') if
                CXX_version else 'COMMAND NOT FOUND!',
          "If you're trying to use a compiler other than GCC",
          "or clang, there appears to be something wrong with your",
          "environment.",
          "",
          "If you are trying to use a compiler other than those listed",
          "above you will need to ease fix SConstruct and ",
          "src/SConscript to support that compiler.")))

if main['GCC']:
    if compareVersions(main['CXXVERSION'], "5") < 0:
        error('gcc version 5 or newer required.\n'
              'Installed version:', main['CXXVERSION'])

    # Add the appropriate Link-Time Optimization (LTO) flags if `--with-lto` is
    # set.
    if GetOption('with_lto'):
        # g++ uses "make" to parallelize LTO. The program can be overriden with
        # the environment variable "MAKE", but we currently make no attempt to
        # plumb that variable through.
        parallelism = ''
        if main.Detect('make'):
            parallelism = '=%d' % GetOption('num_jobs')
        else:
            warning('"make" not found, link time optimization will be '
                    'single threaded.')

        for var in 'LTO_CCFLAGS', 'LTO_LDFLAGS':
            # Use the same amount of jobs for LTO as we are running scons with.
            main[var] = ['-flto%s' % parallelism]

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin-malloc', '-fno-builtin-calloc',
                                  '-fno-builtin-realloc', '-fno-builtin-free'])

elif main['CLANG']:
    if compareVersions(main['CXXVERSION'], "6") < 0:
        error('clang version 6 or newer required.\n'
              'Installed version:', main['CXXVERSION'])

    # Set the Link-Time Optimization (LTO) flags if enabled.
    if GetOption('with_lto'):
        for var in 'LTO_CCFLAGS', 'LTO_LDFLAGS':
            main[var] = ['-flto']

    # clang has a few additional warnings that we disable.
    with gem5_scons.Configure(main) as conf:
        conf.CheckCxxFlag('-Wno-c99-designator')
        conf.CheckCxxFlag('-Wno-defaulted-function-deleted')

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin'])

    # On Mac OS X/Darwin we need to also use libc++ (part of XCode) as
    # opposed to libstdc++, as the later is dated.
    if sys.platform == "darwin":
        main.Append(CXXFLAGS=['-stdlib=libc++'])
        main.Append(LIBS=['c++'])

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
    main['ENV']['LSAN_OPTIONS'] = ':'.join([suppressions_opt,
                                            'print_suppressions=0'])
    print()
    warning('To suppress false positive leaks, set the LSAN_OPTIONS '
            'environment variable to "%s" when running gem5' %
            suppressions_opt)
    warning('LSAN_OPTIONS=suppressions=%s' % suppressions_opt)
    print()
if sanitizers:
    sanitizers = ','.join(sanitizers)
    if main['GCC'] or main['CLANG']:
        main.Append(CCFLAGS=['-fsanitize=%s' % sanitizers,
                             '-fno-omit-frame-pointer'],
                    LINKFLAGS='-fsanitize=%s' % sanitizers)
    else:
        warning("Don't know how to enable %s sanitizer(s) for your "
                "compiler." % sanitizers)

# Do this after we save setting back, or else we'll tack on an
# extra 'qdo' every time we run scons.
if main['BATCH']:
    main['CC']     = main['BATCH_CMD'] + ' ' + main['CC']
    main['CXX']    = main['BATCH_CMD'] + ' ' + main['CXX']
    main['AS']     = main['BATCH_CMD'] + ' ' + main['AS']
    main['AR']     = main['BATCH_CMD'] + ' ' + main['AR']
    main['RANLIB'] = main['BATCH_CMD'] + ' ' + main['RANLIB']

if sys.platform == 'cygwin':
    # cygwin has some header file issues...
    main.Append(CCFLAGS=["-Wno-uninitialized"])


# Cache build files in the supplied directory.
if main['M5_BUILD_CACHE']:
    print('Using build cache located at', main['M5_BUILD_CACHE'])
    CacheDir(main['M5_BUILD_CACHE'])

if not GetOption('no_compress_debug'):
    with gem5_scons.Configure(main) as conf:
        if not conf.CheckCxxFlag('-gz'):
            warning("Can't enable object file debug section compression")
        if not conf.CheckLinkFlag('-gz'):
            warning("Can't enable executable debug section compression")


########################################################################
#
# Detect and configure external dependencies.
#
########################################################################

main['USE_PYTHON'] = not GetOption('without_python')
if main['USE_PYTHON']:
    # Find Python include and library directories for embedding the
    # interpreter. We rely on python-config to resolve the appropriate
    # includes and linker flags. If you want to link in an alternate version
    # of python, override the PYTHON_CONFIG variable.

    python_config = main.Detect(main['PYTHON_CONFIG'])
    if python_config is None:
        error("Can't find a suitable python-config, tried %s" % \
              main['PYTHON_CONFIG'])

    print("Info: Using Python config: %s" % python_config)

    cmd = [python_config, '--ldflags', '--includes']

    # Starting in Python 3.8 the --embed flag is required. Use it if supported.
    with gem5_scons.Configure(main) as conf:
        if conf.TryAction('@%s --embed' % python_config)[0]:
            cmd.append('--embed')

    def flag_filter(env, cmd_output):
        flags = cmd_output.split()
        prefixes = ('-l', '-L', '-I')
        is_useful = lambda x: any(x.startswith(prefix) for prefix in prefixes)
        useful_flags = list(filter(is_useful, flags))
        env.MergeFlags(' '.join(useful_flags))

    main.ParseConfig(cmd, flag_filter)

    main.Prepend(CPPPATH=Dir('ext/pybind11/include/'))

    with gem5_scons.Configure(main) as conf:
        # verify that this stuff works
        if not conf.CheckHeader('Python.h', '<>'):
            error("Check failed for Python.h header.\n",
                  "Two possible reasons:\n"
                  "1. Python headers are not installed (You can install the "
                  "package python-dev on Ubuntu and RedHat)\n"
                  "2. SCons is using a wrong C compiler. This can happen if "
                  "CC has the wrong value.\n"
                  "CC = %s" % main['CC'])
        py_version = conf.CheckPythonLib()
        if not py_version:
            error("Can't find a working Python installation")

    marshal_env = main.Clone()

    # Bare minimum environment that only includes python
    marshal_env.Append(CCFLAGS='$MARSHAL_CCFLAGS_EXTRA')
    marshal_env.Append(LINKFLAGS='$MARSHAL_LDFLAGS_EXTRA')

    # Found a working Python installation. Check if it meets minimum
    # requirements.
    ver_string = '.'.join(map(str, py_version))
    if py_version[0] < 3 or (py_version[0] == 3 and py_version[1] < 6):
        error('Embedded python library 3.6 or newer required, found %s.' %
              ver_string)
    elif py_version[0] > 3:
        warning('Embedded python library too new. '
                'Python 3 expected, found %s.' % ver_string)

main['HAVE_PKG_CONFIG'] = main.Detect('pkg-config')

with gem5_scons.Configure(main) as conf:
    # On Solaris you need to use libsocket for socket ops
    if not conf.CheckLibWithHeader(
            [None, 'socket'], 'sys/socket.h', 'C++', 'accept(0,0,0);'):
       error("Can't find library with socket calls (e.g. accept()).")

    if not conf.CheckLibWithHeader('z', 'zlib.h', 'C++','zlibVersion();'):
        error('Did not find needed zlib compression library '
              'and/or zlib.h header file.\n'
              'Please install zlib and try again.')

if not GetOption('without_tcmalloc'):
    with gem5_scons.Configure(main) as conf:
        if conf.CheckLib('tcmalloc'):
            conf.env.Append(CCFLAGS=conf.env['TCMALLOC_CCFLAGS'])
        elif conf.CheckLib('tcmalloc_minimal'):
            conf.env.Append(CCFLAGS=conf.env['TCMALLOC_CCFLAGS'])
        else:
            warning("You can get a 12% performance improvement by "
                    "installing tcmalloc (libgoogle-perftools-dev package "
                    "on Ubuntu or RedHat).")


########################################################################
#
# Read and process SConsopts files. These can add new settings which
# affect each variant directory independently.
#
########################################################################

# Register a callback which is called after all SConsopts files have been read.
after_sconsopts_callbacks = []
def AfterSConsopts(cb):
    after_sconsopts_callbacks.append(cb)
Export('AfterSConsopts')

# Sticky variables get saved in the variables file so they persist from
# one invocation to the next (unless overridden, in which case the new
# value becomes sticky).
sticky_vars = Variables(args=ARGUMENTS)
Export('sticky_vars')

# Sticky variables that should be exported to #defines in config/*.hh
# (see src/SConscript).
export_vars = []
Export('export_vars')

# Walk the tree and execute all SConsopts scripts that wil add to the
# above variables
if GetOption('verbose'):
    print("Reading SConsopts")
for bdir in [ base_dir ] + extras_dir_list:
    if not isdir(bdir):
        error("Directory '%s' does not exist." % bdir)
    for root, dirs, files in os.walk(bdir):
        if 'SConsopts' in files:
            if GetOption('verbose'):
                print("Reading", os.path.join(root, 'SConsopts'))
            SConscript(os.path.join(root, 'SConsopts'))

# Call any callbacks which the SConsopts files registered.
for cb in after_sconsopts_callbacks:
    cb()

# Add any generic sticky variables here.
sticky_vars.Add(BoolVariable('USE_EFENCE',
    'Link with Electric Fence malloc debugger', False))


########################################################################
#
# Find and process all the SConscript files in ext. These are shared by
# all variants in a build root.
#
########################################################################

ext_dir = Dir('#ext').abspath
ext_build_dirs = []
for root, dirs, files in os.walk(ext_dir):
    if 'SConscript' in files:
        build_dir = os.path.relpath(root, ext_dir)
        ext_build_dirs.append(build_dir)
        main.SConscript(os.path.join(root, 'SConscript'),
                        variant_dir=os.path.join(build_root, build_dir))

gdb_xml_dir = os.path.join(ext_dir, 'gdb-xml')
Export('gdb_xml_dir')


########################################################################
#
# Define build environments for required variants.
#
########################################################################

for variant_path in variant_paths:
    if not GetOption('silent'):
        print("Building in", variant_path)

    # Make a copy of the build-root environment to use for this config.
    env = main.Clone()
    env['BUILDDIR'] = variant_path

    # variant_dir is the tail component of build path, and is used to
    # determine the build parameters (e.g., 'X86')
    (build_root, variant_dir) = os.path.split(variant_path)

    # Set env variables according to the build directory config.
    sticky_vars.files = []
    # Variables for $BUILD_ROOT/$VARIANT_DIR are stored in
    # $BUILD_ROOT/variables/$VARIANT_DIR so you can nuke
    # $BUILD_ROOT/$VARIANT_DIR without losing your variables settings.
    current_vars_file = os.path.join(build_root, 'variables', variant_dir)
    if isfile(current_vars_file):
        sticky_vars.files.append(current_vars_file)
        if not GetOption('silent'):
            print("Using saved variables file %s" % current_vars_file)
    elif variant_dir in ext_build_dirs:
        # Things in ext are built without a variant directory.
        continue
    else:
        # Variant specific variables file doesn't exist.

        # Make sure the directory is there so we can create the file later.
        opt_dir = dirname(current_vars_file)
        if not isdir(opt_dir):
            mkdir(opt_dir)

        # Get default build variables from source tree.  Variables are
        # normally determined by name of $VARIANT_DIR, but can be
        # overridden by '--default=' arg on command line.
        default = GetOption('default')
        opts_dir = Dir('#build_opts').abspath
        if default:
            default_vars_files = [
                    os.path.join(build_root, 'variables', default),
                    os.path.join(opts_dir, default)
                ]
        else:
            default_vars_files = [os.path.join(opts_dir, variant_dir)]
        existing_files = list(filter(isfile, default_vars_files))
        if existing_files:
            default_vars_file = existing_files[0]
            sticky_vars.files.append(default_vars_file)
            print("Variables file %s not found,\n  using defaults in %s"
                  % (current_vars_file, default_vars_file))
        else:
            error("Cannot find variables file %s or default file(s) %s"
                  % (current_vars_file, ' or '.join(default_vars_files)))
            Exit(1)

    # Apply current variable settings to env
    sticky_vars.Update(env)

    Help('''
Build variables for {dir}:
{help}
'''.format(dir=variant_dir, help=sticky_vars.GenerateHelpText(env)),
         append=True)

    # Process variable settings.
    if env['USE_EFENCE']:
        env.Append(LIBS=['efence'])

    if env['KVM_ISA'] != env['TARGET_ISA']:
        env['USE_KVM'] = False

    # Save sticky variable settings back to current variables file
    sticky_vars.Save(current_vars_file, env)

    env.Append(CCFLAGS='$CCFLAGS_EXTRA')
    env.Append(LINKFLAGS='$LDFLAGS_EXTRA')

    exports=['env']
    if main['USE_PYTHON']:
        exports.append('marshal_env')

    # The src/SConscript file sets up the build rules in 'env' according
    # to the configured variables.  It returns a list of environments,
    # one for each variant build (debug, opt, etc.)
    SConscript('src/SConscript', variant_dir=variant_path, exports=exports)

atexit.register(summarize_warnings)
