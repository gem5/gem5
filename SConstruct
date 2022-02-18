# -*- mode:python -*-

# Copyright (c) 2013, 2015-2020, 2023 ARM Limited
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
import itertools
import os
import sys

from os import mkdir, remove, environ, listdir
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
# Default to --no-duplicate-sources, but keep --duplicate-sources to opt-out
# of this new build behaviour in case it introduces regressions. We could use
# action=argparse.BooleanOptionalAction here once Python 3.9 is required.
AddOption('--duplicate-sources', action='store_true', default=False,
          dest='duplicate_sources',
          help='Create symlinks to sources in the build directory')
AddOption('--no-duplicate-sources', action='store_false',
          dest='duplicate_sources',
          help='Do not create symlinks to sources in the build directory')

# Inject the built_tools directory into the python path.
sys.path[1:1] = [ Dir('#build_tools').abspath ]

# Imports of gem5_scons happen here since it depends on some options which are
# declared above.
from gem5_scons import error, warning, summarize_warnings, parse_build_path
from gem5_scons import TempFileSpawn, EnvDefaults, MakeAction, MakeActionTool
from gem5_scons import kconfig
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

# Patch re.compile to support inline flags anywhere within a RE
# string. Required to use PLY with Python 3.11+.
gem5_scons.patch_re_compile_for_inline_flags()

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

buildopts_dir = Dir('#build_opts')
buildopts = list([f for f in os.listdir(buildopts_dir.abspath) if
        isfile(os.path.join(buildopts_dir.abspath, f))])
buildopts.sort()

buildopt_list = '\n'.join(' ' * 10 + buildopt for buildopt in buildopts)

Help(f"""
Targets:
        To build gem5 using a predefined configuration, use a target with
        a directory called "build" in the path, followed by a directory named
        after a predefined configuration, and then the actual target, likely
        a gem5 binary. For example:

        scons build/X86/gem5.opt

        The "build" component tells SCons that the next part names an initial
        configuration, and the part after that is the actual target.
        The predefined targets currently available are:

{buildopt_list}

        The extension on the gem5 binary specifies what type of binary to
        build. Options are:

        debug: A debug binary with optimizations turned off and debug info
            turned on.
        opt: An optimized binary with debugging still turned on.
        fast: An optimized binary with debugging, asserts, and tracing
            disabled.

        gem5 can also be built as a static or dynamic library. In that case,
        the extension is fixed by the operating system, so the binary type
        is part of the target file name. For example:

        scons build/ARM/libgem5_opt.so

        To build unit tests, you can use a target like this:

        scons build/RISCV/unittests.debug

        The unittests.debug part of the target is actual a directory which
        holds the results for all the unit tests built with the "debug"
        settings. When that's used as the target, SCons will build all the
        files under that directory, which will run all the tests.

        To build and run an individual test, you can built it's binary
        specifically and then run it manually:

        scons build/SPARC/base/bitunion.test.opt
        build/SPARC/base/bitunion.test.opt
""", append=True)


########################################################################
#
# Figure out which configurations to set up based on the path(s) of
# the target(s).
#
########################################################################

kconfig_actions = (
    'defconfig',
    'guiconfig',
    'listnewconfig',
    'menuconfig',
    'savedefconfig',
    'setconfig',
)

Help("""
Kconfig:
        In addition to the default configs, you can also create your own
        configs, or edit one that already exists. To use one of the kconfig
        tools with a particular directory, use a target which is the directory
        to configure, and then the name of the tool. For example, to run
        menuconfig on directory build/foo/bar, run:

        scons menuconfig build/foo/bar

        will set up a build directory in build/foo/bar if one doesn't already
        exist, and open the menuconfig editor to view/set configuration
        values.

        The tools available for working with kconfig are generally very
        similar to ones used with the linux kernel, so information about the
        kernel versions will typically (but not always) apply here as well.

Kconfig tools:
        defconfig:
        Set up a config using values specified in a defconfig file, or if no
        value is given, use the default. The second argument specifies the
        defconfig file. A defconfig file in the build_opts directory can be
        implicitly specified in the build path via `build/<defconfig file>/`

        scons defconfig build/foo/bar build_opts/MIPS


        guiconfig:
        Opens the guiconfig editor which will let you view and edit config
        values, and view help text. guiconfig runs as a graphical application.

        scons guiconfig build/foo/bar


        listnewconfig:
        Lists config options which are new in the Kconfig and which are not
        currently set in the existing config file.

        scons listnewconfig build/foo/bar


        menuconfig:
        Opens the menuconfig editor which will let you view and edit config
        values, and view help text. menuconfig runs in text mode.

        scons menuconfig build/foo/bar


        savedefconfig:
        Save a defconfig file which would give rise to the current config.
        For instance, you could use menuconfig to set up a config how you want
        it with the options you cared about, and then use savedefconfig to save
        a minimal config file. These files would be suitable to use in the
        defconfig directory. The second argument specifies the filename for
        the new defconfig file.

        scons savedefconfig build/foo/bar new_def_config


        setconfig:
        Set values in an existing config directory as specified on the command
        line. For example, to enable gem5's built in systemc kernel:

        scons setconfig build/foo/bar USE_SYSTEMC=y
""", append=True)

# Take a list of paths (or SCons Nodes) and return a list with all
# paths made absolute and ~-expanded.  Paths will be interpreted
# relative to the launch directory unless a different root is provided

def makePathAbsolute(path, root=GetLaunchDir()):
    return abspath(os.path.join(root, expanduser(str(path))))
def makePathListAbsolute(path_list, root=GetLaunchDir()):
    return [makePathAbsolute(p, root) for p in path_list]

if BUILD_TARGETS and BUILD_TARGETS[0] in kconfig_actions:
    # The build targets are really arguments for the kconfig action.
    kconfig_args = BUILD_TARGETS[:]
    BUILD_TARGETS[:] = []

    kconfig_action = kconfig_args[0]
    if len(kconfig_args) < 2:
        error(f'Missing arguments for kconfig action {kconfig_action}')
    dir_to_configure = makePathAbsolute(kconfig_args[1])

    kconfig_args = kconfig_args[2:]

    variant_paths = {dir_to_configure}
else:
    # Each target must have 'build' in the interior of the path; the
    # directory below this will determine the build parameters.  For
    # example, for target 'foo/bar/build/X86/arch/x86/blah.do' we
    # recognize that X86 specifies the configuration because it
    # follow 'build' in the build path.

    # The funky assignment to "[:]" is needed to replace the list contents
    # in place rather than reassign the symbol to a new list, which
    # doesn't work (obviously!).
    BUILD_TARGETS[:] = makePathListAbsolute(BUILD_TARGETS)

    # Generate a list of the unique build directories that the collected
    # targets reference.
    variant_paths = set(map(parse_build_path, BUILD_TARGETS))
    kconfig_action = None


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
if not GetOption('duplicate_sources'):
    main.Prepend(CPPPATH=Dir('src'))


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

# Find the gem5 binary target architecture (usually host architecture). The
# "Target: <target>" is consistent accross gcc and clang at the time of
# writting this.
bin_target_arch = readCommand([main['CXX'], '--verbose'], exception=False)
main["BIN_TARGET_ARCH"] = (
    "x86_64"
    if bin_target_arch.find("Target: x86_64") != -1
    else "aarch64"
    if bin_target_arch.find("Target: aarch64") != -1
    else "unknown"
)

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

    gem5_build = os.path.join(variant_path, 'gem5.build')
    env['GEM5BUILD'] = gem5_build
    Execute(Mkdir(gem5_build))

    config_file = Dir(gem5_build).File('config')
    kconfig_file = Dir(gem5_build).File('Kconfig')
    gem5_kconfig_file = Dir('#src').File('Kconfig')

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

        if compareVersions(env['CXXVERSION'], "9") < 0:
            # `libstdc++fs`` must be explicitly linked for `std::filesystem``
            # in GCC version 8. As of GCC version 9, this is not required.
            #
            # In GCC 7 the `libstdc++fs`` library explicit linkage is also
            # required but the `std::filesystem` is under the `experimental`
            # namespace(`std::experimental::filesystem`).
            #
            # Note: gem5 does not support GCC versions < 7.
            env.Append(LIBS=['stdc++fs'])

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

        if compareVersions(env['CXXVERSION'], "11") < 0:
            # `libstdc++fs`` must be explicitly linked for `std::filesystem``
            # in clang versions 6 through 10.
            #
            # In addition, for these versions, the
            # `std::filesystem` is under the `experimental`
            # namespace(`std::experimental::filesystem`).
            #
            # Note: gem5 does not support clang versions < 6.
            env.Append(LIBS=['stdc++fs'])


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
            libsan = (
                ['-static-libubsan', '-static-libasan']
                if env['GCC']
                else ['-static-libsan']
            )
            env.Append(CCFLAGS=['-fsanitize=%s' % sanitizers,
                                 '-fno-omit-frame-pointer'],
                       LINKFLAGS=['-fsanitize=%s' % sanitizers] + libsan)

            if main["BIN_TARGET_ARCH"] == "x86_64":
                # Sanitizers can enlarge binary size drammatically, north of
                # 2GB.  This can prevent successful linkage due to symbol
                # relocation outside from the 2GB region allocated by the small
                # x86_64 code model that is enabled by default (32-bit relative
                # offset limitation).  Switching to the medium model in x86_64
                # enables 64-bit relative offset for large objects (>64KB by
                # default) while sticking to 32-bit relative addressing for
                # code and smaller objects. Note this comes at a potential
                # performance cost so it should not be enabled in all cases.
                # This should still be a very happy medium for
                # non-perf-critical sanitized builds.
                env.Append(CCFLAGS='-mcmodel=medium')
                env.Append(LINKFLAGS='-mcmodel=medium')
            elif main["BIN_TARGET_ARCH"] == "aarch64":
                # aarch64 default code model is small but with different
                # constrains than for x86_64. With aarch64, the small code
                # model enables 4GB distance between symbols. This is
                # sufficient for the largest ALL/gem5.debug target with all
                # sanitizers enabled at the time of writting this. Note that
                # the next aarch64 code model is "large" which prevents dynamic
                # linkage so it should be avoided when possible.
                pass
            else:
                warning(
                    "Unknown code model options for your architecture. "
                    "Linkage might fail for larger binaries "
                    "(e.g., ALL/gem5.debug with sanitizers enabled)."
                )
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

    env['HAVE_PKG_CONFIG'] = env.Detect('pkg-config') == 'pkg-config'

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
            if conf.CheckLib('tcmalloc_minimal'):
                conf.env.Append(CCFLAGS=conf.env['TCMALLOC_CCFLAGS'])
            elif conf.CheckLib('tcmalloc'):
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

    extras_file = os.path.join(gem5_build, 'extras')
    extras_var = Variables(extras_file, args=ARGUMENTS)

    extras_var.Add(('EXTRAS', 'Add extra directories to the compilation', ''))

    # Apply current settings for EXTRAS to env.
    extras_var.Update(env)

    # Parse EXTRAS variable to build list of all directories where we're
    # look for sources etc.  This list is exported as extras_dir_list.
    if env['EXTRAS']:
        extras_dir_list = makePathListAbsolute(env['EXTRAS'].split(':'))
    else:
        extras_dir_list = []

    Export('extras_dir_list')

    # Generate a Kconfig that will source the main gem5 one, and any in any
    # EXTRAS directories.
    kconfig_base_py = Dir('#build_tools').File('kconfig_base.py')
    kconfig_base_cmd_parts = [f'"{kconfig_base_py}" "{kconfig_file.abspath}"',
            f'"{gem5_kconfig_file.abspath}"']
    for ed in extras_dir_list:
        kconfig_base_cmd_parts.append(f'"{ed}"')
    kconfig_base_cmd = ' '.join(kconfig_base_cmd_parts)
    if env.Execute(kconfig_base_cmd) != 0:
        error("Failed to build base Kconfig file")

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

    # Handle any requested kconfig action, then exit.
    if kconfig_action:
        if kconfig_action == 'defconfig':
            if len(kconfig_args) != 1:
                error('Usage: scons defconfig <build dir> <defconfig file>')
            defconfig_path = makePathAbsolute(kconfig_args[0])
            kconfig.defconfig(env, kconfig_file.abspath,
                    defconfig_path, config_file.abspath)
        elif kconfig_action == 'guiconfig':
            kconfig.guiconfig(env, kconfig_file.abspath, config_file.abspath,
                    variant_path)
        elif kconfig_action == 'listnewconfig':
            kconfig.listnewconfig(env, kconfig_file.abspath,
                    config_file.abspath)
        elif kconfig_action == 'menuconfig':
            kconfig.menuconfig(env, kconfig_file.abspath, config_file.abspath,
                    variant_path)
        elif kconfig_action == 'savedefconfig':
            if len(kconfig_args) != 1:
                error('Usage: scons defconfig <build dir> <defconfig file>')
            defconfig_path = makePathAbsolute(kconfig_args[0])
            kconfig.savedefconfig(env, kconfig_file.abspath,
                    config_file.abspath, defconfig_path)
        elif kconfig_action == 'setconfig':
            kconfig.setconfig(env, kconfig_file.abspath, config_file.abspath,
                    ARGUMENTS)
        else:
            error(f'Unrecognized kconfig action {kconfig_action}')
        Exit(0)

    # If no config exists yet, see if we know how to make one?
    if not isfile(config_file.abspath):
        buildopts_file = Dir('#build_opts').File(variant_dir)
        if not isfile(buildopts_file.abspath):
            error('No config found, and no implicit config recognized')
        kconfig.defconfig(env, kconfig_file.abspath, buildopts_file.abspath,
                config_file.abspath)

    kconfig.update_env(env, kconfig_file.abspath, config_file.abspath)

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
                       exports=exports,
                       duplicate=GetOption('duplicate_sources'))

    # The src/SConscript file sets up the build rules in 'env' according
    # to the configured variables.  It returns a list of environments,
    # one for each variant build (debug, opt, etc.)
    SConscript('src/SConscript', variant_dir=variant_path, exports=exports,
               duplicate=GetOption('duplicate_sources'))

atexit.register(summarize_warnings)
