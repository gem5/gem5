# -*- mode:python -*-

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
#
# Authors: Steve Reinhardt

###################################################
#
# SCons top-level build description (SConstruct) file.
#
# While in this directory ('m5'), just type 'scons' to build the default
# configuration (see below), or type 'scons build/<CONFIG>/<binary>'
# to build some other configuration (e.g., 'build/ALPHA_FS/m5.opt' for
# the optimized full-system version).
#
# You can build M5 in a different directory as long as there is a
# 'build/<CONFIG>' somewhere along the target path.  The build system
# expdects that all configs under the same build directory are being
# built for the same host system.
#
# Examples:
#   These two commands are equivalent.  The '-u' option tells scons to
#   search up the directory tree for this SConstruct file.
#   % cd <path-to-src>/m5 ; scons build/ALPHA_FS/m5.debug
#   % cd <path-to-src>/m5/build/ALPHA_FS; scons -u m5.debug
#   These two commands are equivalent and demonstrate building in a
#   directory outside of the source tree.  The '-C' option tells scons
#   to chdir to the specified directory to find this SConstruct file.
#   % cd <path-to-src>/m5 ; scons /local/foo/build/ALPHA_FS/m5.debug
#   % cd /local/foo/build/ALPHA_FS; scons -C <path-to-src>/m5 m5.debug
#
# You can use 'scons -H' to print scons options.  If you're in this
# 'm5' directory (or use -u or -C to tell scons where to find this
# file), you can use 'scons -h' to print all the M5-specific build
# options as well.
#
###################################################

# Python library imports
import sys
import os

# Check for recent-enough Python and SCons versions.  If your system's
# default installation of Python is not recent enough, you can use a
# non-default installation of the Python interpreter by either (1)
# rearranging your PATH so that scons finds the non-default 'python'
# first or (2) explicitly invoking an alternative interpreter on the
# scons script, e.g., "/usr/local/bin/python2.4 `which scons` [args]".
EnsurePythonVersion(2,4)

# Ironically, SCons 0.96 dies if you give EnsureSconsVersion a
# 3-element version number.
min_scons_version = (0,96,91)
try:
    EnsureSConsVersion(*min_scons_version)
except:
    print "Error checking current SCons version."
    print "SCons", ".".join(map(str,min_scons_version)), "or greater required."
    Exit(2)
    

# The absolute path to the current directory (where this file lives).
ROOT = Dir('.').abspath

# Paths to the M5 and external source trees.
SRCDIR = os.path.join(ROOT, 'src')

# tell python where to find m5 python code
sys.path.append(os.path.join(ROOT, 'src/python'))

###################################################
#
# Figure out which configurations to set up based on the path(s) of
# the target(s).
#
###################################################

# Find default configuration & binary.
Default(os.environ.get('M5_DEFAULT_BINARY', 'build/ALPHA_SE/m5.debug'))

# Ask SCons which directory it was invoked from.
launch_dir = GetLaunchDir()

# Make targets relative to invocation directory
abs_targets = map(lambda x: os.path.normpath(os.path.join(launch_dir, str(x))),
                  BUILD_TARGETS)

# helper function: find last occurrence of element in list
def rfind(l, elt, offs = -1):
    for i in range(len(l)+offs, 0, -1):
        if l[i] == elt:
            return i
    raise ValueError, "element not found"

# Each target must have 'build' in the interior of the path; the
# directory below this will determine the build parameters.  For
# example, for target 'foo/bar/build/ALPHA_SE/arch/alpha/blah.do' we
# recognize that ALPHA_SE specifies the configuration because it
# follow 'build' in the bulid path.

# Generate a list of the unique build roots and configs that the
# collected targets reference.
build_paths = []
build_root = None
for t in abs_targets:
    path_dirs = t.split('/')
    try:
        build_top = rfind(path_dirs, 'build', -2)
    except:
        print "Error: no non-leaf 'build' dir found on target path", t
        Exit(1)
    this_build_root = os.path.join('/',*path_dirs[:build_top+1])
    if not build_root:
        build_root = this_build_root
    else:
        if this_build_root != build_root:
            print "Error: build targets not under same build root\n"\
                  "  %s\n  %s" % (build_root, this_build_root)
            Exit(1)
    build_path = os.path.join('/',*path_dirs[:build_top+2])
    if build_path not in build_paths:
        build_paths.append(build_path)

###################################################
#
# Set up the default build environment.  This environment is copied
# and modified according to each selected configuration.
#
###################################################

env = Environment(ENV = os.environ,  # inherit user's environment vars
                  ROOT = ROOT,
                  SRCDIR = SRCDIR)

env.SConsignFile("sconsign")

# Default duplicate option is to use hard links, but this messes up
# when you use emacs to edit a file in the target dir, as emacs moves
# file to file~ then copies to file, breaking the link.  Symbolic
# (soft) links work better.
env.SetOption('duplicate', 'soft-copy')

# I waffle on this setting... it does avoid a few painful but
# unnecessary builds, but it also seems to make trivial builds take
# noticeably longer.
if False:
    env.TargetSignatures('content')

# M5_PLY is used by isa_parser.py to find the PLY package.
env.Append(ENV = { 'M5_PLY' : Dir('ext/ply') })

# Set up default C++ compiler flags
env.Append(CCFLAGS='-pipe')
env.Append(CCFLAGS='-fno-strict-aliasing')
env.Append(CCFLAGS=Split('-Wall -Wno-sign-compare -Werror -Wundef'))
if sys.platform == 'cygwin':
    # cygwin has some header file issues...
    env.Append(CCFLAGS=Split("-Wno-uninitialized"))
env.Append(CPPPATH=[Dir('ext/dnet')])

# Find Python include and library directories for embedding the
# interpreter.  For consistency, we will use the same Python
# installation used to run scons (and thus this script).  If you want
# to link in an alternate version, see above for instructions on how
# to invoke scons with a different copy of the Python interpreter.

# Get brief Python version name (e.g., "python2.4") for locating
# include & library files
py_version_name = 'python' + sys.version[:3]

# include path, e.g. /usr/local/include/python2.4
env.Append(CPPPATH = os.path.join(sys.exec_prefix, 'include', py_version_name))
env.Append(LIBS = py_version_name)
# add library path too if it's not in the default place
if sys.exec_prefix != '/usr':
    env.Append(LIBPATH = os.path.join(sys.exec_prefix, 'lib'))

# Set up SWIG flags & scanner

env.Append(SWIGFLAGS=Split('-c++ -python -modern $_CPPINCFLAGS'))

import SCons.Scanner

swig_inc_re = '^[ \t]*[%,#][ \t]*(?:include|import)[ \t]*(<|")([^>"]+)(>|")'

swig_scanner = SCons.Scanner.ClassicCPP("SwigScan", ".i", "CPPPATH",
                                        swig_inc_re)

env.Append(SCANNERS = swig_scanner)

# Other default libraries
env.Append(LIBS=['z'])

# Platform-specific configuration.  Note again that we assume that all
# builds under a given build root run on the same host platform.
conf = Configure(env,
                 conf_dir = os.path.join(build_root, '.scons_config'),
                 log_file = os.path.join(build_root, 'scons_config.log'))

# Check for <fenv.h> (C99 FP environment control)
have_fenv = conf.CheckHeader('fenv.h', '<>')
if not have_fenv:
    print "Warning: Header file <fenv.h> not found."
    print "         This host has no IEEE FP rounding mode control."

# Check for mysql.
mysql_config = WhereIs('mysql_config')
have_mysql = mysql_config != None

# Check MySQL version.
if have_mysql:
    mysql_version = os.popen(mysql_config + ' --version').read()
    mysql_version = mysql_version.split('.')
    mysql_major = int(mysql_version[0])
    mysql_minor = int(mysql_version[1])
    # This version check is probably overly conservative, but it deals
    # with the versions we have installed.
    if mysql_major < 4 or (mysql_major == 4 and mysql_minor < 1):
        print "Warning: MySQL v4.1 or newer required."
        have_mysql = False

# Set up mysql_config commands.
if have_mysql:
    mysql_config_include = mysql_config + ' --include'
    if os.system(mysql_config_include + ' > /dev/null') != 0:
        # older mysql_config versions don't support --include, use
        # --cflags instead
        mysql_config_include = mysql_config + ' --cflags | sed s/\\\'//g'
    # This seems to work in all versions
    mysql_config_libs = mysql_config + ' --libs'

env = conf.Finish()

# Define the universe of supported ISAs
env['ALL_ISA_LIST'] = ['alpha', 'sparc', 'mips']

# Define the universe of supported CPU models
env['ALL_CPU_LIST'] = ['AtomicSimpleCPU', 'TimingSimpleCPU',
                       'FullCPU', 'AlphaO3CPU',
                       'OzoneSimpleCPU', 'OzoneCPU']

# Sticky options get saved in the options file so they persist from
# one invocation to the next (unless overridden, in which case the new
# value becomes sticky).
sticky_opts = Options(args=ARGUMENTS)
sticky_opts.AddOptions(
    EnumOption('TARGET_ISA', 'Target ISA', 'alpha', env['ALL_ISA_LIST']),
    BoolOption('FULL_SYSTEM', 'Full-system support', False),
    # There's a bug in scons 0.96.1 that causes ListOptions with list
    # values (more than one value) not to be able to be restored from
    # a saved option file.  If this causes trouble then upgrade to
    # scons 0.96.90 or later.
    ListOption('CPU_MODELS', 'CPU models', 'AtomicSimpleCPU,TimingSimpleCPU',
               env['ALL_CPU_LIST']),
    BoolOption('ALPHA_TLASER',
               'Model Alpha TurboLaser platform (vs. Tsunami)', False),
    BoolOption('NO_FAST_ALLOC', 'Disable fast object allocator', False),
    BoolOption('EFENCE', 'Link with Electric Fence malloc debugger',
               False),
    BoolOption('SS_COMPATIBLE_FP',
               'Make floating-point results compatible with SimpleScalar',
               False),
    BoolOption('USE_SSE2',
               'Compile for SSE2 (-msse2) to get IEEE FP on x86 hosts',
               False),
    BoolOption('USE_MYSQL', 'Use MySQL for stats output', have_mysql),
    BoolOption('USE_FENV', 'Use <fenv.h> IEEE mode control', have_fenv),
    BoolOption('USE_CHECKER', 'Use checker for detailed CPU models', False),
    ('CC', 'C compiler', os.environ.get('CC', env['CC'])),
    ('CXX', 'C++ compiler', os.environ.get('CXX', env['CXX'])),
    BoolOption('BATCH', 'Use batch pool for build and tests', False),
    ('BATCH_CMD', 'Batch pool submission command name', 'qdo')
    )

# Non-sticky options only apply to the current build.
nonsticky_opts = Options(args=ARGUMENTS)
nonsticky_opts.AddOptions(
    BoolOption('update_ref', 'Update test reference outputs', False)
    )

# These options get exported to #defines in config/*.hh (see m5/SConscript).
env.ExportOptions = ['FULL_SYSTEM', 'ALPHA_TLASER', 'USE_FENV', \
                     'USE_MYSQL', 'NO_FAST_ALLOC', 'SS_COMPATIBLE_FP', \
                     'USE_CHECKER']

# Define a handy 'no-op' action
def no_action(target, source, env):
    return 0

env.NoAction = Action(no_action, None)

###################################################
#
# Define a SCons builder for configuration flag headers.
#
###################################################

# This function generates a config header file that #defines the
# option symbol to the current option setting (0 or 1).  The source
# operands are the name of the option and a Value node containing the
# value of the option.
def build_config_file(target, source, env):
    (option, value) = [s.get_contents() for s in source]
    f = file(str(target[0]), 'w')
    print >> f, '#define', option, value
    f.close()
    return None

# Generate the message to be printed when building the config file.
def build_config_file_string(target, source, env):
    (option, value) = [s.get_contents() for s in source]
    return "Defining %s as %s in %s." % (option, value, target[0])

# Combine the two functions into a scons Action object.
config_action = Action(build_config_file, build_config_file_string)

# The emitter munges the source & target node lists to reflect what
# we're really doing.
def config_emitter(target, source, env):
    # extract option name from Builder arg
    option = str(target[0])
    # True target is config header file
    target = os.path.join('config', option.lower() + '.hh')
    # Force value to 0/1 even if it's a Python bool
    val = int(eval(str(env[option])))
    # Sources are option name & value (packaged in SCons Value nodes)
    return ([target], [Value(option), Value(val)])

config_builder = Builder(emitter = config_emitter, action = config_action)

env.Append(BUILDERS = { 'ConfigFile' : config_builder })

###################################################
#
# Define a SCons builder for copying files.  This is used by the
# Python zipfile code in src/python/SConscript, but is placed up here
# since it's potentially more generally applicable.
#
###################################################

copy_builder = Builder(action = Copy("$TARGET", "$SOURCE"))

env.Append(BUILDERS = { 'CopyFile' : copy_builder })

###################################################
#
# Define a simple SCons builder to concatenate files.
#
# Used to append the Python zip archive to the executable.
#
###################################################

concat_builder = Builder(action = Action(['cat $SOURCES > $TARGET',
                                          'chmod +x $TARGET']))

env.Append(BUILDERS = { 'Concat' : concat_builder })


# base help text
help_text = '''
Usage: scons [scons options] [build options] [target(s)]

'''

# libelf build is shared across all configs in the build root.
env.SConscript('ext/libelf/SConscript',
               build_dir = os.path.join(build_root, 'libelf'),
               exports = 'env')

###################################################
#
# Define build environments for selected configurations.
#
###################################################

# rename base env
base_env = env

for build_path in build_paths:
    print "Building in", build_path
    # build_dir is the tail component of build path, and is used to
    # determine the build parameters (e.g., 'ALPHA_SE')
    (build_root, build_dir) = os.path.split(build_path)
    # Make a copy of the build-root environment to use for this config.
    env = base_env.Copy()

    # Set env options according to the build directory config.
    sticky_opts.files = []
    # Options for $BUILD_ROOT/$BUILD_DIR are stored in
    # $BUILD_ROOT/options/$BUILD_DIR so you can nuke
    # $BUILD_ROOT/$BUILD_DIR without losing your options settings.
    current_opts_file = os.path.join(build_root, 'options', build_dir)
    if os.path.isfile(current_opts_file):
        sticky_opts.files.append(current_opts_file)
        print "Using saved options file %s" % current_opts_file
    else:
        # Build dir-specific options file doesn't exist.

        # Make sure the directory is there so we can create it later
        opt_dir = os.path.dirname(current_opts_file)
        if not os.path.isdir(opt_dir):
            os.mkdir(opt_dir)

        # Get default build options from source tree.  Options are
        # normally determined by name of $BUILD_DIR, but can be
        # overriden by 'default=' arg on command line.
        default_opts_file = os.path.join('build_opts',
                                         ARGUMENTS.get('default', build_dir))
        if os.path.isfile(default_opts_file):
            sticky_opts.files.append(default_opts_file)
            print "Options file %s not found,\n  using defaults in %s" \
                  % (current_opts_file, default_opts_file)
        else:
            print "Error: cannot find options file %s or %s" \
                  % (current_opts_file, default_opts_file)
            Exit(1)

    # Apply current option settings to env
    sticky_opts.Update(env)
    nonsticky_opts.Update(env)

    help_text += "Sticky options for %s:\n" % build_dir \
                 + sticky_opts.GenerateHelpText(env) \
                 + "\nNon-sticky options for %s:\n" % build_dir \
                 + nonsticky_opts.GenerateHelpText(env)

    # Process option settings.

    if not have_fenv and env['USE_FENV']:
        print "Warning: <fenv.h> not available; " \
              "forcing USE_FENV to False in", build_dir + "."
        env['USE_FENV'] = False

    if not env['USE_FENV']:
        print "Warning: No IEEE FP rounding mode control in", build_dir + "."
        print "         FP results may deviate slightly from other platforms."

    if env['EFENCE']:
        env.Append(LIBS=['efence'])

    if env['USE_MYSQL']:
        if not have_mysql:
            print "Warning: MySQL not available; " \
                  "forcing USE_MYSQL to False in", build_dir + "."
            env['USE_MYSQL'] = False
        else:
            print "Compiling in", build_dir, "with MySQL support."
            env.ParseConfig(mysql_config_libs)
            env.ParseConfig(mysql_config_include)

    # Check if the Checker is being used.  If so append it to env['CPU_MODELS']
    if env['USE_CHECKER']:
        env['CPU_MODELS'].append('CheckerCPU')

    # Save sticky option settings back to current options file
    sticky_opts.Save(current_opts_file, env)

    # Do this after we save setting back, or else we'll tack on an
    # extra 'qdo' every time we run scons.
    if env['BATCH']:
        env['CC']  = env['BATCH_CMD'] + ' ' + env['CC']
        env['CXX'] = env['BATCH_CMD'] + ' ' + env['CXX']

    if env['USE_SSE2']:
        env.Append(CCFLAGS='-msse2')

    # The m5/SConscript file sets up the build rules in 'env' according
    # to the configured options.  It returns a list of environments,
    # one for each variant build (debug, opt, etc.)
    envList = SConscript('src/SConscript', build_dir = build_path,
                         exports = 'env')

    # Set up the regression tests for each build.
#    for e in envList:
#        SConscript('m5-test/SConscript',
#                   build_dir = os.path.join(build_dir, 'test', e.Label),
#                   exports = { 'env' : e }, duplicate = False)

Help(help_text)

###################################################
#
# Let SCons do its thing.  At this point SCons will use the defined
# build environments to build the requested targets.
#
###################################################

