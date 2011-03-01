# -*- mode:python -*-

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
#
# Authors: Steve Reinhardt
#          Nathan Binkert

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
# expects that all configs under the same build directory are being
# built for the same host system.
#
# Examples:
#
#   The following two commands are equivalent.  The '-u' option tells
#   scons to search up the directory tree for this SConstruct file.
#   % cd <path-to-src>/m5 ; scons build/ALPHA_FS/m5.debug
#   % cd <path-to-src>/m5/build/ALPHA_FS; scons -u m5.debug
#
#   The following two commands are equivalent and demonstrate building
#   in a directory outside of the source tree.  The '-C' option tells
#   scons to chdir to the specified directory to find this SConstruct
#   file.
#   % cd <path-to-src>/m5 ; scons /local/foo/build/ALPHA_FS/m5.debug
#   % cd /local/foo/build/ALPHA_FS; scons -C <path-to-src>/m5 m5.debug
#
# You can use 'scons -H' to print scons options.  If you're in this
# 'm5' directory (or use -u or -C to tell scons where to find this
# file), you can use 'scons -h' to print all the M5-specific build
# options as well.
#
###################################################

# Check for recent-enough Python and SCons versions.
try:
    # Really old versions of scons only take two options for the
    # function, so check once without the revision and once with the
    # revision, the first instance will fail for stuff other than
    # 0.98, and the second will fail for 0.98.0
    EnsureSConsVersion(0, 98)
    EnsureSConsVersion(0, 98, 1)
except SystemExit, e:
    print """
For more details, see:
    http://m5sim.org/wiki/index.php/Compiling_M5
"""
    raise

# We ensure the python version early because we have stuff that
# requires python 2.4
try:
    EnsurePythonVersion(2, 4)
except SystemExit, e:
    print """
You can use a non-default installation of the Python interpreter by
either (1) rearranging your PATH so that scons finds the non-default
'python' first or (2) explicitly invoking an alternative interpreter
on the scons script.

For more details, see:
    http://m5sim.org/wiki/index.php/Using_a_non-default_Python_installation
"""
    raise

# Global Python includes
import os
import re
import subprocess
import sys

from os import mkdir, environ
from os.path import abspath, basename, dirname, expanduser, normpath
from os.path import exists,  isdir, isfile
from os.path import join as joinpath, split as splitpath

# SCons includes
import SCons
import SCons.Node

extra_python_paths = [
    Dir('src/python').srcnode().abspath, # M5 includes
    Dir('ext/ply').srcnode().abspath, # ply is used by several files
    ]
    
sys.path[1:1] = extra_python_paths

from m5.util import compareVersions, readCommand

AddOption('--colors', dest='use_colors', action='store_true')
AddOption('--no-colors', dest='use_colors', action='store_false')
use_colors = GetOption('use_colors')

if use_colors:
    from m5.util.terminal import termcap
elif use_colors is None:
    # option unspecified; default behavior is to use colors iff isatty
    from m5.util.terminal import tty_termcap as termcap
else:
    from m5.util.terminal import no_termcap as termcap

########################################################################
#
# Set up the main build environment.
#
########################################################################
use_vars = set([ 'AS', 'AR', 'CC', 'CXX', 'HOME', 'LD_LIBRARY_PATH', 'PATH',
                 'PYTHONPATH', 'RANLIB' ])

use_env = {}
for key,val in os.environ.iteritems():
    if key in use_vars or key.startswith("M5"):
        use_env[key] = val

main = Environment(ENV=use_env)
main.root = Dir(".")         # The current directory (where this file lives).
main.srcdir = Dir("src")     # The source directory

# add useful python code PYTHONPATH so it can be used by subprocesses
# as well
main.AppendENVPath('PYTHONPATH', extra_python_paths)

########################################################################
#
# Mercurial Stuff.
#
# If the M5 directory is a mercurial repository, we should do some
# extra things.
#
########################################################################

hgdir = main.root.Dir(".hg")

mercurial_style_message = """
You're missing the M5 style hook.
Please install the hook so we can ensure that all code fits a common style.

All you'd need to do is add the following lines to your repository .hg/hgrc
or your personal .hgrc
----------------

[extensions]
style = %s/util/style.py

[hooks]
pretxncommit.style = python:style.check_style
pre-qrefresh.style = python:style.check_style
""" % (main.root)

mercurial_bin_not_found = """
Mercurial binary cannot be found, unfortunately this means that we
cannot easily determine the version of M5 that you are running and
this makes error messages more difficult to collect.  Please consider
installing mercurial if you choose to post an error message
"""

mercurial_lib_not_found = """
Mercurial libraries cannot be found, ignoring style hook
If you are actually a M5 developer, please fix this and
run the style hook. It is important.
"""

hg_info = "Unknown"
if hgdir.exists():
    # 1) Grab repository revision if we know it.
    cmd = "hg id -n -i -t -b"
    try:
        hg_info = readCommand(cmd, cwd=main.root.abspath).strip()
    except OSError:
        print mercurial_bin_not_found

    # 2) Ensure that the style hook is in place.
    try:
        ui = None
        if ARGUMENTS.get('IGNORE_STYLE') != 'True':
            from mercurial import ui
            ui = ui.ui()
    except ImportError:
        print mercurial_lib_not_found

    if ui is not None:
        ui.readconfig(hgdir.File('hgrc').abspath)
        style_hook = ui.config('hooks', 'pretxncommit.style', None)

        if not style_hook:
            print mercurial_style_message
            sys.exit(1)
else:
    print ".hg directory not found"

main['HG_INFO'] = hg_info

###################################################
#
# Figure out which configurations to set up based on the path(s) of
# the target(s).
#
###################################################

# Find default configuration & binary.
Default(environ.get('M5_DEFAULT_BINARY', 'build/ALPHA_SE/m5.debug'))

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

# Generate absolute paths to targets so we can see where the build dir is
if COMMAND_LINE_TARGETS:
    # Ask SCons which directory it was invoked from
    launch_dir = GetLaunchDir()
    # Make targets relative to invocation directory
    abs_targets = [ normpath(joinpath(launch_dir, str(x))) for x in \
                    COMMAND_LINE_TARGETS]
else:
    # Default targets are relative to root of tree
    abs_targets = [ normpath(joinpath(main.root.abspath, str(x))) for x in \
                    DEFAULT_TARGETS]


# Generate a list of the unique build roots and configs that the
# collected targets reference.
variant_paths = []
build_root = None
for t in abs_targets:
    path_dirs = t.split('/')
    try:
        build_top = rfind(path_dirs, 'build', -2)
    except:
        print "Error: no non-leaf 'build' dir found on target path", t
        Exit(1)
    this_build_root = joinpath('/',*path_dirs[:build_top+1])
    if not build_root:
        build_root = this_build_root
    else:
        if this_build_root != build_root:
            print "Error: build targets not under same build root\n"\
                  "  %s\n  %s" % (build_root, this_build_root)
            Exit(1)
    variant_path = joinpath('/',*path_dirs[:build_top+2])
    if variant_path not in variant_paths:
        variant_paths.append(variant_path)

# Make sure build_root exists (might not if this is the first build there)
if not isdir(build_root):
    mkdir(build_root)
main['BUILDROOT'] = build_root

Export('main')

main.SConsignFile(joinpath(build_root, "sconsign"))

# Default duplicate option is to use hard links, but this messes up
# when you use emacs to edit a file in the target dir, as emacs moves
# file to file~ then copies to file, breaking the link.  Symbolic
# (soft) links work better.
main.SetOption('duplicate', 'soft-copy')

#
# Set up global sticky variables... these are common to an entire build
# tree (not specific to a particular build like ALPHA_SE)
#

# Variable validators & converters for global sticky variables
def PathListMakeAbsolute(val):
    if not val:
        return val
    f = lambda p: abspath(expanduser(p))
    return ':'.join(map(f, val.split(':')))

def PathListAllExist(key, val, env):
    if not val:
        return
    paths = val.split(':')
    for path in paths:
        if not isdir(path):
            raise SCons.Errors.UserError("Path does not exist: '%s'" % path)

global_sticky_vars_file = joinpath(build_root, 'variables.global')

global_sticky_vars = Variables(global_sticky_vars_file, args=ARGUMENTS)
global_nonsticky_vars = Variables(args=ARGUMENTS)

global_sticky_vars.AddVariables(
    ('CC', 'C compiler', environ.get('CC', main['CC'])),
    ('CXX', 'C++ compiler', environ.get('CXX', main['CXX'])),
    ('BATCH', 'Use batch pool for build and tests', False),
    ('BATCH_CMD', 'Batch pool submission command name', 'qdo'),
    ('M5_BUILD_CACHE', 'Cache built objects in this directory', False),
    ('EXTRAS', 'Add Extra directories to the compilation', '',
     PathListAllExist, PathListMakeAbsolute),
    )

global_nonsticky_vars.AddVariables(
    ('VERBOSE', 'Print full tool command lines', False),
    ('update_ref', 'Update test reference outputs', False)
    )

# Update main environment with values from ARGUMENTS & global_sticky_vars_file
global_sticky_vars.Update(main)
global_nonsticky_vars.Update(main)
global_help_texts = {
    "global_sticky" : global_sticky_vars.GenerateHelpText(main),
    "global_nonsticky" : global_nonsticky_vars.GenerateHelpText(main)
}

# base help text
help_text = '''
Usage: scons [scons options] [build options] [target(s)]

Global sticky options:
%(global_sticky)s
Global nonsticky options:
%(global_nonsticky)s
''' % global_help_texts

# Save sticky variable settings back to current variables file
global_sticky_vars.Save(global_sticky_vars_file, main)

# Parse EXTRAS variable to build list of all directories where we're
# look for sources etc.  This list is exported as base_dir_list.
base_dir = main.srcdir.abspath
if main['EXTRAS']:
    extras_dir_list = main['EXTRAS'].split(':')
else:
    extras_dir_list = []

Export('base_dir')
Export('extras_dir_list')

# the ext directory should be on the #includes path
main.Append(CPPPATH=[Dir('ext')])

def strip_build_path(path, env):
    path = str(path)
    variant_base = env['BUILDROOT'] + os.path.sep
    if path.startswith(variant_base):
        path = path[len(variant_base):]
    elif path.startswith('build/'):
        path = path[6:]
    return path

# Generate a string of the form:
#   common/path/prefix/src1, src2 -> tgt1, tgt2
# to print while building.
class Transform(object):
    # all specific color settings should be here and nowhere else
    tool_color = termcap.Normal
    pfx_color = termcap.Yellow
    srcs_color = termcap.Yellow + termcap.Bold
    arrow_color = termcap.Blue + termcap.Bold
    tgts_color = termcap.Yellow + termcap.Bold

    def __init__(self, tool, max_sources=99):
        self.format = self.tool_color + (" [%8s] " % tool) \
                      + self.pfx_color + "%s" \
                      + self.srcs_color + "%s" \
                      + self.arrow_color + " -> " \
                      + self.tgts_color + "%s" \
                      + termcap.Normal
        self.max_sources = max_sources

    def __call__(self, target, source, env, for_signature=None):
        # truncate source list according to max_sources param
        source = source[0:self.max_sources]
        def strip(f):
            return strip_build_path(str(f), env)
        if len(source) > 0:
            srcs = map(strip, source)
        else:
            srcs = ['']
        tgts = map(strip, target)
        # surprisingly, os.path.commonprefix is a dumb char-by-char string
        # operation that has nothing to do with paths.
        com_pfx = os.path.commonprefix(srcs + tgts)
        com_pfx_len = len(com_pfx)
        if com_pfx:
            # do some cleanup and sanity checking on common prefix
            if com_pfx[-1] == ".":
                # prefix matches all but file extension: ok
                # back up one to change 'foo.cc -> o' to 'foo.cc -> .o'
                com_pfx = com_pfx[0:-1]
            elif com_pfx[-1] == "/":
                # common prefix is directory path: OK
                pass
            else:
                src0_len = len(srcs[0])
                tgt0_len = len(tgts[0])
                if src0_len == com_pfx_len:
                    # source is a substring of target, OK
                    pass
                elif tgt0_len == com_pfx_len:
                    # target is a substring of source, need to back up to
                    # avoid empty string on RHS of arrow
                    sep_idx = com_pfx.rfind(".")
                    if sep_idx != -1:
                        com_pfx = com_pfx[0:sep_idx]
                    else:
                        com_pfx = ''
                elif src0_len > com_pfx_len and srcs[0][com_pfx_len] == ".":
                    # still splitting at file extension: ok
                    pass
                else:
                    # probably a fluke; ignore it
                    com_pfx = ''
        # recalculate length in case com_pfx was modified
        com_pfx_len = len(com_pfx)
        def fmt(files):
            f = map(lambda s: s[com_pfx_len:], files)
            return ', '.join(f)
        return self.format % (com_pfx, fmt(srcs), fmt(tgts))

Export('Transform')


if main['VERBOSE']:
    def MakeAction(action, string, *args, **kwargs):
        return Action(action, *args, **kwargs)
else:
    MakeAction = Action
    main['CCCOMSTR']        = Transform("CC")
    main['CXXCOMSTR']       = Transform("CXX")
    main['ASCOMSTR']        = Transform("AS")
    main['SWIGCOMSTR']      = Transform("SWIG")
    main['ARCOMSTR']        = Transform("AR", 0)
    main['LINKCOMSTR']      = Transform("LINK", 0)
    main['RANLIBCOMSTR']    = Transform("RANLIB", 0)
    main['M4COMSTR']        = Transform("M4")
    main['SHCCCOMSTR']      = Transform("SHCC")
    main['SHCXXCOMSTR']     = Transform("SHCXX")
Export('MakeAction')

CXX_version = readCommand([main['CXX'],'--version'], exception=False)
CXX_V = readCommand([main['CXX'],'-V'], exception=False)

main['GCC'] = CXX_version and CXX_version.find('g++') >= 0
main['SUNCC'] = CXX_V and CXX_V.find('Sun C++') >= 0
main['ICC'] = CXX_V and CXX_V.find('Intel') >= 0
if main['GCC'] + main['SUNCC'] + main['ICC'] > 1:
    print 'Error: How can we have two at the same time?'
    Exit(1)

# Set up default C++ compiler flags
if main['GCC']:
    main.Append(CCFLAGS=['-pipe'])
    main.Append(CCFLAGS=['-fno-strict-aliasing'])
    main.Append(CCFLAGS=['-Wall', '-Wno-sign-compare', '-Wundef'])
    main.Append(CXXFLAGS=['-Wno-deprecated'])
    # Read the GCC version to check for versions with bugs
    # Note CCVERSION doesn't work here because it is run with the CC
    # before we override it from the command line
    gcc_version = readCommand([main['CXX'], '-dumpversion'], exception=False)
    if not compareVersions(gcc_version, '4.4.1') or \
       not compareVersions(gcc_version, '4.4.2'):
        print 'Info: Tree vectorizer in GCC 4.4.1 & 4.4.2 is buggy, disabling.'
        main.Append(CCFLAGS=['-fno-tree-vectorize'])
elif main['ICC']:
    pass #Fix me... add warning flags once we clean up icc warnings
elif main['SUNCC']:
    main.Append(CCFLAGS=['-Qoption ccfe'])
    main.Append(CCFLAGS=['-features=gcc'])
    main.Append(CCFLAGS=['-features=extensions'])
    main.Append(CCFLAGS=['-library=stlport4'])
    main.Append(CCFLAGS=['-xar'])
    #main.Append(CCFLAGS=['-instances=semiexplicit'])
else:
    print 'Error: Don\'t know what compiler options to use for your compiler.'
    print '       Please fix SConstruct and src/SConscript and try again.'
    Exit(1)

# Set up common yacc/bison flags (needed for Ruby)
main['YACCFLAGS'] = '-d'
main['YACCHXXFILESUFFIX'] = '.hh'

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

# Check for SWIG
if not main.has_key('SWIG'):
    print 'Error: SWIG utility not found.'
    print '       Please install (see http://www.swig.org) and retry.'
    Exit(1)

# Check for appropriate SWIG version
swig_version = readCommand(('swig', '-version'), exception='').split()
# First 3 words should be "SWIG Version x.y.z"
if len(swig_version) < 3 or \
        swig_version[0] != 'SWIG' or swig_version[1] != 'Version':
    print 'Error determining SWIG version.'
    Exit(1)

min_swig_version = '1.3.28'
if compareVersions(swig_version[2], min_swig_version) < 0:
    print 'Error: SWIG version', min_swig_version, 'or newer required.'
    print '       Installed version:', swig_version[2]
    Exit(1)

# Set up SWIG flags & scanner
swig_flags=Split('-c++ -python -modern -templatereduce $_CPPINCFLAGS')
main.Append(SWIGFLAGS=swig_flags)

# filter out all existing swig scanners, they mess up the dependency
# stuff for some reason
scanners = []
for scanner in main['SCANNERS']:
    skeys = scanner.skeys
    if skeys == '.i':
        continue

    if isinstance(skeys, (list, tuple)) and '.i' in skeys:
        continue

    scanners.append(scanner)

# add the new swig scanner that we like better
from SCons.Scanner import ClassicCPP as CPPScanner
swig_inc_re = '^[ \t]*[%,#][ \t]*(?:include|import)[ \t]*(<|")([^>"]+)(>|")'
scanners.append(CPPScanner("SwigScan", [ ".i" ], "CPPPATH", swig_inc_re))

# replace the scanners list that has what we want
main['SCANNERS'] = scanners

# Add a custom Check function to the Configure context so that we can
# figure out if the compiler adds leading underscores to global
# variables.  This is needed for the autogenerated asm files that we
# use for embedding the python code.
def CheckLeading(context):
    context.Message("Checking for leading underscore in global variables...")
    # 1) Define a global variable called x from asm so the C compiler
    #    won't change the symbol at all.
    # 2) Declare that variable.
    # 3) Use the variable
    #
    # If the compiler prepends an underscore, this will successfully
    # link because the external symbol 'x' will be called '_x' which
    # was defined by the asm statement.  If the compiler does not
    # prepend an underscore, this will not successfully link because
    # '_x' will have been defined by assembly, while the C portion of
    # the code will be trying to use 'x'
    ret = context.TryLink('''
        asm(".globl _x; _x: .byte 0");
        extern int x;
        int main() { return x; }
        ''', extension=".c")
    context.env.Append(LEADING_UNDERSCORE=ret)
    context.Result(ret)
    return ret

# Platform-specific configuration.  Note again that we assume that all
# builds under a given build root run on the same host platform.
conf = Configure(main,
                 conf_dir = joinpath(build_root, '.scons_config'),
                 log_file = joinpath(build_root, 'scons_config.log'),
                 custom_tests = { 'CheckLeading' : CheckLeading })

# Check for leading underscores.  Don't really need to worry either
# way so don't need to check the return code.
conf.CheckLeading()

# Check if we should compile a 64 bit binary on Mac OS X/Darwin
try:
    import platform
    uname = platform.uname()
    if uname[0] == 'Darwin' and compareVersions(uname[2], '9.0.0') >= 0:
        if int(readCommand('sysctl -n hw.cpu64bit_capable')[0]):
            main.Append(CCFLAGS=['-arch', 'x86_64'])
            main.Append(CFLAGS=['-arch', 'x86_64'])
            main.Append(LINKFLAGS=['-arch', 'x86_64'])
            main.Append(ASFLAGS=['-arch', 'x86_64'])
except:
    pass

# Recent versions of scons substitute a "Null" object for Configure()
# when configuration isn't necessary, e.g., if the "--help" option is
# present.  Unfortuantely this Null object always returns false,
# breaking all our configuration checks.  We replace it with our own
# more optimistic null object that returns True instead.
if not conf:
    def NullCheck(*args, **kwargs):
        return True

    class NullConf:
        def __init__(self, env):
            self.env = env
        def Finish(self):
            return self.env
        def __getattr__(self, mname):
            return NullCheck

    conf = NullConf(main)

# Find Python include and library directories for embedding the
# interpreter.  For consistency, we will use the same Python
# installation used to run scons (and thus this script).  If you want
# to link in an alternate version, see above for instructions on how
# to invoke scons with a different copy of the Python interpreter.
from distutils import sysconfig

py_getvar = sysconfig.get_config_var

py_debug = getattr(sys, 'pydebug', False)
py_version = 'python' + py_getvar('VERSION') + (py_debug and "_d" or "")

py_general_include = sysconfig.get_python_inc()
py_platform_include = sysconfig.get_python_inc(plat_specific=True)
py_includes = [ py_general_include ]
if py_platform_include != py_general_include:
    py_includes.append(py_platform_include)

py_lib_path = [ py_getvar('LIBDIR') ]
# add the prefix/lib/pythonX.Y/config dir, but only if there is no
# shared library in prefix/lib/.
if not py_getvar('Py_ENABLE_SHARED'):
    py_lib_path.append(py_getvar('LIBPL'))

py_libs = []
for lib in py_getvar('LIBS').split() + py_getvar('SYSLIBS').split():
    assert lib.startswith('-l')
    lib = lib[2:]   
    if lib not in py_libs:
        py_libs.append(lib)
py_libs.append(py_version)

main.Append(CPPPATH=py_includes)
main.Append(LIBPATH=py_lib_path)

# Cache build files in the supplied directory.
if main['M5_BUILD_CACHE']:
    print 'Using build cache located at', main['M5_BUILD_CACHE']
    CacheDir(main['M5_BUILD_CACHE'])


# verify that this stuff works
if not conf.CheckHeader('Python.h', '<>'):
    print "Error: can't find Python.h header in", py_includes
    Exit(1)

for lib in py_libs:
    if not conf.CheckLib(lib):
        print "Error: can't find library %s required by python" % lib
        Exit(1)

# On Solaris you need to use libsocket for socket ops
if not conf.CheckLibWithHeader(None, 'sys/socket.h', 'C++', 'accept(0,0,0);'):
   if not conf.CheckLibWithHeader('socket', 'sys/socket.h', 'C++', 'accept(0,0,0);'):
       print "Can't find library with socket calls (e.g. accept())"
       Exit(1)

# Check for zlib.  If the check passes, libz will be automatically
# added to the LIBS environment variable.
if not conf.CheckLibWithHeader('z', 'zlib.h', 'C++','zlibVersion();'):
    print 'Error: did not find needed zlib compression library '\
          'and/or zlib.h header file.'
    print '       Please install zlib and try again.'
    Exit(1)

# Check for librt.
have_posix_clock = \
    conf.CheckLibWithHeader(None, 'time.h', 'C',
                            'clock_nanosleep(0,0,NULL,NULL);') or \
    conf.CheckLibWithHeader('rt', 'time.h', 'C',
                            'clock_nanosleep(0,0,NULL,NULL);')

if not have_posix_clock:
    print "Can't find library for POSIX clocks."

# Check for <fenv.h> (C99 FP environment control)
have_fenv = conf.CheckHeader('fenv.h', '<>')
if not have_fenv:
    print "Warning: Header file <fenv.h> not found."
    print "         This host has no IEEE FP rounding mode control."

######################################################################
#
# Check for mysql.
#
mysql_config = WhereIs('mysql_config')
have_mysql = bool(mysql_config)

# Check MySQL version.
if have_mysql:
    mysql_version = readCommand(mysql_config + ' --version')
    min_mysql_version = '4.1'
    if compareVersions(mysql_version, min_mysql_version) < 0:
        print 'Warning: MySQL', min_mysql_version, 'or newer required.'
        print '         Version', mysql_version, 'detected.'
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

######################################################################
#
# Finish the configuration
#
main = conf.Finish()

######################################################################
#
# Collect all non-global variables
#

# Define the universe of supported ISAs
all_isa_list = [ ]
Export('all_isa_list')

class CpuModel(object):
    '''The CpuModel class encapsulates everything the ISA parser needs to
    know about a particular CPU model.'''

    # Dict of available CPU model objects.  Accessible as CpuModel.dict.
    dict = {}
    list = []
    defaults = []

    # Constructor.  Automatically adds models to CpuModel.dict.
    def __init__(self, name, filename, includes, strings, default=False):
        self.name = name           # name of model
        self.filename = filename   # filename for output exec code
        self.includes = includes   # include files needed in exec file
        # The 'strings' dict holds all the per-CPU symbols we can
        # substitute into templates etc.
        self.strings = strings

        # This cpu is enabled by default
        self.default = default

        # Add self to dict
        if name in CpuModel.dict:
            raise AttributeError, "CpuModel '%s' already registered" % name
        CpuModel.dict[name] = self
        CpuModel.list.append(name)

Export('CpuModel')

# Sticky variables get saved in the variables file so they persist from
# one invocation to the next (unless overridden, in which case the new
# value becomes sticky).
sticky_vars = Variables(args=ARGUMENTS)
Export('sticky_vars')

# Sticky variables that should be exported
export_vars = []
Export('export_vars')

# Walk the tree and execute all SConsopts scripts that wil add to the
# above variables
for bdir in [ base_dir ] + extras_dir_list:
    for root, dirs, files in os.walk(bdir):
        if 'SConsopts' in files:
            print "Reading", joinpath(root, 'SConsopts')
            SConscript(joinpath(root, 'SConsopts'))

all_isa_list.sort()

sticky_vars.AddVariables(
    EnumVariable('TARGET_ISA', 'Target ISA', 'alpha', all_isa_list),
    BoolVariable('FULL_SYSTEM', 'Full-system support', False),
    ListVariable('CPU_MODELS', 'CPU models',
                 sorted(n for n,m in CpuModel.dict.iteritems() if m.default),
                 sorted(CpuModel.list)),
    BoolVariable('NO_FAST_ALLOC', 'Disable fast object allocator', False),
    BoolVariable('FAST_ALLOC_DEBUG', 'Enable fast object allocator debugging',
                 False),
    BoolVariable('FAST_ALLOC_STATS', 'Enable fast object allocator statistics',
                 False),
    BoolVariable('EFENCE', 'Link with Electric Fence malloc debugger',
                 False),
    BoolVariable('SS_COMPATIBLE_FP',
                 'Make floating-point results compatible with SimpleScalar',
                 False),
    BoolVariable('USE_SSE2',
                 'Compile for SSE2 (-msse2) to get IEEE FP on x86 hosts',
                 False),
    BoolVariable('USE_MYSQL', 'Use MySQL for stats output', have_mysql),
    BoolVariable('USE_POSIX_CLOCK', 'Use POSIX Clocks', have_posix_clock),
    BoolVariable('USE_FENV', 'Use <fenv.h> IEEE mode control', have_fenv),
    BoolVariable('USE_CHECKER', 'Use checker for detailed CPU models', False),
    BoolVariable('CP_ANNOTATE', 'Enable critical path annotation capability', False),
    BoolVariable('RUBY', 'Build with Ruby', False),
    )

# These variables get exported to #defines in config/*.hh (see src/SConscript).
export_vars += ['FULL_SYSTEM', 'USE_FENV', 'USE_MYSQL',
                'NO_FAST_ALLOC', 'FAST_ALLOC_DEBUG', 'FAST_ALLOC_STATS',
                'SS_COMPATIBLE_FP', 'USE_CHECKER', 'TARGET_ISA', 'CP_ANNOTATE',
                'USE_POSIX_CLOCK' ]

###################################################
#
# Define a SCons builder for configuration flag headers.
#
###################################################

# This function generates a config header file that #defines the
# variable symbol to the current variable setting (0 or 1).  The source
# operands are the name of the variable and a Value node containing the
# value of the variable.
def build_config_file(target, source, env):
    (variable, value) = [s.get_contents() for s in source]
    f = file(str(target[0]), 'w')
    print >> f, '#define', variable, value
    f.close()
    return None

# Generate the message to be printed when building the config file.
def build_config_file_string(target, source, env):
    (variable, value) = [s.get_contents() for s in source]
    return "Defining %s as %s in %s." % (variable, value, target[0])

# Combine the two functions into a scons Action object.
config_action = Action(build_config_file, build_config_file_string)

# The emitter munges the source & target node lists to reflect what
# we're really doing.
def config_emitter(target, source, env):
    # extract variable name from Builder arg
    variable = str(target[0])
    # True target is config header file
    target = joinpath('config', variable.lower() + '.hh')
    val = env[variable]
    if isinstance(val, bool):
        # Force value to 0/1
        val = int(val)
    elif isinstance(val, str):
        val = '"' + val + '"'

    # Sources are variable name & value (packaged in SCons Value nodes)
    return ([target], [Value(variable), Value(val)])

config_builder = Builder(emitter = config_emitter, action = config_action)

main.Append(BUILDERS = { 'ConfigFile' : config_builder })

# libelf build is shared across all configs in the build root.
main.SConscript('ext/libelf/SConscript',
                variant_dir = joinpath(build_root, 'libelf'))

# gzstream build is shared across all configs in the build root.
main.SConscript('ext/gzstream/SConscript',
                variant_dir = joinpath(build_root, 'gzstream'))

###################################################
#
# This function is used to set up a directory with switching headers
#
###################################################

main['ALL_ISA_LIST'] = all_isa_list
def make_switching_dir(dname, switch_headers, env):
    # Generate the header.  target[0] is the full path of the output
    # header to generate.  'source' is a dummy variable, since we get the
    # list of ISAs from env['ALL_ISA_LIST'].
    def gen_switch_hdr(target, source, env):
        fname = str(target[0])
        f = open(fname, 'w')
        isa = env['TARGET_ISA'].lower()
        print >>f, '#include "%s/%s/%s"' % (dname, isa, basename(fname))
        f.close()

    # Build SCons Action object. 'varlist' specifies env vars that this
    # action depends on; when env['ALL_ISA_LIST'] changes these actions
    # should get re-executed.
    switch_hdr_action = MakeAction(gen_switch_hdr,
                          Transform("GENERATE"), varlist=['ALL_ISA_LIST'])

    # Instantiate actions for each header
    for hdr in switch_headers:
        env.Command(hdr, [], switch_hdr_action)
Export('make_switching_dir')

###################################################
#
# Define build environments for selected configurations.
#
###################################################

for variant_path in variant_paths:
    print "Building in", variant_path

    # Make a copy of the build-root environment to use for this config.
    env = main.Clone()
    env['BUILDDIR'] = variant_path

    # variant_dir is the tail component of build path, and is used to
    # determine the build parameters (e.g., 'ALPHA_SE')
    (build_root, variant_dir) = splitpath(variant_path)

    # Set env variables according to the build directory config.
    sticky_vars.files = []
    # Variables for $BUILD_ROOT/$VARIANT_DIR are stored in
    # $BUILD_ROOT/variables/$VARIANT_DIR so you can nuke
    # $BUILD_ROOT/$VARIANT_DIR without losing your variables settings.
    current_vars_file = joinpath(build_root, 'variables', variant_dir)
    if isfile(current_vars_file):
        sticky_vars.files.append(current_vars_file)
        print "Using saved variables file %s" % current_vars_file
    else:
        # Build dir-specific variables file doesn't exist.

        # Make sure the directory is there so we can create it later
        opt_dir = dirname(current_vars_file)
        if not isdir(opt_dir):
            mkdir(opt_dir)

        # Get default build variables from source tree.  Variables are
        # normally determined by name of $VARIANT_DIR, but can be
        # overriden by 'default=' arg on command line.
        default_vars_file = joinpath('build_opts',
                                     ARGUMENTS.get('default', variant_dir))
        if isfile(default_vars_file):
            sticky_vars.files.append(default_vars_file)
            print "Variables file %s not found,\n  using defaults in %s" \
                  % (current_vars_file, default_vars_file)
        else:
            print "Error: cannot find variables file %s or %s" \
                  % (current_vars_file, default_vars_file)
            Exit(1)

    # Apply current variable settings to env
    sticky_vars.Update(env)

    help_text += "\nSticky variables for %s:\n" % variant_dir \
                 + sticky_vars.GenerateHelpText(env)

    # Process variable settings.

    if not have_fenv and env['USE_FENV']:
        print "Warning: <fenv.h> not available; " \
              "forcing USE_FENV to False in", variant_dir + "."
        env['USE_FENV'] = False

    if not env['USE_FENV']:
        print "Warning: No IEEE FP rounding mode control in", variant_dir + "."
        print "         FP results may deviate slightly from other platforms."

    if env['EFENCE']:
        env.Append(LIBS=['efence'])

    if env['USE_MYSQL']:
        if not have_mysql:
            print "Warning: MySQL not available; " \
                  "forcing USE_MYSQL to False in", variant_dir + "."
            env['USE_MYSQL'] = False
        else:
            print "Compiling in", variant_dir, "with MySQL support."
            env.ParseConfig(mysql_config_libs)
            env.ParseConfig(mysql_config_include)

    # Save sticky variable settings back to current variables file
    sticky_vars.Save(current_vars_file, env)

    if env['USE_SSE2']:
        env.Append(CCFLAGS=['-msse2'])

    # The src/SConscript file sets up the build rules in 'env' according
    # to the configured variables.  It returns a list of environments,
    # one for each variant build (debug, opt, etc.)
    envList = SConscript('src/SConscript', variant_dir = variant_path,
                         exports = 'env')

    # Set up the regression tests for each build.
    for e in envList:
        SConscript('tests/SConscript',
                   variant_dir = joinpath(variant_path, 'tests', e.Label),
                   exports = { 'env' : e }, duplicate = False)

Help(help_text)
