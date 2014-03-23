# -*- mode:python -*-

# Copyright (c) 2013 ARM Limited
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
#
# Authors: Steve Reinhardt
#          Nathan Binkert

###################################################
#
# SCons top-level build description (SConstruct) file.
#
# While in this directory ('gem5'), just type 'scons' to build the default
# configuration (see below), or type 'scons build/<CONFIG>/<binary>'
# to build some other configuration (e.g., 'build/ALPHA/gem5.opt' for
# the optimized full-system version).
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
#   % cd <path-to-src>/gem5 ; scons build/ALPHA/gem5.debug
#   % cd <path-to-src>/gem5/build/ALPHA; scons -u gem5.debug
#
#   The following two commands are equivalent and demonstrate building
#   in a directory outside of the source tree.  The '-C' option tells
#   scons to chdir to the specified directory to find this SConstruct
#   file.
#   % cd <path-to-src>/gem5 ; scons /local/foo/build/ALPHA/gem5.debug
#   % cd /local/foo/build/ALPHA; scons -C <path-to-src>/gem5 gem5.debug
#
# You can use 'scons -H' to print scons options.  If you're in this
# 'gem5' directory (or use -u or -C to tell scons where to find this
# file), you can use 'scons -h' to print all the gem5-specific build
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
    http://gem5.org/Dependencies
"""
    raise

# We ensure the python version early because because python-config
# requires python 2.5
try:
    EnsurePythonVersion(2, 5)
except SystemExit, e:
    print """
You can use a non-default installation of the Python interpreter by
rearranging your PATH so that scons finds the non-default 'python' and
'python-config' first.

For more details, see:
    http://gem5.org/wiki/index.php/Using_a_non-default_Python_installation
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
    Dir('src/python').srcnode().abspath, # gem5 includes
    Dir('ext/ply').srcnode().abspath, # ply is used by several files
    ]

sys.path[1:1] = extra_python_paths

from m5.util import compareVersions, readCommand
from m5.util.terminal import get_termcap

help_texts = {
    "options" : "",
    "global_vars" : "",
    "local_vars" : ""
}

Export("help_texts")


# There's a bug in scons in that (1) by default, the help texts from
# AddOption() are supposed to be displayed when you type 'scons -h'
# and (2) you can override the help displayed by 'scons -h' using the
# Help() function, but these two features are incompatible: once
# you've overridden the help text using Help(), there's no way to get
# at the help texts from AddOptions.  See:
#     http://scons.tigris.org/issues/show_bug.cgi?id=2356
#     http://scons.tigris.org/issues/show_bug.cgi?id=2611
# This hack lets us extract the help text from AddOptions and
# re-inject it via Help().  Ideally someday this bug will be fixed and
# we can just use AddOption directly.
def AddLocalOption(*args, **kwargs):
    col_width = 30

    help = "  " + ", ".join(args)
    if "help" in kwargs:
        length = len(help)
        if length >= col_width:
            help += "\n" + " " * col_width
        else:
            help += " " * (col_width - length)
        help += kwargs["help"]
    help_texts["options"] += help + "\n"

    AddOption(*args, **kwargs)

AddLocalOption('--colors', dest='use_colors', action='store_true',
               help="Add color to abbreviated scons output")
AddLocalOption('--no-colors', dest='use_colors', action='store_false',
               help="Don't add color to abbreviated scons output")
AddLocalOption('--default', dest='default', type='string', action='store',
               help='Override which build_opts file to use for defaults')
AddLocalOption('--ignore-style', dest='ignore_style', action='store_true',
               help='Disable style checking hooks')
AddLocalOption('--no-lto', dest='no_lto', action='store_true',
               help='Disable Link-Time Optimization for fast')
AddLocalOption('--update-ref', dest='update_ref', action='store_true',
               help='Update test reference outputs')
AddLocalOption('--verbose', dest='verbose', action='store_true',
               help='Print full tool command lines')

termcap = get_termcap(GetOption('use_colors'))

########################################################################
#
# Set up the main build environment.
#
########################################################################

# export TERM so that clang reports errors in color
use_vars = set([ 'AS', 'AR', 'CC', 'CXX', 'HOME', 'LD_LIBRARY_PATH',
                 'LIBRARY_PATH', 'PATH', 'PKG_CONFIG_PATH', 'PROTOC',
                 'PYTHONPATH', 'RANLIB', 'SWIG', 'TERM' ])

use_prefixes = [
    "M5",           # M5 configuration (e.g., path to kernels)
    "DISTCC_",      # distcc (distributed compiler wrapper) configuration
    "CCACHE_",      # ccache (caching compiler wrapper) configuration
    "CCC_",         # clang static analyzer configuration
    ]

use_env = {}
for key,val in os.environ.iteritems():
    if key in use_vars or \
            any([key.startswith(prefix) for prefix in use_prefixes]):
        use_env[key] = val

main = Environment(ENV=use_env)
main.Decider('MD5-timestamp')
main.root = Dir(".")         # The current directory (where this file lives).
main.srcdir = Dir("src")     # The source directory

main_dict_keys = main.Dictionary().keys()

# Check that we have a C/C++ compiler
if not ('CC' in main_dict_keys and 'CXX' in main_dict_keys):
    print "No C++ compiler installed (package g++ on Ubuntu and RedHat)"
    Exit(1)

# Check that swig is present
if not 'SWIG' in main_dict_keys:
    print "swig is not installed (package swig on Ubuntu and RedHat)"
    Exit(1)

# add useful python code PYTHONPATH so it can be used by subprocesses
# as well
main.AppendENVPath('PYTHONPATH', extra_python_paths)

########################################################################
#
# Mercurial Stuff.
#
# If the gem5 directory is a mercurial repository, we should do some
# extra things.
#
########################################################################

hgdir = main.root.Dir(".hg")

mercurial_style_message = """
You're missing the gem5 style hook, which automatically checks your code
against the gem5 style rules on hg commit and qrefresh commands.  This
script will now install the hook in your .hg/hgrc file.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_hook = """
# The following lines were automatically added by gem5/SConstruct
# to provide the gem5 style-checking hooks
[extensions]
style = %s/util/style.py

[hooks]
pretxncommit.style = python:style.check_style
pre-qrefresh.style = python:style.check_style
# End of SConstruct additions

""" % (main.root.abspath)

mercurial_lib_not_found = """
Mercurial libraries cannot be found, ignoring style hook.  If
you are a gem5 developer, please fix this and run the style
hook. It is important.
"""

# Check for style hook and prompt for installation if it's not there.
# Skip this if --ignore-style was specified, there's no .hg dir to
# install a hook in, or there's no interactive terminal to prompt.
if not GetOption('ignore_style') and hgdir.exists() and sys.stdin.isatty():
    style_hook = True
    try:
        from mercurial import ui
        ui = ui.ui()
        ui.readconfig(hgdir.File('hgrc').abspath)
        style_hook = ui.config('hooks', 'pretxncommit.style', None) and \
                     ui.config('hooks', 'pre-qrefresh.style', None)
    except ImportError:
        print mercurial_lib_not_found

    if not style_hook:
        print mercurial_style_message,
        # continue unless user does ctrl-c/ctrl-d etc.
        try:
            raw_input()
        except:
            print "Input exception, exiting scons.\n"
            sys.exit(1)
        hgrc_path = '%s/.hg/hgrc' % main.root.abspath
        print "Adding style hook to", hgrc_path, "\n"
        try:
            hgrc = open(hgrc_path, 'a')
            hgrc.write(mercurial_style_hook)
            hgrc.close()
        except:
            print "Error updating", hgrc_path
            sys.exit(1)


###################################################
#
# Figure out which configurations to set up based on the path(s) of
# the target(s).
#
###################################################

# Find default configuration & binary.
Default(environ.get('M5_DEFAULT_BINARY', 'build/ALPHA/gem5.debug'))

# helper function: find last occurrence of element in list
def rfind(l, elt, offs = -1):
    for i in range(len(l)+offs, 0, -1):
        if l[i] == elt:
            return i
    raise ValueError, "element not found"

# Take a list of paths (or SCons Nodes) and return a list with all
# paths made absolute and ~-expanded.  Paths will be interpreted
# relative to the launch directory unless a different root is provided
def makePathListAbsolute(path_list, root=GetLaunchDir()):
    return [abspath(joinpath(root, expanduser(str(p))))
            for p in path_list]

# Each target must have 'build' in the interior of the path; the
# directory below this will determine the build parameters.  For
# example, for target 'foo/bar/build/ALPHA_SE/arch/alpha/blah.do' we
# recognize that ALPHA_SE specifies the configuration because it
# follow 'build' in the build path.

# The funky assignment to "[:]" is needed to replace the list contents
# in place rather than reassign the symbol to a new list, which
# doesn't work (obviously!).
BUILD_TARGETS[:] = makePathListAbsolute(BUILD_TARGETS)

# Generate a list of the unique build roots and configs that the
# collected targets reference.
variant_paths = []
build_root = None
for t in BUILD_TARGETS:
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

global_vars_file = joinpath(build_root, 'variables.global')

global_vars = Variables(global_vars_file, args=ARGUMENTS)

global_vars.AddVariables(
    ('CC', 'C compiler', environ.get('CC', main['CC'])),
    ('CXX', 'C++ compiler', environ.get('CXX', main['CXX'])),
    ('SWIG', 'SWIG tool', environ.get('SWIG', main['SWIG'])),
    ('PROTOC', 'protoc tool', environ.get('PROTOC', 'protoc')),
    ('BATCH', 'Use batch pool for build and tests', False),
    ('BATCH_CMD', 'Batch pool submission command name', 'qdo'),
    ('M5_BUILD_CACHE', 'Cache built objects in this directory', False),
    ('EXTRAS', 'Add extra directories to the compilation', '')
    )

# Update main environment with values from ARGUMENTS & global_vars_file
global_vars.Update(main)
help_texts["global_vars"] += global_vars.GenerateHelpText(main)

# Save sticky variable settings back to current variables file
global_vars.Save(global_vars_file, main)

# Parse EXTRAS variable to build list of all directories where we're
# look for sources etc.  This list is exported as extras_dir_list.
base_dir = main.srcdir.abspath
if main['EXTRAS']:
    extras_dir_list = makePathListAbsolute(main['EXTRAS'].split(':'))
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

# enable the regression script to use the termcap
main['TERMCAP'] = termcap

if GetOption('verbose'):
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

# Initialize the Link-Time Optimization (LTO) flags
main['LTO_CCFLAGS'] = []
main['LTO_LDFLAGS'] = []

# According to the readme, tcmalloc works best if the compiler doesn't
# assume that we're using the builtin malloc and friends. These flags
# are compiler-specific, so we need to set them after we detect which
# compiler we're using.
main['TCMALLOC_CCFLAGS'] = []

CXX_version = readCommand([main['CXX'],'--version'], exception=False)
CXX_V = readCommand([main['CXX'],'-V'], exception=False)

main['GCC'] = CXX_version and CXX_version.find('g++') >= 0
main['CLANG'] = CXX_version and CXX_version.find('clang') >= 0
if main['GCC'] + main['CLANG'] > 1:
    print 'Error: How can we have two at the same time?'
    Exit(1)

# Set up default C++ compiler flags
if main['GCC'] or main['CLANG']:
    # As gcc and clang share many flags, do the common parts here
    main.Append(CCFLAGS=['-pipe'])
    main.Append(CCFLAGS=['-fno-strict-aliasing'])
    # Enable -Wall and then disable the few warnings that we
    # consistently violate
    main.Append(CCFLAGS=['-Wall', '-Wno-sign-compare', '-Wundef'])
    # We always compile using C++11, but only gcc >= 4.7 and clang 3.1
    # actually use that name, so we stick with c++0x
    main.Append(CXXFLAGS=['-std=c++0x'])
    # Add selected sanity checks from -Wextra
    main.Append(CXXFLAGS=['-Wmissing-field-initializers',
                          '-Woverloaded-virtual'])
else:
    print termcap.Yellow + termcap.Bold + 'Error' + termcap.Normal,
    print "Don't know what compiler options to use for your compiler."
    print termcap.Yellow + '       compiler:' + termcap.Normal, main['CXX']
    print termcap.Yellow + '       version:' + termcap.Normal,
    if not CXX_version:
        print termcap.Yellow + termcap.Bold + "COMMAND NOT FOUND!" +\
               termcap.Normal
    else:
        print CXX_version.replace('\n', '<nl>')
    print "       If you're trying to use a compiler other than GCC"
    print "       or clang, there appears to be something wrong with your"
    print "       environment."
    print "       "
    print "       If you are trying to use a compiler other than those listed"
    print "       above you will need to ease fix SConstruct and "
    print "       src/SConscript to support that compiler."
    Exit(1)

if main['GCC']:
    # Check for a supported version of gcc, >= 4.4 is needed for c++0x
    # support. See http://gcc.gnu.org/projects/cxx0x.html for details
    gcc_version = readCommand([main['CXX'], '-dumpversion'], exception=False)
    if compareVersions(gcc_version, "4.4") < 0:
        print 'Error: gcc version 4.4 or newer required.'
        print '       Installed version:', gcc_version
        Exit(1)

    main['GCC_VERSION'] = gcc_version

    # Check for versions with bugs
    if not compareVersions(gcc_version, '4.4.1') or \
       not compareVersions(gcc_version, '4.4.2'):
        print 'Info: Tree vectorizer in GCC 4.4.1 & 4.4.2 is buggy, disabling.'
        main.Append(CCFLAGS=['-fno-tree-vectorize'])

    # LTO support is only really working properly from 4.6 and beyond
    if compareVersions(gcc_version, '4.6') >= 0:
        # Add the appropriate Link-Time Optimization (LTO) flags
        # unless LTO is explicitly turned off. Note that these flags
        # are only used by the fast target.
        if not GetOption('no_lto'):
            # Pass the LTO flag when compiling to produce GIMPLE
            # output, we merely create the flags here and only append
            # them later/
            main['LTO_CCFLAGS'] = ['-flto=%d' % GetOption('num_jobs')]

            # Use the same amount of jobs for LTO as we are running
            # scons with, we hardcode the use of the linker plugin
            # which requires either gold or GNU ld >= 2.21
            main['LTO_LDFLAGS'] = ['-flto=%d' % GetOption('num_jobs'),
                                   '-fuse-linker-plugin']

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin-malloc', '-fno-builtin-calloc',
                                  '-fno-builtin-realloc', '-fno-builtin-free'])

elif main['CLANG']:
    # Check for a supported version of clang, >= 2.9 is needed to
    # support similar features as gcc 4.4. See
    # http://clang.llvm.org/cxx_status.html for details
    clang_version_re = re.compile(".* version (\d+\.\d+)")
    clang_version_match = clang_version_re.search(CXX_version)
    if (clang_version_match):
        clang_version = clang_version_match.groups()[0]
        if compareVersions(clang_version, "2.9") < 0:
            print 'Error: clang version 2.9 or newer required.'
            print '       Installed version:', clang_version
            Exit(1)
    else:
        print 'Error: Unable to determine clang version.'
        Exit(1)

    # clang has a few additional warnings that we disable,
    # tautological comparisons are allowed due to unsigned integers
    # being compared to constants that happen to be 0, and extraneous
    # parantheses are allowed due to Ruby's printing of the AST,
    # finally self assignments are allowed as the generated CPU code
    # is relying on this
    main.Append(CCFLAGS=['-Wno-tautological-compare',
                         '-Wno-parentheses',
                         '-Wno-self-assign'])

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin'])

    # On Mac OS X/Darwin we need to also use libc++ (part of XCode) as
    # opposed to libstdc++, as the later is dated.
    if sys.platform == "darwin":
        main.Append(CXXFLAGS=['-stdlib=libc++'])
        main.Append(LIBS=['c++'])

else:
    print termcap.Yellow + termcap.Bold + 'Error' + termcap.Normal,
    print "Don't know what compiler options to use for your compiler."
    print termcap.Yellow + '       compiler:' + termcap.Normal, main['CXX']
    print termcap.Yellow + '       version:' + termcap.Normal,
    if not CXX_version:
        print termcap.Yellow + termcap.Bold + "COMMAND NOT FOUND!" +\
               termcap.Normal
    else:
        print CXX_version.replace('\n', '<nl>')
    print "       If you're trying to use a compiler other than GCC"
    print "       or clang, there appears to be something wrong with your"
    print "       environment."
    print "       "
    print "       If you are trying to use a compiler other than those listed"
    print "       above you will need to ease fix SConstruct and "
    print "       src/SConscript to support that compiler."
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

# Check for the protobuf compiler
protoc_version = readCommand([main['PROTOC'], '--version'],
                             exception='').split()

# First two words should be "libprotoc x.y.z"
if len(protoc_version) < 2 or protoc_version[0] != 'libprotoc':
    print termcap.Yellow + termcap.Bold + \
        'Warning: Protocol buffer compiler (protoc) not found.\n' + \
        '         Please install protobuf-compiler for tracing support.' + \
        termcap.Normal
    main['PROTOC'] = False
else:
    # Based on the availability of the compress stream wrappers,
    # require 2.1.0
    min_protoc_version = '2.1.0'
    if compareVersions(protoc_version[1], min_protoc_version) < 0:
        print termcap.Yellow + termcap.Bold + \
            'Warning: protoc version', min_protoc_version, \
            'or newer required.\n' + \
            '         Installed version:', protoc_version[1], \
            termcap.Normal
        main['PROTOC'] = False
    else:
        # Attempt to determine the appropriate include path and
        # library path using pkg-config, that means we also need to
        # check for pkg-config. Note that it is possible to use
        # protobuf without the involvement of pkg-config. Later on we
        # check go a library config check and at that point the test
        # will fail if libprotobuf cannot be found.
        if readCommand(['pkg-config', '--version'], exception=''):
            try:
                # Attempt to establish what linking flags to add for protobuf
                # using pkg-config
                main.ParseConfig('pkg-config --cflags --libs-only-L protobuf')
            except:
                print termcap.Yellow + termcap.Bold + \
                    'Warning: pkg-config could not get protobuf flags.' + \
                    termcap.Normal

# Check for SWIG
if not main.has_key('SWIG'):
    print 'Error: SWIG utility not found.'
    print '       Please install (see http://www.swig.org) and retry.'
    Exit(1)

# Check for appropriate SWIG version
swig_version = readCommand([main['SWIG'], '-version'], exception='').split()
# First 3 words should be "SWIG Version x.y.z"
if len(swig_version) < 3 or \
        swig_version[0] != 'SWIG' or swig_version[1] != 'Version':
    print 'Error determining SWIG version.'
    Exit(1)

min_swig_version = '1.3.34'
if compareVersions(swig_version[2], min_swig_version) < 0:
    print 'Error: SWIG version', min_swig_version, 'or newer required.'
    print '       Installed version:', swig_version[2]
    Exit(1)

# Older versions of swig do not play well with more recent versions of
# gcc due to assumptions on implicit includes (cstddef) and use of
# namespaces
if main['GCC'] and compareVersions(gcc_version, '4.6') > 0 and \
        compareVersions(swig_version[2], '2') < 0:
    print '\n' + termcap.Yellow + termcap.Bold + \
        'Warning: SWIG 1.x cause issues with gcc 4.6 and later.\n' + \
        termcap.Normal + \
        'Use SWIG 2.x to avoid assumptions on implicit includes\n' + \
        'and use of namespaces\n'

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

# Add a custom Check function to test for structure members.
def CheckMember(context, include, decl, member, include_quotes="<>"):
    context.Message("Checking for member %s in %s..." %
                    (member, decl))
    text = """
#include %(header)s
int main(){
  %(decl)s test;
  (void)test.%(member)s;
  return 0;
};
""" % { "header" : include_quotes[0] + include + include_quotes[1],
        "decl" : decl,
        "member" : member,
        }

    ret = context.TryCompile(text, extension=".cc")
    context.Result(ret)
    return ret

# Platform-specific configuration.  Note again that we assume that all
# builds under a given build root run on the same host platform.
conf = Configure(main,
                 conf_dir = joinpath(build_root, '.scons_config'),
                 log_file = joinpath(build_root, 'scons_config.log'),
                 custom_tests = {
        'CheckLeading' : CheckLeading,
        'CheckMember' : CheckMember,
        })

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

# Cache build files in the supplied directory.
if main['M5_BUILD_CACHE']:
    print 'Using build cache located at', main['M5_BUILD_CACHE']
    CacheDir(main['M5_BUILD_CACHE'])

# Find Python include and library directories for embedding the
# interpreter. We rely on python-config to resolve the appropriate
# includes and linker flags. ParseConfig does not seem to understand
# the more exotic linker flags such as -Xlinker and -export-dynamic so
# we add them explicitly below. If you want to link in an alternate
# version of python, see above for instructions on how to invoke
# scons with the appropriate PATH set.
py_includes = readCommand(['python-config', '--includes'],
                          exception='').split()
# Strip the -I from the include folders before adding them to the
# CPPPATH
main.Append(CPPPATH=map(lambda inc: inc[2:], py_includes))

# Read the linker flags and split them into libraries and other link
# flags. The libraries are added later through the call the CheckLib.
py_ld_flags = readCommand(['python-config', '--ldflags'], exception='').split()
py_libs = []
for lib in py_ld_flags:
     if not lib.startswith('-l'):
         main.Append(LINKFLAGS=[lib])
     else:
         lib = lib[2:]
         if lib not in py_libs:
             py_libs.append(lib)

# verify that this stuff works
if not conf.CheckHeader('Python.h', '<>'):
    print "Error: can't find Python.h header in", py_includes
    print "Install Python headers (package python-dev on Ubuntu and RedHat)"
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

# If we have the protobuf compiler, also make sure we have the
# development libraries. If the check passes, libprotobuf will be
# automatically added to the LIBS environment variable. After
# this, we can use the HAVE_PROTOBUF flag to determine if we have
# got both protoc and libprotobuf available.
main['HAVE_PROTOBUF'] = main['PROTOC'] and \
    conf.CheckLibWithHeader('protobuf', 'google/protobuf/message.h',
                            'C++', 'GOOGLE_PROTOBUF_VERIFY_VERSION;')

# If we have the compiler but not the library, print another warning.
if main['PROTOC'] and not main['HAVE_PROTOBUF']:
    print termcap.Yellow + termcap.Bold + \
        'Warning: did not find protocol buffer library and/or headers.\n' + \
    '       Please install libprotobuf-dev for tracing support.' + \
    termcap.Normal

# Check for librt.
have_posix_clock = \
    conf.CheckLibWithHeader(None, 'time.h', 'C',
                            'clock_nanosleep(0,0,NULL,NULL);') or \
    conf.CheckLibWithHeader('rt', 'time.h', 'C',
                            'clock_nanosleep(0,0,NULL,NULL);')

have_posix_timers = \
    conf.CheckLibWithHeader([None, 'rt'], [ 'time.h', 'signal.h' ], 'C',
                            'timer_create(CLOCK_MONOTONIC, NULL, NULL);')

if conf.CheckLib('tcmalloc'):
    main.Append(CCFLAGS=main['TCMALLOC_CCFLAGS'])
elif conf.CheckLib('tcmalloc_minimal'):
    main.Append(CCFLAGS=main['TCMALLOC_CCFLAGS'])
else:
    print termcap.Yellow + termcap.Bold + \
          "You can get a 12% performance improvement by installing tcmalloc "\
          "(libgoogle-perftools-dev package on Ubuntu or RedHat)." + \
          termcap.Normal

if not have_posix_clock:
    print "Can't find library for POSIX clocks."

# Check for <fenv.h> (C99 FP environment control)
have_fenv = conf.CheckHeader('fenv.h', '<>')
if not have_fenv:
    print "Warning: Header file <fenv.h> not found."
    print "         This host has no IEEE FP rounding mode control."

# Check if we should enable KVM-based hardware virtualization. The API
# we rely on exists since version 2.6.36 of the kernel, but somehow
# the KVM_API_VERSION does not reflect the change. We test for one of
# the types as a fall back.
have_kvm = conf.CheckHeader('linux/kvm.h', '<>') and \
    conf.CheckTypeSize('struct kvm_xsave', '#include <linux/kvm.h>') != 0
if not have_kvm:
    print "Info: Compatible header file <linux/kvm.h> not found, " \
        "disabling KVM support."

# Check if the requested target ISA is compatible with the host
def is_isa_kvm_compatible(isa):
    isa_comp_table = {
        "arm" : ( "armv7l" ),
        "x86" : ( "x86_64" ),
        }
    try:
        import platform
        host_isa = platform.machine()
    except:
        print "Warning: Failed to determine host ISA."
        return False

    return host_isa in isa_comp_table.get(isa, [])


# Check if the exclude_host attribute is available. We want this to
# get accurate instruction counts in KVM.
main['HAVE_PERF_ATTR_EXCLUDE_HOST'] = conf.CheckMember(
    'linux/perf_event.h', 'struct perf_event_attr', 'exclude_host')


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

# For Ruby
all_protocols = []
Export('all_protocols')
protocol_dirs = []
Export('protocol_dirs')
slicc_includes = []
Export('slicc_includes')

# Walk the tree and execute all SConsopts scripts that wil add to the
# above variables
if GetOption('verbose'):
    print "Reading SConsopts"
for bdir in [ base_dir ] + extras_dir_list:
    if not isdir(bdir):
        print "Error: directory '%s' does not exist" % bdir
        Exit(1)
    for root, dirs, files in os.walk(bdir):
        if 'SConsopts' in files:
            if GetOption('verbose'):
                print "Reading", joinpath(root, 'SConsopts')
            SConscript(joinpath(root, 'SConsopts'))

all_isa_list.sort()

sticky_vars.AddVariables(
    EnumVariable('TARGET_ISA', 'Target ISA', 'alpha', all_isa_list),
    ListVariable('CPU_MODELS', 'CPU models',
                 sorted(n for n,m in CpuModel.dict.iteritems() if m.default),
                 sorted(CpuModel.list)),
    BoolVariable('EFENCE', 'Link with Electric Fence malloc debugger',
                 False),
    BoolVariable('SS_COMPATIBLE_FP',
                 'Make floating-point results compatible with SimpleScalar',
                 False),
    BoolVariable('USE_SSE2',
                 'Compile for SSE2 (-msse2) to get IEEE FP on x86 hosts',
                 False),
    BoolVariable('USE_POSIX_CLOCK', 'Use POSIX Clocks', have_posix_clock),
    BoolVariable('USE_FENV', 'Use <fenv.h> IEEE mode control', have_fenv),
    BoolVariable('CP_ANNOTATE', 'Enable critical path annotation capability', False),
    BoolVariable('USE_KVM', 'Enable hardware virtualized (KVM) CPU models', have_kvm),
    EnumVariable('PROTOCOL', 'Coherence protocol for Ruby', 'None',
                  all_protocols),
    )

# These variables get exported to #defines in config/*.hh (see src/SConscript).
export_vars += ['USE_FENV', 'SS_COMPATIBLE_FP', 'TARGET_ISA', 'CP_ANNOTATE',
                'USE_POSIX_CLOCK', 'PROTOCOL', 'HAVE_PROTOBUF',
                'HAVE_PERF_ATTR_EXCLUDE_HOST']

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

# Combine the two functions into a scons Action object.
config_action = MakeAction(build_config_file, Transform("CONFIG H", 2))

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

# libfdt build is shared across all configs in the build root.
main.SConscript('ext/libfdt/SConscript',
                variant_dir = joinpath(build_root, 'libfdt'))

# fputils build is shared across all configs in the build root.
main.SConscript('ext/fputils/SConscript',
                variant_dir = joinpath(build_root, 'fputils'))

# DRAMSim2 build is shared across all configs in the build root.
main.SConscript('ext/dramsim2/SConscript',
                variant_dir = joinpath(build_root, 'dramsim2'))

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
    if not GetOption('silent'):
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
        if not GetOption('silent'):
            print "Using saved variables file %s" % current_vars_file
    else:
        # Build dir-specific variables file doesn't exist.

        # Make sure the directory is there so we can create it later
        opt_dir = dirname(current_vars_file)
        if not isdir(opt_dir):
            mkdir(opt_dir)

        # Get default build variables from source tree.  Variables are
        # normally determined by name of $VARIANT_DIR, but can be
        # overridden by '--default=' arg on command line.
        default = GetOption('default')
        opts_dir = joinpath(main.root.abspath, 'build_opts')
        if default:
            default_vars_files = [joinpath(build_root, 'variables', default),
                                  joinpath(opts_dir, default)]
        else:
            default_vars_files = [joinpath(opts_dir, variant_dir)]
        existing_files = filter(isfile, default_vars_files)
        if existing_files:
            default_vars_file = existing_files[0]
            sticky_vars.files.append(default_vars_file)
            print "Variables file %s not found,\n  using defaults in %s" \
                  % (current_vars_file, default_vars_file)
        else:
            print "Error: cannot find variables file %s or " \
                  "default file(s) %s" \
                  % (current_vars_file, ' or '.join(default_vars_files))
            Exit(1)

    # Apply current variable settings to env
    sticky_vars.Update(env)

    help_texts["local_vars"] += \
        "Build variables for %s:\n" % variant_dir \
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

    if env['USE_KVM']:
        if not have_kvm:
            print "Warning: Can not enable KVM, host seems to lack KVM support"
            env['USE_KVM'] = False
        elif not have_posix_timers:
            print "Warning: Can not enable KVM, host seems to lack support " \
                "for POSIX timers"
            env['USE_KVM'] = False
        elif not is_isa_kvm_compatible(env['TARGET_ISA']):
            print "Info: KVM support disabled due to unsupported host and " \
                "target ISA combination"
            env['USE_KVM'] = False

    # Warn about missing optional functionality
    if env['USE_KVM']:
        if not main['HAVE_PERF_ATTR_EXCLUDE_HOST']:
            print "Warning: perf_event headers lack support for the " \
                "exclude_host attribute. KVM instruction counts will " \
                "be inaccurate."

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

# base help text
Help('''
Usage: scons [scons options] [build variables] [target(s)]

Extra scons options:
%(options)s

Global build variables:
%(global_vars)s

%(local_vars)s
''' % help_texts)
