# -*- mode:python -*-

# Copyright (c) 2013, 2015-2017 ARM Limited
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

# pybind11 requires python 2.7
try:
    EnsurePythonVersion(2, 7)
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
import itertools
import os
import re
import shutil
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
AddLocalOption('--with-cxx-config', dest='with_cxx_config',
               action='store_true',
               help="Build with support for C++-based configuration")
AddLocalOption('--default', dest='default', type='string', action='store',
               help='Override which build_opts file to use for defaults')
AddLocalOption('--ignore-style', dest='ignore_style', action='store_true',
               help='Disable style checking hooks')
AddLocalOption('--no-lto', dest='no_lto', action='store_true',
               help='Disable Link-Time Optimization for fast')
AddLocalOption('--force-lto', dest='force_lto', action='store_true',
               help='Use Link-Time Optimization instead of partial linking' +
                    ' when the compiler doesn\'t support using them together.')
AddLocalOption('--update-ref', dest='update_ref', action='store_true',
               help='Update test reference outputs')
AddLocalOption('--verbose', dest='verbose', action='store_true',
               help='Print full tool command lines')
AddLocalOption('--without-python', dest='without_python',
               action='store_true',
               help='Build without Python configuration support')
AddLocalOption('--without-tcmalloc', dest='without_tcmalloc',
               action='store_true',
               help='Disable linking against tcmalloc')
AddLocalOption('--with-ubsan', dest='with_ubsan', action='store_true',
               help='Build with Undefined Behavior Sanitizer if available')
AddLocalOption('--with-asan', dest='with_asan', action='store_true',
               help='Build with Address Sanitizer if available')

if GetOption('no_lto') and GetOption('force_lto'):
    print '--no-lto and --force-lto are mutually exclusive'
    Exit(1)

termcap = get_termcap(GetOption('use_colors'))

########################################################################
#
# Set up the main build environment.
#
########################################################################

# export TERM so that clang reports errors in color
use_vars = set([ 'AS', 'AR', 'CC', 'CXX', 'HOME', 'LD_LIBRARY_PATH',
                 'LIBRARY_PATH', 'PATH', 'PKG_CONFIG_PATH', 'PROTOC',
                 'PYTHONPATH', 'RANLIB', 'TERM' ])

use_prefixes = [
    "ASAN_",           # address sanitizer symbolizer path and settings
    "CCACHE_",         # ccache (caching compiler wrapper) configuration
    "CCC_",            # clang static analyzer configuration
    "DISTCC_",         # distcc (distributed compiler wrapper) configuration
    "INCLUDE_SERVER_", # distcc pump server settings
    "M5",              # M5 configuration (e.g., path to kernels)
    ]

use_env = {}
for key,val in sorted(os.environ.iteritems()):
    if key in use_vars or \
            any([key.startswith(prefix) for prefix in use_prefixes]):
        use_env[key] = val

# Tell scons to avoid implicit command dependencies to avoid issues
# with the param wrappes being compiled twice (see
# http://scons.tigris.org/issues/show_bug.cgi?id=2811)
main = Environment(ENV=use_env, IMPLICIT_COMMAND_DEPENDENCIES=0)
main.Decider('MD5-timestamp')
main.root = Dir(".")         # The current directory (where this file lives).
main.srcdir = Dir("src")     # The source directory

main_dict_keys = main.Dictionary().keys()

# Check that we have a C/C++ compiler
if not ('CC' in main_dict_keys and 'CXX' in main_dict_keys):
    print "No C++ compiler installed (package g++ on Ubuntu and RedHat)"
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


style_message = """
You're missing the gem5 style hook, which automatically checks your code
against the gem5 style rules on %s.
This script will now install the hook in your %s.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_message = """
You're missing the gem5 style hook, which automatically checks your code
against the gem5 style rules on hg commit and qrefresh commands.
This script will now install the hook in your .hg/hgrc file.
Press enter to continue, or ctrl-c to abort: """

git_style_message = """
You're missing the gem5 style or commit message hook. These hooks help
to ensure that your code follows gem5's style rules on git commit.
This script will now install the hook in your .git/hooks/ directory.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_upgrade_message = """
Your Mercurial style hooks are not up-to-date. This script will now
try to automatically update them. A backup of your hgrc will be saved
in .hg/hgrc.old.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_hook = """
# The following lines were automatically added by gem5/SConstruct
# to provide the gem5 style-checking hooks
[extensions]
hgstyle = %s/util/hgstyle.py

[hooks]
pretxncommit.style = python:hgstyle.check_style
pre-qrefresh.style = python:hgstyle.check_style
# End of SConstruct additions

""" % (main.root.abspath)

mercurial_lib_not_found = """
Mercurial libraries cannot be found, ignoring style hook.  If
you are a gem5 developer, please fix this and run the style
hook. It is important.
"""

# Check for style hook and prompt for installation if it's not there.
# Skip this if --ignore-style was specified, there's no interactive
# terminal to prompt, or no recognized revision control system can be
# found.
ignore_style = GetOption('ignore_style') or not sys.stdin.isatty()

# Try wire up Mercurial to the style hooks
if not ignore_style and hgdir.exists():
    style_hook = True
    style_hooks = tuple()
    hgrc = hgdir.File('hgrc')
    hgrc_old = hgdir.File('hgrc.old')
    try:
        from mercurial import ui
        ui = ui.ui()
        ui.readconfig(hgrc.abspath)
        style_hooks = (ui.config('hooks', 'pretxncommit.style', None),
                       ui.config('hooks', 'pre-qrefresh.style', None))
        style_hook = all(style_hooks)
        style_extension = ui.config('extensions', 'style', None)
    except ImportError:
        print mercurial_lib_not_found

    if "python:style.check_style" in style_hooks:
        # Try to upgrade the style hooks
        print mercurial_style_upgrade_message
        # continue unless user does ctrl-c/ctrl-d etc.
        try:
            raw_input()
        except:
            print "Input exception, exiting scons.\n"
            sys.exit(1)
        shutil.copyfile(hgrc.abspath, hgrc_old.abspath)
        re_style_hook = re.compile(r"^([^=#]+)\.style\s*=\s*([^#\s]+).*")
        re_style_extension = re.compile("style\s*=\s*([^#\s]+).*")
        old, new = open(hgrc_old.abspath, 'r'), open(hgrc.abspath, 'w')
        for l in old:
            m_hook = re_style_hook.match(l)
            m_ext = re_style_extension.match(l)
            if m_hook:
                hook, check = m_hook.groups()
                if check != "python:style.check_style":
                    print "Warning: %s.style is using a non-default " \
                        "checker: %s" % (hook, check)
                if hook not in ("pretxncommit", "pre-qrefresh"):
                    print "Warning: Updating unknown style hook: %s" % hook

                l = "%s.style = python:hgstyle.check_style\n" % hook
            elif m_ext and m_ext.group(1) == style_extension:
                l = "hgstyle = %s/util/hgstyle.py\n" % main.root.abspath

            new.write(l)
    elif not style_hook:
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
            with open(hgrc_path, 'a') as f:
                f.write(mercurial_style_hook)
        except:
            print "Error updating", hgrc_path
            sys.exit(1)

def install_git_style_hooks():
    try:
        gitdir = Dir(readCommand(
            ["git", "rev-parse", "--git-dir"]).strip("\n"))
    except Exception, e:
        print "Warning: Failed to find git repo directory: %s" % e
        return

    git_hooks = gitdir.Dir("hooks")
    def hook_exists(hook_name):
        hook = git_hooks.File(hook_name)
        return hook.exists()

    def hook_install(hook_name, script):
        hook = git_hooks.File(hook_name)
        if hook.exists():
            print "Warning: Can't install %s, hook already exists." % hook_name
            return

        if hook.islink():
            print "Warning: Removing broken symlink for hook %s." % hook_name
            os.unlink(hook.get_abspath())

        if not git_hooks.exists():
            mkdir(git_hooks.get_abspath())
            git_hooks.clear()

        abs_symlink_hooks = git_hooks.islink() and \
            os.path.isabs(os.readlink(git_hooks.get_abspath()))

        # Use a relative symlink if the hooks live in the source directory,
        # and the hooks directory is not a symlink to an absolute path.
        if hook.is_under(main.root) and not abs_symlink_hooks:
            script_path = os.path.relpath(
                os.path.realpath(script.get_abspath()),
                os.path.realpath(hook.Dir(".").get_abspath()))
        else:
            script_path = script.get_abspath()

        try:
            os.symlink(script_path, hook.get_abspath())
        except:
            print "Error updating git %s hook" % hook_name
            raise

    if hook_exists("pre-commit") and hook_exists("commit-msg"):
        return

    print git_style_message,
    try:
        raw_input()
    except:
        print "Input exception, exiting scons.\n"
        sys.exit(1)

    git_style_script = File("util/git-pre-commit.py")
    git_msg_script = File("ext/git-commit-msg")

    hook_install("pre-commit", git_style_script)
    hook_install("commit-msg", git_msg_script)

# Try to wire up git to the style hooks
if not ignore_style and main.root.Entry(".git").exists():
    install_git_style_hooks()

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

# Add shared top-level headers
main.Prepend(CPPPATH=Dir('include'))

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
    main['ARCOMSTR']        = Transform("AR", 0)
    main['LINKCOMSTR']      = Transform("LINK", 0)
    main['SHLINKCOMSTR']    = Transform("SHLINK", 0)
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
    # Enable -Wall and -Wextra and then disable the few warnings that
    # we consistently violate
    main.Append(CCFLAGS=['-Wall', '-Wundef', '-Wextra',
                         '-Wno-sign-compare', '-Wno-unused-parameter'])
    # We always compile using C++11
    main.Append(CXXFLAGS=['-std=c++11'])
    if sys.platform.startswith('freebsd'):
        main.Append(CCFLAGS=['-I/usr/local/include'])
        main.Append(CXXFLAGS=['-I/usr/local/include'])

    main['FILTER_PSHLINKFLAGS'] = lambda x: str(x).replace(' -shared', '')
    main['PSHLINKFLAGS'] = main.subst('${FILTER_PSHLINKFLAGS(SHLINKFLAGS)}')
    main['PLINKFLAGS'] = main.subst('${LINKFLAGS}')
    shared_partial_flags = ['-r', '-nostdlib']
    main.Append(PSHLINKFLAGS=shared_partial_flags)
    main.Append(PLINKFLAGS=shared_partial_flags)
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
    # Check for a supported version of gcc. >= 4.8 is chosen for its
    # level of c++11 support. See
    # http://gcc.gnu.org/projects/cxx0x.html for details.
    gcc_version = readCommand([main['CXX'], '-dumpversion'], exception=False)
    if compareVersions(gcc_version, "4.8") < 0:
        print 'Error: gcc version 4.8 or newer required.'
        print '       Installed version:', gcc_version
        Exit(1)

    main['GCC_VERSION'] = gcc_version

    if compareVersions(gcc_version, '4.9') >= 0:
        # Incremental linking with LTO is currently broken in gcc versions
        # 4.9 and above. A version where everything works completely hasn't
        # yet been identified.
        #
        # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67548
        main['BROKEN_INCREMENTAL_LTO'] = True
    if compareVersions(gcc_version, '6.0') >= 0:
        # gcc versions 6.0 and greater accept an -flinker-output flag which
        # selects what type of output the linker should generate. This is
        # necessary for incremental lto to work, but is also broken in
        # current versions of gcc. It may not be necessary in future
        # versions. We add it here since it might be, and as a reminder that
        # it exists. It's excluded if lto is being forced.
        #
        # https://gcc.gnu.org/gcc-6/changes.html
        # https://gcc.gnu.org/ml/gcc-patches/2015-11/msg03161.html
        # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69866
        if not GetOption('force_lto'):
            main.Append(PSHLINKFLAGS='-flinker-output=rel')
            main.Append(PLINKFLAGS='-flinker-output=rel')

    # gcc from version 4.8 and above generates "rep; ret" instructions
    # to avoid performance penalties on certain AMD chips. Older
    # assemblers detect this as an error, "Error: expecting string
    # instruction after `rep'"
    as_version_raw = readCommand([main['AS'], '-v', '/dev/null',
                                  '-o', '/dev/null'],
                                 exception=False).split()

    # version strings may contain extra distro-specific
    # qualifiers, so play it safe and keep only what comes before
    # the first hyphen
    as_version = as_version_raw[-1].split('-')[0] if as_version_raw else None

    if not as_version or compareVersions(as_version, "2.23") < 0:
        print termcap.Yellow + termcap.Bold + \
            'Warning: This combination of gcc and binutils have' + \
            ' known incompatibilities.\n' + \
            '         If you encounter build problems, please update ' + \
            'binutils to 2.23.' + \
            termcap.Normal

    # Make sure we warn if the user has requested to compile with the
    # Undefined Benahvior Sanitizer and this version of gcc does not
    # support it.
    if GetOption('with_ubsan') and \
            compareVersions(gcc_version, '4.9') < 0:
        print termcap.Yellow + termcap.Bold + \
            'Warning: UBSan is only supported using gcc 4.9 and later.' + \
            termcap.Normal

    disable_lto = GetOption('no_lto')
    if not disable_lto and main.get('BROKEN_INCREMENTAL_LTO', False) and \
            not GetOption('force_lto'):
        print termcap.Yellow + termcap.Bold + \
            'Warning: Your compiler doesn\'t support incremental linking' + \
            ' and lto at the same time, so lto is being disabled. To force' + \
            ' lto on anyway, use the --force-lto option. That will disable' + \
            ' partial linking.' + \
            termcap.Normal
        disable_lto = True

    # Add the appropriate Link-Time Optimization (LTO) flags
    # unless LTO is explicitly turned off. Note that these flags
    # are only used by the fast target.
    if not disable_lto:
        # Pass the LTO flag when compiling to produce GIMPLE
        # output, we merely create the flags here and only append
        # them later
        main['LTO_CCFLAGS'] = ['-flto=%d' % GetOption('num_jobs')]

        # Use the same amount of jobs for LTO as we are running
        # scons with
        main['LTO_LDFLAGS'] = ['-flto=%d' % GetOption('num_jobs')]

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin-malloc', '-fno-builtin-calloc',
                                  '-fno-builtin-realloc', '-fno-builtin-free'])

    # add option to check for undeclared overrides
    if compareVersions(gcc_version, "5.0") > 0:
        main.Append(CCFLAGS=['-Wno-error=suggest-override'])

elif main['CLANG']:
    # Check for a supported version of clang, >= 3.1 is needed to
    # support similar features as gcc 4.8. See
    # http://clang.llvm.org/cxx_status.html for details
    clang_version_re = re.compile(".* version (\d+\.\d+)")
    clang_version_match = clang_version_re.search(CXX_version)
    if (clang_version_match):
        clang_version = clang_version_match.groups()[0]
        if compareVersions(clang_version, "3.1") < 0:
            print 'Error: clang version 3.1 or newer required.'
            print '       Installed version:', clang_version
            Exit(1)
    else:
        print 'Error: Unable to determine clang version.'
        Exit(1)

    # clang has a few additional warnings that we disable, extraneous
    # parantheses are allowed due to Ruby's printing of the AST,
    # finally self assignments are allowed as the generated CPU code
    # is relying on this
    main.Append(CCFLAGS=['-Wno-parentheses',
                         '-Wno-self-assign',
                         # Some versions of libstdc++ (4.8?) seem to
                         # use struct hash and class hash
                         # interchangeably.
                         '-Wno-mismatched-tags',
                         ])

    main.Append(TCMALLOC_CCFLAGS=['-fno-builtin'])

    # On Mac OS X/Darwin we need to also use libc++ (part of XCode) as
    # opposed to libstdc++, as the later is dated.
    if sys.platform == "darwin":
        main.Append(CXXFLAGS=['-stdlib=libc++'])
        main.Append(LIBS=['c++'])

    # On FreeBSD we need libthr.
    if sys.platform.startswith('freebsd'):
        main.Append(LIBS=['thr'])

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


# Check for 'timeout' from GNU coreutils. If present, regressions will
# be run with a time limit. We require version 8.13 since we rely on
# support for the '--foreground' option.
if sys.platform.startswith('freebsd'):
    timeout_lines = readCommand(['gtimeout', '--version'],
                                exception='').splitlines()
else:
    timeout_lines = readCommand(['timeout', '--version'],
                                exception='').splitlines()
# Get the first line and tokenize it
timeout_version = timeout_lines[0].split() if timeout_lines else []
main['TIMEOUT'] =  timeout_version and \
    compareVersions(timeout_version[-1], '8.13') >= 0

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
        'CheckMember' : CheckMember,
        })

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

main['USE_PYTHON'] = not GetOption('without_python')
if main['USE_PYTHON']:
    # Find Python include and library directories for embedding the
    # interpreter. We rely on python-config to resolve the appropriate
    # includes and linker flags. ParseConfig does not seem to understand
    # the more exotic linker flags such as -Xlinker and -export-dynamic so
    # we add them explicitly below. If you want to link in an alternate
    # version of python, see above for instructions on how to invoke
    # scons with the appropriate PATH set.
    #
    # First we check if python2-config exists, else we use python-config
    python_config = readCommand(['which', 'python2-config'],
                                exception='').strip()
    if not os.path.exists(python_config):
        python_config = readCommand(['which', 'python-config'],
                                    exception='').strip()
    py_includes = readCommand([python_config, '--includes'],
                              exception='').split()
    # Strip the -I from the include folders before adding them to the
    # CPPPATH
    main.Append(CPPPATH=map(lambda inc: inc[2:], py_includes))

    # Read the linker flags and split them into libraries and other link
    # flags. The libraries are added later through the call the CheckLib.
    py_ld_flags = readCommand([python_config, '--ldflags'],
        exception='').split()
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

if not GetOption('without_tcmalloc'):
    if conf.CheckLib('tcmalloc'):
        main.Append(CCFLAGS=main['TCMALLOC_CCFLAGS'])
    elif conf.CheckLib('tcmalloc_minimal'):
        main.Append(CCFLAGS=main['TCMALLOC_CCFLAGS'])
    else:
        print termcap.Yellow + termcap.Bold + \
              "You can get a 12% performance improvement by "\
              "installing tcmalloc (libgoogle-perftools-dev package "\
              "on Ubuntu or RedHat)." + termcap.Normal


# Detect back trace implementations. The last implementation in the
# list will be used by default.
backtrace_impls = [ "none" ]

if conf.CheckLibWithHeader(None, 'execinfo.h', 'C',
                           'backtrace_symbols_fd((void*)0, 0, 0);'):
    backtrace_impls.append("glibc")
elif conf.CheckLibWithHeader('execinfo', 'execinfo.h', 'C',
                           'backtrace_symbols_fd((void*)0, 0, 0);'):
    # NetBSD and FreeBSD need libexecinfo.
    backtrace_impls.append("glibc")
    main.Append(LIBS=['execinfo'])

if backtrace_impls[-1] == "none":
    default_backtrace_impl = "none"
    print termcap.Yellow + termcap.Bold + \
        "No suitable back trace implementation found." + \
        termcap.Normal

if not have_posix_clock:
    print "Can't find library for POSIX clocks."

# Check for <fenv.h> (C99 FP environment control)
have_fenv = conf.CheckHeader('fenv.h', '<>')
if not have_fenv:
    print "Warning: Header file <fenv.h> not found."
    print "         This host has no IEEE FP rounding mode control."

# Check for <png.h> (libpng library needed if wanting to dump
# frame buffer image in png format)
have_png = conf.CheckHeader('png.h', '<>')
if not have_png:
    print "Warning: Header file <png.h> not found."
    print "         This host has no libpng library."
    print "         Disabling support for PNG framebuffers."

# Check if we should enable KVM-based hardware virtualization. The API
# we rely on exists since version 2.6.36 of the kernel, but somehow
# the KVM_API_VERSION does not reflect the change. We test for one of
# the types as a fall back.
have_kvm = conf.CheckHeader('linux/kvm.h', '<>')
if not have_kvm:
    print "Info: Compatible header file <linux/kvm.h> not found, " \
        "disabling KVM support."

# Check if the TUN/TAP driver is available.
have_tuntap = conf.CheckHeader('linux/if_tun.h', '<>')
if not have_tuntap:
    print "Info: Compatible header file <linux/if_tun.h> not found."

# x86 needs support for xsave. We test for the structure here since we
# won't be able to run new tests by the time we know which ISA we're
# targeting.
have_kvm_xsave = conf.CheckTypeSize('struct kvm_xsave',
                                    '#include <linux/kvm.h>') != 0

# Check if the requested target ISA is compatible with the host
def is_isa_kvm_compatible(isa):
    try:
        import platform
        host_isa = platform.machine()
    except:
        print "Warning: Failed to determine host ISA."
        return False

    if not have_posix_timers:
        print "Warning: Can not enable KVM, host seems to lack support " \
            "for POSIX timers"
        return False

    if isa == "arm":
        return host_isa in ( "armv7l", "aarch64" )
    elif isa == "x86":
        if host_isa != "x86_64":
            return False

        if not have_kvm_xsave:
            print "KVM on x86 requires xsave support in kernel headers."
            return False

        return True
    else:
        return False


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
all_gpu_isa_list = [ ]
Export('all_isa_list')
Export('all_gpu_isa_list')

class CpuModel(object):
    '''The CpuModel class encapsulates everything the ISA parser needs to
    know about a particular CPU model.'''

    # Dict of available CPU model objects.  Accessible as CpuModel.dict.
    dict = {}

    # Constructor.  Automatically adds models to CpuModel.dict.
    def __init__(self, name, default=False):
        self.name = name           # name of model

        # This cpu is enabled by default
        self.default = default

        # Add self to dict
        if name in CpuModel.dict:
            raise AttributeError, "CpuModel '%s' already registered" % name
        CpuModel.dict[name] = self

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
all_gpu_isa_list.sort()

sticky_vars.AddVariables(
    EnumVariable('TARGET_ISA', 'Target ISA', 'alpha', all_isa_list),
    EnumVariable('TARGET_GPU_ISA', 'Target GPU ISA', 'hsail', all_gpu_isa_list),
    ListVariable('CPU_MODELS', 'CPU models',
                 sorted(n for n,m in CpuModel.dict.iteritems() if m.default),
                 sorted(CpuModel.dict.keys())),
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
    BoolVariable('USE_PNG',  'Enable support for PNG images', have_png),
    BoolVariable('CP_ANNOTATE', 'Enable critical path annotation capability',
                 False),
    BoolVariable('USE_KVM', 'Enable hardware virtualized (KVM) CPU models',
                 have_kvm),
    BoolVariable('USE_TUNTAP',
                 'Enable using a tap device to bridge to the host network',
                 have_tuntap),
    BoolVariable('BUILD_GPU', 'Build the compute-GPU model', False),
    EnumVariable('PROTOCOL', 'Coherence protocol for Ruby', 'None',
                  all_protocols),
    EnumVariable('BACKTRACE_IMPL', 'Post-mortem dump implementation',
                 backtrace_impls[-1], backtrace_impls)
    )

# These variables get exported to #defines in config/*.hh (see src/SConscript).
export_vars += ['USE_FENV', 'SS_COMPATIBLE_FP', 'TARGET_ISA', 'TARGET_GPU_ISA',
                'CP_ANNOTATE', 'USE_POSIX_CLOCK', 'USE_KVM', 'USE_TUNTAP',
                'PROTOCOL', 'HAVE_PROTOBUF', 'HAVE_PERF_ATTR_EXCLUDE_HOST',
                'USE_PNG']

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

###################################################
#
# Builders for static and shared partially linked object files.
#
###################################################

partial_static_builder = Builder(action=SCons.Defaults.LinkAction,
                                 src_suffix='$OBJSUFFIX',
                                 src_builder=['StaticObject', 'Object'],
                                 LINKFLAGS='$PLINKFLAGS',
                                 LIBS='')

def partial_shared_emitter(target, source, env):
    for tgt in target:
        tgt.attributes.shared = 1
    return (target, source)
partial_shared_builder = Builder(action=SCons.Defaults.ShLinkAction,
                                 emitter=partial_shared_emitter,
                                 src_suffix='$SHOBJSUFFIX',
                                 src_builder='SharedObject',
                                 SHLINKFLAGS='$PSHLINKFLAGS',
                                 LIBS='')

main.Append(BUILDERS = { 'PartialShared' : partial_shared_builder,
                         'PartialStatic' : partial_static_builder })

# builds in ext are shared across all configs in the build root.
ext_dir = abspath(joinpath(str(main.root), 'ext'))
ext_build_dirs = []
for root, dirs, files in os.walk(ext_dir):
    if 'SConscript' in files:
        build_dir = os.path.relpath(root, ext_dir)
        ext_build_dirs.append(build_dir)
        main.SConscript(joinpath(root, 'SConscript'),
                        variant_dir=joinpath(build_root, build_dir))

main.Prepend(CPPPATH=Dir('ext/pybind11/include/'))

###################################################
#
# This builder and wrapper method are used to set up a directory with
# switching headers. Those are headers which are in a generic location and
# that include more specific headers from a directory chosen at build time
# based on the current build settings.
#
###################################################

def build_switching_header(target, source, env):
    path = str(target[0])
    subdir = str(source[0])
    dp, fp = os.path.split(path)
    dp = os.path.relpath(os.path.realpath(dp),
                         os.path.realpath(env['BUILDDIR']))
    with open(path, 'w') as hdr:
        print >>hdr, '#include "%s/%s/%s"' % (dp, subdir, fp)

switching_header_action = MakeAction(build_switching_header,
                                     Transform('GENERATE'))

switching_header_builder = Builder(action=switching_header_action,
                                   source_factory=Value,
                                   single_source=True)

main.Append(BUILDERS = { 'SwitchingHeader': switching_header_builder })

def switching_headers(self, headers, source):
    for header in headers:
        self.SwitchingHeader(header, source)

main.AddMethod(switching_headers, 'SwitchingHeaders')

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
    elif variant_dir in ext_build_dirs:
        # Things in ext are built without a variant directory.
        continue
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

    if not have_png and env['USE_PNG']:
        print "Warning: <png.h> not available; " \
              "forcing USE_PNG to False in", variant_dir + "."
        env['USE_PNG'] = False

    if env['USE_PNG']:
        env.Append(LIBS=['png'])

    if env['EFENCE']:
        env.Append(LIBS=['efence'])

    if env['USE_KVM']:
        if not have_kvm:
            print "Warning: Can not enable KVM, host seems to lack KVM support"
            env['USE_KVM'] = False
        elif not is_isa_kvm_compatible(env['TARGET_ISA']):
            print "Info: KVM support disabled due to unsupported host and " \
                "target ISA combination"
            env['USE_KVM'] = False

    if env['USE_TUNTAP']:
        if not have_tuntap:
            print "Warning: Can't connect EtherTap with a tap device."
            env['USE_TUNTAP'] = False

    if env['BUILD_GPU']:
        env.Append(CPPDEFINES=['BUILD_GPU'])

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
    SConscript('src/SConscript', variant_dir = variant_path, exports = 'env')

# base help text
Help('''
Usage: scons [scons options] [build variables] [target(s)]

Extra scons options:
%(options)s

Global build variables:
%(global_vars)s

%(local_vars)s
''' % help_texts)
