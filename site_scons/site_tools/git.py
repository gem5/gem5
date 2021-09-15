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

import os
import sys

import gem5_scons.util
import SCons.Script

git_style_message = """
You're missing the gem5 style or commit message hook. These hooks help
to ensure that your code follows gem5's style rules on git commit.
This script will now install the hook in your .git/hooks/ directory.
Press enter to continue, or ctrl-c to abort: """

def install_style_hooks(env):
    try:
        gitdir = env.Dir(gem5_scons.util.readCommand(
            ["git", "rev-parse", "--git-dir"]).strip("\n"))
    except Exception as e:
        print("Warning: Failed to find git repo directory: %s" % e)
        return

    git_hooks = gitdir.Dir("hooks")
    def hook_exists(hook_name):
        hook = git_hooks.File(hook_name)
        return hook.exists()

    def hook_install(hook_name, script):
        hook = git_hooks.File(hook_name)
        if hook.exists():
            print("Warning: Can't install %s, hook already exists." %
                    hook_name)
            return

        if hook.islink():
            print("Warning: Removing broken symlink for hook %s." % hook_name)
            os.unlink(hook.get_abspath())

        if not git_hooks.exists():
            os.mkdir(git_hooks.get_abspath())
            git_hooks.clear()

        abs_symlink_hooks = git_hooks.islink() and \
            os.path.isabs(os.readlink(git_hooks.get_abspath()))

        # Use a relative symlink if the hooks live in the source directory,
        # and the hooks directory is not a symlink to an absolute path.
        if hook.is_under(env.Dir("#")) and not abs_symlink_hooks:
            script_path = os.path.relpath(
                os.path.realpath(script.get_abspath()),
                os.path.realpath(hook.Dir(".").get_abspath()))
        else:
            script_path = script.get_abspath()

        try:
            os.symlink(script_path, hook.get_abspath())
        except:
            print("Error updating git %s hook" % hook_name)
            raise

    if hook_exists("pre-commit") and hook_exists("commit-msg"):
        return

    print(git_style_message, end=' ')
    if SCons.Script.GetOption('install-hooks'):
        print("Installing revision control hooks automatically.")
    else:
        try:
            input()
        except:
            print("Input exception, exiting scons.\n")
            sys.exit(1)

    git_style_script = env.Dir("#util").File("git-pre-commit.py")
    git_msg_script = env.Dir("#ext").File("git-commit-msg")

    hook_install("pre-commit", git_style_script)
    hook_install("commit-msg", git_msg_script)

def generate(env):
    if exists(env) and not gem5_scons.util.ignore_style():
        install_style_hooks(env)

def exists(env):
    return env.Entry('#.git').exists()
