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

from __future__ import print_function
import re
import sys

import gem5_scons.util

mercurial_style_message = """
You're missing the gem5 style hook, which automatically checks your code
against the gem5 style rules on hg commit and qrefresh commands.
This script will now install the hook in your .hg/hgrc file.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_upgrade_message = """
Your Mercurial style hooks are not up-to-date. This script will now
try to automatically update them. A backup of your hgrc will be saved
in .hg/hgrc.old.
Press enter to continue, or ctrl-c to abort: """

mercurial_style_hook_template = """
# The following lines were automatically added by gem5/SConstruct
# to provide the gem5 style-checking hooks
[extensions]
hgstyle = %s/util/hgstyle.py

[hooks]
pretxncommit.style = python:hgstyle.check_style
pre-qrefresh.style = python:hgstyle.check_style
# End of SConstruct additions

"""

mercurial_lib_not_found = """
Mercurial libraries cannot be found, ignoring style hook.  If
you are a gem5 developer, please fix this and run the style
hook. It is important.
"""

def install_style_hooks(env):
    hgdir = env.Dir('#.hg')

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
        print(mercurial_lib_not_found)

    if "python:style.check_style" in style_hooks:
        # Try to upgrade the style hooks
        print(mercurial_style_upgrade_message)
        # continue unless user does ctrl-c/ctrl-d etc.
        try:
            raw_input()
        except:
            print("Input exception, exiting scons.\n")
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
                    print("Warning: %s.style is using a non-default " \
                        "checker: %s" % (hook, check))
                if hook not in ("pretxncommit", "pre-qrefresh"):
                    print("Warning: Updating unknown style hook: %s" % hook)

                l = "%s.style = python:hgstyle.check_style\n" % hook
            elif m_ext and m_ext.group(1) == style_extension:
                l = "hgstyle = %s/util/hgstyle.py\n" % env.root.abspath

            new.write(l)
    elif not style_hook:
        print(mercurial_style_message, end=' ')
        # continue unless user does ctrl-c/ctrl-d etc.
        try:
            raw_input()
        except:
            print("Input exception, exiting scons.\n")
            sys.exit(1)
        hgrc_path = '%s/.hg/hgrc' % env.root.abspath
        print("Adding style hook to", hgrc_path, "\n")
        try:
            with open(hgrc_path, 'a') as f:
                f.write(mercurial_style_hook_template % env.root.abspath)
        except:
            print("Error updating", hgrc_path)
            sys.exit(1)

def generate(env):
    if exists(env) and not gem5_scons.util.ignore_style():
        install_style_hooks(env)

def exists(env):
    return env.Dir('#.hg').exists()
