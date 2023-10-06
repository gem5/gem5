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
import contextlib
import os

import SCons.Script
import SCons.Util


def CheckCxxFlag(context, flag, autoadd=True):
    context.Message(f"Checking for compiler {flag} support... ")
    last_cxxflags = context.env["CXXFLAGS"]
    context.env.Append(CXXFLAGS=[flag])
    pre_werror = context.env["CXXFLAGS"]
    context.env.Append(CXXFLAGS=["-Werror"])
    ret = context.TryCompile("// CheckCxxFlag DO NOTHING", ".cc")
    context.env["CXXFLAGS"] = pre_werror
    if not (ret and autoadd):
        context.env["CXXFLAGS"] = last_cxxflags
    context.Result(ret)
    return ret


def CheckLinkFlag(context, flag, autoadd=True, set_for_shared=True):
    context.Message(f"Checking for linker {flag} support... ")
    last_linkflags = context.env["LINKFLAGS"]
    context.env.Append(LINKFLAGS=[flag])
    pre_werror = context.env["LINKFLAGS"]
    context.env.Append(LINKFLAGS=["-Werror"])
    ret = context.TryLink("int main(int, char *[]) { return 0; }", ".cc")
    context.env["LINKFLAGS"] = pre_werror
    if not (ret and autoadd):
        context.env["LINKFLAGS"] = last_linkflags
    if ret and set_for_shared:
        assert autoadd
        context.env.Append(SHLINKFLAGS=[flag])
    context.Result(ret)
    return ret


# Add a custom Check function to test for structure members.
def CheckMember(context, include, decl, member, include_quotes="<>"):
    context.Message(f"Checking for member {member} in {decl}...")
    text = """
#include {header}
int main(){{
  {decl} test;
  (void)test.{member};
  return 0;
}};
""".format(
        header=include_quotes[0] + include + include_quotes[1],
        decl=decl,
        member=member,
    )

    ret = context.TryCompile(text, extension=".cc")
    context.Result(ret)
    return ret


def CheckPythonLib(context):
    context.Message("Checking Python version... ")
    ret = context.TryRun(
        r"""
#include <pybind11/embed.h>

int
main(int argc, char **argv) {
    pybind11::scoped_interpreter guard{};
    pybind11::exec(
        "import sys\n"
        "vi = sys.version_info\n"
        "sys.stdout.write('%i.%i.%i' % (vi.major, vi.minor, vi.micro));\n");
    return 0;
}
    """,
        extension=".cc",
    )
    context.Result(ret[1] if ret[0] == 1 else 0)
    if ret[0] == 0:
        return None
    else:
        return tuple(map(int, ret[1].split(".")))


def CheckPkgConfig(context, pkgs, *args):
    if not SCons.Util.is_List(pkgs):
        pkgs = [pkgs]
    assert pkgs

    for pkg in pkgs:
        context.Message(f"Checking for pkg-config package {pkg}... ")
        ret = context.TryAction(f"pkg-config {pkg}")[0]
        if not ret:
            context.Result(ret)
            continue

        if len(args) == 0:
            break

        cmd = " ".join(["pkg-config"] + list(args) + [pkg])
        try:
            context.env.ParseConfig(cmd)
            ret = 1
            context.Result(ret)
            break
        except Exception as e:
            ret = 0
            context.Result(ret)

    return ret


@contextlib.contextmanager
def Configure(env, *args, **kwargs):
    kwargs.setdefault(
        "conf_dir",
        os.path.join(env["GEM5BUILD"], "scons_config"),
    )
    kwargs.setdefault(
        "log_file",
        os.path.join(env["GEM5BUILD"], "scons_config.log"),
    )
    kwargs.setdefault("custom_tests", {})
    kwargs["custom_tests"].update(
        {
            "CheckCxxFlag": CheckCxxFlag,
            "CheckLinkFlag": CheckLinkFlag,
            "CheckMember": CheckMember,
            "CheckPkgConfig": CheckPkgConfig,
            "CheckPythonLib": CheckPythonLib,
        },
    )
    conf = SCons.Script.Configure(env, *args, **kwargs)

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

    try:
        yield conf
    finally:
        env.Replace(**conf.Finish().Dictionary())
