#!/usr/bin/env python
#
# Copyright (c) 2016 ARM Limited
# All rights reserved
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
# Authors: Andreas Sandberg

import subprocess
from threading import Timer
import time
import re

class CallTimeoutException(Exception):
    """Exception that indicates that a process call timed out"""

    def __init__(self, status, stdout, stderr):
        self.status = status
        self.stdout = stdout
        self.stderr = stderr

class ProcessHelper(subprocess.Popen):
    """Helper class to run child processes.

    This class wraps a subprocess.Popen class and adds support for
    using it in a with block. When the process goes out of scope, it's
    automatically terminated.

    with ProcessHelper(["/bin/ls"], stdout=subprocess.PIPE) as p:
        return p.call()
    """
    def __init__(self, *args, **kwargs):
        super(ProcessHelper, self).__init__(*args, **kwargs)

    def _terminate_nicely(self, timeout=5):
        def on_timeout():
            self.kill()

        if self.returncode is not None:
            return self.returncode

        timer = Timer(timeout, on_timeout)
        self.terminate()
        status = self.wait()
        timer.cancel()

        return status

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.returncode is None:
            self._terminate_nicely()

    def call(self, timeout=0):
        self._timeout = False
        def on_timeout():
            self._timeout = True
            self._terminate_nicely()

        status, stdout, stderr = None, None, None
        timer = Timer(timeout, on_timeout)
        if timeout:
            timer.start()

        stdout, stderr = self.communicate()
        status = self.wait()

        timer.cancel()

        if self._timeout:
            self._terminate_nicely()
            raise CallTimeoutException(self.returncode, stdout, stderr)
        else:
            return status, stdout, stderr

class FileIgnoreList(object):
    """Helper class to implement file ignore lists.

    This class implements ignore lists using plain string matching and
    regular expressions. In the simplest use case, rules are created
    statically upon initialization:

        ignore_list = FileIgnoreList(name=("ignore_me.txt", ), rex=(r".*~", )

    Ignores can be queried using in the same ways as normal Python
    containers:

        if file_name in ignore_list:
            print "Ignoring %s" % file_name


    New rules can be added at runtime by extending the list in the
    rules attribute:

        ignore_list.rules.append(FileIgnoreList.simple("bar.txt"))
    """

    @staticmethod
    def simple(r):
        return lambda f: f == r

    @staticmethod
    def rex(r):
        re_obj = r if hasattr(r, "search") else re.compile(r)
        return lambda name: re_obj.search(name)

    def __init__(self, names=(), rex=()):
        self.rules = [ FileIgnoreList.simple(n) for n in names ] + \
                     [ FileIgnoreList.rex(r) for r in rex ]

    def __contains__(self, name):
        for rule in self.rules:
            if rule(name):
                return True
        return False

if __name__ == "__main__":
    # Run internal self tests to ensure that the helpers are working
    # properly. The expected output when running this script is
    # "SUCCESS!".

    cmd_foo = [ "/bin/echo", "-n", "foo" ]
    cmd_sleep = [ "/bin/sleep", "10" ]

    # Test that things don't break if the process hasn't been started
    with ProcessHelper(cmd_foo) as p:
        pass

    with ProcessHelper(cmd_foo, stdout=subprocess.PIPE) as p:
        status, stdout, stderr = p.call()
    assert stdout == "foo"
    assert status == 0

    try:
        with ProcessHelper(cmd_sleep) as p:
            status, stdout, stderr = p.call(timeout=1)
        assert False, "Timeout not triggered"
    except CallTimeoutException:
        pass

    ignore_list = FileIgnoreList(
        names=("ignore.txt", "foo/test.txt"),
        rex=(r"~$", re.compile("^#")))

    assert "ignore.txt" in ignore_list
    assert "bar.txt" not in ignore_list
    assert "foo/test.txt" in ignore_list
    assert "test.txt" not in ignore_list
    assert "file1.c~" in ignore_list
    assert "file1.c" not in ignore_list
    assert "#foo" in ignore_list
    assert "foo#" not in ignore_list

    ignore_list.rules.append(FileIgnoreList.simple("bar.txt"))
    assert "bar.txt" in ignore_list

    print "SUCCESS!"
