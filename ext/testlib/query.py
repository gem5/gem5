# Copyright (c) 2017 Mark D. Hill and David A. Wood
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
# Authors: Sean Wilson

import testlib.terminal as terminal
import testlib.log as log

# TODO Refactor print logic out of this so the objects
# created are separate from print logic.
class QueryRunner(object):
    def __init__(self, test_schedule):
        self.schedule = test_schedule

    def tags(self):
        tags = set()
        for suite in self.schedule:
            tags = tags | set(suite.tags)
        return tags

    def suites(self):
        return [suite for suite in self.schedule]

    def suites_with_tag(self, tag):
        return filter(lambda suite: tag in suite.tags, self.suites())

    def list_tests(self):
        log.test_log.message(terminal.separator())
        log.test_log.message("Listing all Test Cases.", bold=True)
        log.test_log.message(terminal.separator())
        for suite in self.schedule:
            for test in suite:
                log.test_log.message(test.uid, machine_readable=True)

    def list_suites(self):
        log.test_log.message(terminal.separator())
        log.test_log.message("Listing all Test Suites.", bold=True)
        log.test_log.message(terminal.separator())
        for suite in self.suites():
            log.test_log.message(suite.uid, machine_readable=True)

    def list_tags(self):
        log.test_log.message(terminal.separator())
        log.test_log.message("Listing all Test Tags.", bold=True)
        log.test_log.message(terminal.separator())

        for tag in self.tags():
            log.test_log.message(tag, machine_readable=True)
