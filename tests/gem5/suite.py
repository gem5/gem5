# Copyright (c) 2020 ARM Limited
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

import os
import copy
import subprocess
import sys

from testlib.test_util import TestFunction
from testlib.suite import TestSuite
from testlib.helper import log_call
from testlib.configuration import constants, config
from .fixture import TempdirFixture, Gem5Fixture, VariableFixture

from . import verifier


def gem5_verify_config(
    name,
    config,
    config_args,
    verifiers,
    gem5_args=tuple(),
    fixtures=[],
    valid_isas=constants.supported_isas,
    valid_variants=constants.supported_variants,
    length=constants.supported_lengths[0],
    valid_hosts=constants.supported_hosts,
    protocol=None,
    uses_kvm=False,
):
    """
    Helper class to generate common gem5 tests using verifiers.

    The generated TestSuite will run gem5 with the provided config and
    config_args. After that it will run any provided verifiers to verify
    details about the gem5 run.

    .. seealso::  For the verifiers see :mod:`testlib.gem5.verifier`

    :param name: Name of the test.
    :param config: The config to give gem5.
    :param config_args: A list of arguments to pass to the given config.

    :param verifiers: An iterable with Verifier instances which will be placed
        into a suite that will be ran after a gem5 run.

    :param gem5_args: An iterable with arguments to give to gem5. (Arguments
        that would normally go before the config path.)

    :param valid_isas: An iterable with the isas that this test can be ran
        for. If None given, will run for all supported_isas.

    :param valid_variants: An iterable with the variant levels that
        this test can be ran for. (E.g. opt, debug)

    :param uses_kvm: States if this verifier uses KVM. If so, the "kvm" tag
        will be included.
    """
    fixtures = list(fixtures)
    testsuites = []

    for host in valid_hosts:
        for opt in valid_variants:
            for isa in valid_isas:
                # Create a tempdir fixture to be shared throughout the test.
                tempdir = TempdirFixture()
                gem5_returncode = VariableFixture(
                    name=constants.gem5_returncode_fixture_name
                )

                # Common name of this generated testcase.
                _name = f"{name}-{isa}-{host}-{opt}"
                if protocol:
                    _name += "-" + protocol

                # Create the running of gem5 subtest.  NOTE: We specifically
                # create this test before our verifiers so this is listed
                # first.
                tests = []
                gem5_execution = TestFunction(
                    _create_test_run_gem5(config, config_args, gem5_args),
                    name=_name,
                )
                tests.append(gem5_execution)

                # Create copies of the verifier subtests for this isa and
                # variant.
                for verifier in verifiers:
                    tests.append(verifier.instantiate_test(_name))

                # Add the isa and variant to tags list.
                tags = [isa, opt, length, host]

                if uses_kvm:
                    tags.append(constants.kvm_tag)

                # Create the gem5 target for the specific architecture and
                # variant.
                _fixtures = copy.copy(fixtures)
                _fixtures.append(Gem5Fixture(isa, opt, protocol))
                _fixtures.append(tempdir)
                _fixtures.append(gem5_returncode)

                # Finally construct the self contained TestSuite out of our
                # tests.
                testsuites.append(
                    TestSuite(
                        name=_name, fixtures=_fixtures, tags=tags, tests=tests
                    )
                )
    return testsuites


def _create_test_run_gem5(config, config_args, gem5_args):
    def test_run_gem5(params):
        """
        Simple \'test\' which runs gem5 and saves the result into a tempdir.

        NOTE: Requires fixtures: tempdir, gem5
        """
        fixtures = params.fixtures

        if gem5_args is None:
            _gem5_args = tuple()
        elif isinstance(gem5_args, str):
            # If just a single str, place it in an iterable
            _gem5_args = (gem5_args,)
        else:
            _gem5_args = gem5_args

        # FIXME/TODO: I don't like the idea of having to modify this test run
        # or always collect results even if not using a verifier. There should
        # be some configuration in here that only gathers certain results for
        # certain verifiers.
        #
        # I.E. Only the returncode verifier will use the gem5_returncode
        # fixture, but we always require it even if that verifier isn't being
        # ran.
        tempdir = fixtures[constants.tempdir_fixture_name].path
        gem5 = fixtures[constants.gem5_binary_fixture_name].path
        command = [
            gem5,
            "-d",  # Set redirect dir to tempdir.
            tempdir,
            "-re",  # TODO: Change to const. Redirect stdout and stderr
            "--silent-redirect",
        ]
        command.extend(_gem5_args)
        command.append(config)
        # Config_args should set up the program args.
        command.extend(config_args)
        log_call(
            params.log,
            command,
            time=params.time,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

    return test_run_gem5
