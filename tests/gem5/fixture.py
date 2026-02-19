# Copyright (c) 2019 ARM Limited
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

import gzip
import os
import shutil
import socket
import sys
import tempfile
import threading
import urllib.error
import urllib.request
from typing import (
    List,
    Optional,
)

import testlib.log as log
from testlib.configuration import (
    config,
    constants,
)
from testlib.fixture import Fixture
from testlib.helper import (
    absdirpath,
    cacheresult,
    joinpath,
    log_call,
)
from testlib.state import Result


class VariableFixture(Fixture):
    def __init__(self, value=None, name=None):
        super().__init__(name=name)
        self.value = value


class TempdirFixture(Fixture):
    def __init__(self):
        self.path = None
        super().__init__(name=constants.tempdir_fixture_name)

    def setup(self, testitem):
        self.path = tempfile.mkdtemp(prefix="gem5out")

    def post_test_procedure(self, testitem):
        suiteUID = testitem.metadata.uid.suite
        testUID = testitem.metadata.name
        testing_result_folder = os.path.join(
            config.result_path, "SuiteUID-" + suiteUID, "TestUID-" + testUID
        )

        # Copy the output files of the run from /tmp to testing-results
        # We want to wipe the entire result folder for this test first. Why?
        #   If the result folder exists (probably from the previous run), if
        #   this run emits fewer files, there'll be files from the previous
        #   run in this folder, which would cause confusion if one does not
        #   check the timestamp of the file.
        if os.path.exists(testing_result_folder):
            shutil.rmtree(testing_result_folder)
        shutil.copytree(self.path, testing_result_folder)

    def teardown(self, testitem):
        if testitem.result == Result.Passed:
            shutil.rmtree(self.path)


class UniqueFixture(Fixture):
    """
    Base class for fixtures that generate a target in the
    filesystem. If the same fixture is used by more than one
    test/suite, rather than creating a copy of the fixture, it returns
    the same object and makes sure that setup is only executed
    once. Devired classses should override the _init and _setup
    functions.

    :param target: The absolute path of the target in the filesystem.

    """

    fixtures = {}

    def __new__(cls, target):
        if target in cls.fixtures:
            obj = cls.fixtures[target]
        else:
            obj = super().__new__(cls)
            obj.lock = threading.Lock()
            obj.target = target
            cls.fixtures[target] = obj
        return obj

    def __init__(self, *args, **kwargs):
        with self.lock:
            if hasattr(self, "_init_done"):
                return
            super().__init__(self, **kwargs)
            self._init(*args, **kwargs)
            self._init_done = True

    def setup(self, testitem):
        with self.lock:
            if hasattr(self, "_setup_done"):
                return
            self._setup_done = True
            self._setup(testitem)


class CMakeFixture(UniqueFixture):
    """
    Fixture that builds gem5 using CMake + Ninja.

    Runs cmake configure and ninja build once for each unique build
    directory. Replaces the former SConsFixture.

    :param directory: The source directory (repo root). If None is
        provided, will choose the config base_dir.
    """

    # Map gem5 variant names to CMake build types
    _variant_to_build_type = {
        "opt": "GEM5_OPT",
        "debug": "GEM5_DEBUG",
        "fast": "GEM5_FAST",
    }

    def __new__(cls, target):
        obj = super().__new__(cls, target)
        return obj

    def _setup(self, testitem):
        if config.skip_build:
            return

        log.test_log.message(
            "Building gem5 with CMake + Ninja. This may take a while."
        )
        log.test_log.message(f"  Build directory: {self.target_dir}")
        log.test_log.message(
            "You may want to use --skip-build, or use 'rerun'."
        )

        build_type = self._variant_to_build_type.get(
            self.variant, f"GEM5_{self.variant.upper()}"
        )

        # CMake configure
        cmake_command = [
            "cmake",
            "-G",
            "Ninja",
            f"-DCMAKE_BUILD_TYPE={build_type}",
            f"-DGEM5_BUILD_VARIANT={self.isa.upper()}",
            "-DGEM5_NO_COMPRESS_DEBUG=ON",
        ]

        # If there is a cache coherence protocol specified,
        # set it via a Kconfig override.
        if self.protocol:
            cmake_command.append(
                f"-DGEM5_KCONFIG_OVERRIDE=RUBY_PROTOCOL_{self.protocol}=y"
            )

        cmake_command.extend([
            "-S",
            self.directory,
            "-B",
            self.target_dir,
        ])
        log_call(log.test_log, cmake_command, time=None, stderr=sys.stderr)

        # Ninja build
        ninja_command = [
            "ninja",
            "-C",
            self.target_dir,
            "-j",
            str(config.threads),
        ]
        log_call(log.test_log, ninja_command, time=None, stderr=sys.stderr)


# Keep SConsFixture as an alias for backward compatibility with any
# out-of-tree test scripts that may reference it directly.
SConsFixture = CMakeFixture


class Gem5Fixture(CMakeFixture):
    def __new__(cls, isa, variant, protocol=None):
        # Build the ISA component (e.g., "ALL", "ALL_MESI_Two_Level")
        isa_component = isa.upper()
        if protocol:
            isa_component += "_" + protocol
        # Append variant so each CMAKE_BUILD_TYPE gets a separate build
        # directory (CMake does not support multiple build types in a
        # single build tree).
        dir_name = f"{isa_component}_{variant}"
        target_dir = joinpath(config.build_dir, dir_name)
        target = joinpath(target_dir, "gem5")
        obj = super().__new__(cls, target)
        obj.target_dir = target_dir
        obj._isa_component = isa_component
        return obj

    def _init(self, isa, variant, protocol=None):
        self.name = constants.gem5_binary_fixture_name

        # CMake produces the binary as 'gem5' (no variant suffix)
        self.path = joinpath(self.target_dir, "gem5")
        self.directory = config.base_dir

        self.isa = isa
        self.variant = variant
        self.protocol = protocol
        self.set_global()

    def get_get_build_info(self) -> Optional[str]:
        # Return variant-qualified path so CI and listing tools can
        # distinguish opt/debug/fast targets.  Format:
        #   build/<ISA>_<variant>/gem5
        # CI parses ISA and variant by splitting on the last underscore.
        return joinpath(self.target_dir, "gem5")


class MakeFixture(Fixture):
    def __init__(self, directory, *args, **kwargs):
        name = f"make -C {directory}"
        super().__init__(
            build_once=True, lazy_init=False, name=name, *args, **kwargs
        )
        self.targets = []
        self.directory = directory

    def setup(self):
        super().setup()
        targets = set(self.required_by)
        command = ["make", "-C", self.directory]
        command.extend([target.target for target in targets])
        log_call(log.test_log, command, time=None, stderr=sys.stderr)


class MakeTarget(Fixture):
    def __init__(self, target, make_fixture=None, *args, **kwargs):
        """
        :param make_fixture: The make invocation we will be attached to.
        Since we don't have a single global instance of make in gem5 we
        need to know what invocation to attach to. If none given, creates
        its own.
        """
        super().__init__(name=target, *args, **kwargs)
        self.target = self.name

        if make_fixture is None:
            make_fixture = MakeFixture(
                absdirpath(target), lazy_init=True, build_once=False
            )

        self.make_fixture = make_fixture

        # Add our self to the required targets of the main MakeFixture
        self.require(self.make_fixture)

    def setup(self, testitem):
        super().setup()
        self.make_fixture.setup()
        return self


class TestProgram(MakeTarget):
    def __init__(self, program, isa, os, recompile=False):
        make_dir = joinpath(config.bin_dir, program)
        make_fixture = MakeFixture(make_dir)
        target = joinpath("bin", isa, os, program)
        super().__init__(target, make_fixture)
        self.path = joinpath(make_dir, target)
        self.recompile = recompile

    def setup(self, testitem):
        # Check if the program exists if it does then only compile if
        # recompile was given.
        if self.recompile:
            super(MakeTarget, self).setup()
        elif not os.path.exists(self.path):
            super(MakeTarget, self).setup()


class DownloadedProgram(UniqueFixture):
    """Like TestProgram, but checks the version in the gem5 binary repository
    and downloads an updated version if it is needed.
    """

    def __new__(cls, url, path, filename, gzip_decompress=False):
        target = joinpath(path, filename)
        return super().__new__(cls, target)

    def _init(self, url, path, filename, gzip_decompress=False, **kwargs):
        """
        url: string
            The url of the archive
        path: string
            The absolute path of the directory containing the archive
        filename: string
            The name of the archive
        gzip_decompress: boolean
            True if this target resource have been compressed using gzip and
            is to be decompressed prior to usage.
        """

        self.url = url
        self.path = path
        self.filename = joinpath(path, filename)
        self.name = "Downloaded:" + self.filename
        self.gzip_decompress = gzip_decompress

    def _download(self):
        import errno

        log.test_log.debug("Downloading " + self.url + " to " + self.path)
        if not os.path.exists(self.path):
            try:
                os.makedirs(self.path)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
        if self.gzip_decompress:
            gzipped_filename = self.filename + ".gz"
            urllib.request.urlretrieve(self.url, gzipped_filename)

            with open(self.filename, "wb") as outfile:
                with gzip.open(gzipped_filename, "r") as infile:
                    shutil.copyfileobj(infile, outfile)

            os.remove(gzipped_filename)
        else:
            urllib.request.urlretrieve(self.url, self.filename)

    def _getremotetime(self):
        import datetime
        import time

        import _strptime  # Needed for python threading bug

        u = urllib.request.urlopen(self.url, timeout=10)

        return time.mktime(
            datetime.datetime.strptime(
                u.info()["Last-Modified"], "%a, %d %b %Y %X GMT"
            ).timetuple()
        )

    def _setup(self, testitem):
        # Check to see if there is a file downloaded
        if not os.path.exists(self.filename):
            self._download()
        else:
            try:
                t = self._getremotetime()
            except (urllib.error.URLError, socket.timeout):
                # Problem checking the server, use the old files.
                log.test_log.debug(
                    "Could not contact server. Binaries may be old."
                )
                return
            # If the server version is more recent, download it
            if t > os.path.getmtime(self.filename):
                self._download()


class DownloadedArchive(DownloadedProgram):
    """Like TestProgram, but checks the version in the gem5 binary repository
    and downloads an updated version if it is needed.
    """

    def _extract(self):
        import tarfile

        with tarfile.open(self.filename) as tf:

            def is_within_directory(directory, target):
                abs_directory = os.path.abspath(directory)
                abs_target = os.path.abspath(target)

                prefix = os.path.commonprefix([abs_directory, abs_target])

                return prefix == abs_directory

            def safe_extract(
                tar, path=".", members=None, *, numeric_owner=False
            ):
                for member in tar.getmembers():
                    member_path = os.path.join(path, member.name)
                    if not is_within_directory(path, member_path):
                        raise Exception("Attempted Path Traversal in Tar File")

                tar.extractall(path, members, numeric_owner=numeric_owner)

            safe_extract(tf, self.path)

    def _setup(self, testitem):
        # Check to see if there is a file downloaded
        if not os.path.exists(self.filename):
            self._download()
            self._extract()
        else:
            try:
                t = self._getremotetime()
            except (urllib.error.URLError, socket.timeout):
                # Problem checking the server, use the old files.
                log.test_log.debug(
                    "Could not contact server. Binaries may be old."
                )
                return
            # If the server version is more recent, download it
            if t > os.path.getmtime(self.filename):
                self._download()
                self._extract()
