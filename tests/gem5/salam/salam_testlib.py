# Copyright (c) 2026 Akanksha Chaudhari
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

# Shared fixtures and helpers for the SALAM TestLib suites.
#
# These suites build gem5 with --with-salam and build their workloads
# from a fresh copy of the tracked sources. Every subprocess runs in a
# minimal environment made from explicitly selected tools, so compiler
# variables exported by the calling shell never reach a build.

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import threading

import testlib.log as log
from testlib.configuration import (
    config,
    constants,
)
from testlib.fixture import Fixture
from testlib.helper import log_call

from gem5.fixture import UniqueFixture
from gem5.verifier import MatchFileRegex

SALAM_TAG = "salam"
BENCH_DIR = "configs/example/gem5_library/salam-benchmarks/src"
DEFAULT_PROFILE = "src/salam/hw_profiles/default/salam-hw-config.yml"
PROVENANCE_NAME = "salam-test-provenance.json"

# Options used by util/SALAM-tools/run_system.sh.
FS_ARGS = [
    "--mem-size=16GB",
    "--mem-type=DDR4_2400_8x8",
    "--machine-type=VExpress_GEM5_V1",
    "--dtb-file=none",
    "--bare-metal",
    "--cpu-type=DerivO3CPU",
    "--caches",
    "--l2cache",
]

M5_EXIT_REGEX = r"^Exiting @ tick \d+ because m5_exit instruction encountered"


class SalamTestError(Exception):
    pass


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _which(name):
    # Keep the path as found, without resolving symlinks: SCons
    # rebuilds everything when the compiler path changes.
    path = shutil.which(name)
    if path is None:
        raise SalamTestError(f"required tool not found: {name}")
    return os.path.abspath(path)


def _output(argv, env=None, cwd=None):
    return subprocess.run(
        argv,
        env=env,
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    ).stdout


class Toolchain:
    # Absolute tool paths resolved from the calling environment before
    # any environment is sanitized.

    def __init__(self):
        llvm_config = os.environ.get("LLVM_CONFIG")
        if llvm_config:
            if not os.path.isfile(llvm_config):
                raise SalamTestError(f"LLVM_CONFIG not found: {llvm_config}")
            self.llvm_config = os.path.abspath(llvm_config)
        else:
            self.llvm_config = _which("llvm-config")
        self.gcc = _which("gcc")
        self.gxx = _which("g++")
        self.scons = _which("scons")
        self.make = _which("make")
        self.timeout = _which("timeout")
        self.readelf = _which("readelf")
        self.git = _which("git")
        self.python = os.path.abspath(sys.executable)
        self.arm = {
            tool: _which("arm-none-eabi-" + tool)
            for tool in ("gcc", "g++", "as", "ld", "objcopy", "cpp")
        }
        self.llvm_version = _output([self.llvm_config, "--version"]).strip()
        self.llvm_major = int(self.llvm_version.split(".")[0])
        self.clang = self._find_clang()
        for lib in ("libc.a", "libstdc++.a"):
            found = _output(
                [self.arm["gcc"], "-print-file-name=" + lib]
            ).strip()
            if not os.path.isabs(found) or not os.path.isfile(found):
                raise SalamTestError(
                    f"arm-none-eabi toolchain is missing {lib} "
                    "(install newlib and libstdc++ for arm-none-eabi)"
                )

    def _find_clang(self):
        candidates = [f"clang-{self.llvm_major}", "clang"]
        for name in candidates:
            path = shutil.which(name)
            if path is None:
                continue
            version = _output([path, "--version"])
            match = re.search(r"clang version (\d+)\.", version)
            if match and int(match.group(1)) == self.llvm_major:
                return os.path.abspath(path)
        raise SalamTestError(
            f"no clang with LLVM major {self.llvm_major} "
            f"(the LLVM selected by {self.llvm_config})"
        )

    def env(self, extra=None, path_prefix=()):
        dirs = list(path_prefix)
        tools = [
            self.llvm_config,
            self.gcc,
            self.gxx,
            self.scons,
            self.make,
            self.timeout,
            self.git,
            self.python,
            self.clang,
        ] + list(self.arm.values())
        for tool in tools:
            directory = os.path.dirname(tool)
            if directory not in dirs:
                dirs.append(directory)
        for directory in ("/usr/bin", "/bin"):
            if directory not in dirs:
                dirs.append(directory)
        env = {
            "PATH": os.pathsep.join(dirs),
            "HOME": os.environ.get("HOME", "/tmp"),
            "TMPDIR": os.environ.get("TMPDIR", "/tmp"),
            "LANG": "C",
            "LC_ALL": "C",
            "CC": self.gcc,
            "CXX": self.gxx,
            "LLVM_CONFIG": self.llvm_config,
        }
        if extra:
            env.update(extra)
        return env

    def describe(self):
        versions = {}
        for name, argv in (
            ("llvm-config", [self.llvm_config, "--version"]),
            ("clang", [self.clang, "--version"]),
            ("gcc", [self.gcc, "--version"]),
            ("arm-none-eabi-gcc", [self.arm["gcc"], "--version"]),
            ("make", [self.make, "--version"]),
            ("scons", [self.scons, "--version"]),
        ):
            versions[name] = _output(argv).splitlines()[0:2]
        return {
            "llvm_config": self.llvm_config,
            "llvm_version": self.llvm_version,
            "clang": self.clang,
            "gcc": self.gcc,
            "gxx": self.gxx,
            "arm": self.arm,
            "python": self.python,
            "versions": versions,
        }


_toolchain = None
_toolchain_lock = threading.Lock()


def toolchain():
    global _toolchain
    with _toolchain_lock:
        if _toolchain is None:
            _toolchain = Toolchain()
        return _toolchain


def _expected_generated(tools, profile):
    # Render the sources the generator produces for this profile.
    generator = os.path.join(config.base_dir, "util/SALAM-tools/hw_generator")
    script = (
        "import hashlib, json, sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "import SALAMGeneratedSources as g\n"
        "model = g.load_generated_sources(g.load_source_config(sys.argv[2]))\n"
        "g.require_compatible_functional_units(model)\n"
        "out = g.generate_sources(model)\n"
        "print(json.dumps({k: hashlib.sha256(v.encode()).hexdigest()"
        " for k, v in out.items()}))\n"
    )
    text = _output(
        [tools.python, "-c", script, generator, profile], env=tools.env()
    )
    return json.loads(text)


def _built_generated(build_dir):
    root = os.path.join(build_dir, "salam", "generated", "salam")
    built = {}
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            # Only generated sources; the build adds objects and
            # compiled Python next to them.
            if not re.search(r"\.(hh|cc|py)$", name) or name.endswith(
                ".py.cc"
            ):
                continue
            path = os.path.join(dirpath, name)
            built[os.path.relpath(path, root)] = sha256_file(path)
    return built


class SalamGem5Fixture(UniqueFixture):
    # Builds build/ARM/gem5.opt with --with-salam for one hardware
    # profile and verifies which profile the binary was built from.

    def __new__(cls, profile=None):
        profile = os.path.abspath(
            profile or os.path.join(config.base_dir, DEFAULT_PROFILE)
        )
        target = joinpath_key(profile)
        obj = super().__new__(cls, target)
        obj.profile = profile
        return obj

    def _init(self, profile=None):
        self.name = "salam-gem5"
        self.build_dir = os.path.join(config.build_dir, "ARM")
        self.path = os.path.join(self.build_dir, "gem5.opt")
        self.provenance_path = os.path.join(self.build_dir, PROVENANCE_NAME)
        self.default_profile = self.profile == os.path.abspath(
            os.path.join(config.base_dir, DEFAULT_PROFILE)
        )
        self.tools = None
        self.error = None

    def setup(self, testitem):
        # A failed build must fail every suite that uses it.
        with self.lock:
            if not hasattr(self, "_setup_done"):
                self._setup_done = True
                try:
                    self._setup(testitem)
                except Exception as exc:
                    self.error = exc
                    raise
            elif self.error is not None:
                raise SalamTestError(f"SALAM build failed: {self.error}")

    def scons_args(self):
        # gem5 resolves relative targets against the launch directory.
        args = [self.path, "--with-salam"]
        if not self.default_profile:
            args.append("--salam-hw-config=" + self.profile)
        return args

    def _setup(self, testitem):
        self.tools = toolchain()
        expected = _expected_generated(self.tools, self.profile)
        if config.skip_build:
            self._check_provenance(expected)
        else:
            command = [
                self.tools.scons,
                "-C",
                config.base_dir,
                "-j",
                str(config.threads),
            ] + self.scons_args()
            log_call(
                log.test_log,
                command,
                time=None,
                env=self.tools.env(),
                stderr=sys.stderr,
            )
            self._write_provenance(expected)
        self._check_generated(expected)
        self._probe()

    def _provenance(self, expected):
        return {
            "scons_args": self.scons_args(),
            "profile": self.profile,
            "generated": expected,
            "toolchain": self.tools.describe(),
            "binary_sha256": sha256_file(self.path),
            "libllvm": self._needed_llvm(),
        }

    def _write_provenance(self, expected):
        with open(self.provenance_path, "w") as f:
            json.dump(self._provenance(expected), f, indent=1, sort_keys=True)

    def _check_provenance(self, expected):
        if not os.path.isfile(self.provenance_path):
            raise SalamTestError(
                f"--skip-build needs {self.provenance_path}; "
                "run once without --skip-build"
            )
        with open(self.provenance_path) as f:
            recorded = json.load(f)
        current = self._provenance(expected)
        for key in ("scons_args", "profile", "generated", "binary_sha256"):
            if recorded.get(key) != current[key]:
                raise SalamTestError(
                    f"build provenance mismatch in {key}; rebuild without "
                    "--skip-build"
                )
        if recorded["toolchain"]["llvm_config"] != self.tools.llvm_config:
            raise SalamTestError(
                "the binary was built with "
                f"{recorded['toolchain']['llvm_config']}, not "
                f"{self.tools.llvm_config}"
            )

    def _check_generated(self, expected):
        built = _built_generated(self.build_dir)
        if built != expected:
            missing = sorted(set(expected) - set(built))
            extra = sorted(set(built) - set(expected))
            changed = sorted(
                k
                for k in set(built) & set(expected)
                if built[k] != expected[k]
            )
            raise SalamTestError(
                "generated SALAM sources do not match profile "
                f"{self.profile}: missing={missing[:5]} extra={extra[:5]} "
                f"changed={changed[:5]}"
            )

    def _needed_llvm(self):
        dynamic = _output([self.tools.readelf, "-d", self.path])
        return sorted(set(re.findall(r"\[(libLLVM[^\]]*)\]", dynamic)))

    def _probe(self):
        probe = os.path.join(
            os.path.dirname(__file__), "check_salam_binary.py"
        )
        log_call(
            log.test_log,
            [
                self.path,
                "-d",
                os.path.join(self.build_dir, "salam-probe"),
                probe,
            ],
            time=None,
            env=self.tools.env(),
        )


def joinpath_key(profile):
    return os.path.join(config.build_dir, "ARM", "gem5.opt") + "::" + profile


class SalamWorkloadFixture(Fixture):
    # Builds a workload from a fresh copy of the tracked sources.
    #
    # sources: tracked directories (relative to BENCH_DIR) to copy.
    # edits: functions applied to the copy before it is built.

    def __init__(self, name, gem5, tempdir, bench, sources, edits=()):
        super().__init__(name=name)
        self.gem5 = gem5
        self.tempdir = tempdir
        self.bench = bench
        self.sources = sources
        self.edits = edits
        self.root = None
        self.manifest = {}

    def setup(self, testitem):
        tools = self.gem5.tools
        self.root = os.path.join(self.tempdir.path, "workload")
        src = os.path.join(self.root, "src")
        copied = self._copy_sources(tools, src)
        for edit in self.edits:
            edit(src)
        shim = os.path.join(self.root, "bin")
        os.makedirs(shim)
        os.symlink(tools.clang, os.path.join(shim, "clang"))
        env = tools.env(extra={"M5_PATH": config.base_dir}, path_prefix=[shim])
        self.build_env = env
        self._check_header(tools, src, env)
        self.build_log = os.path.join(self.root, "build.log")
        with open(self.build_log, "w") as logf:
            subprocess.run(
                [tools.make, "-C", os.path.join(src, self.bench), "build"],
                env=env,
                check=True,
                stdout=logf,
                stderr=subprocess.STDOUT,
            )
        self.elf = os.path.join(src, self.bench, "sw", "main.elf")
        self.accpath = os.path.join(src, self.bench)
        outputs = {}
        for rel in ("sw/main.elf", "hw/" + self.bench + ".ll", "hw/top.ll"):
            path = os.path.join(self.accpath, rel)
            if os.path.isfile(path):
                outputs[rel] = sha256_file(path)
        git = [tools.git, "-C", config.base_dir]
        self.manifest = {
            "toolchain": tools.describe(),
            "env": env,
            "git_head": _output(git + ["rev-parse", "HEAD"]).strip(),
            "git_tree": _output(git + ["rev-parse", "HEAD^{tree}"]).strip(),
            "git_status": _output(git + ["status", "--porcelain"]),
            "inputs": copied,
            "outputs": outputs,
            "build_log": self.build_log,
            "gem5": self.gem5.path,
            "gem5_sha256": sha256_file(self.gem5.path),
        }
        with open(os.path.join(self.root, "manifest.json"), "w") as f:
            json.dump(self.manifest, f, indent=1, sort_keys=True)

    def _copy_sources(self, tools, src):
        copied = {}
        listing = _output(
            [tools.git, "-C", config.base_dir, "ls-files", "-z", "--"]
            + [os.path.join(BENCH_DIR, d) for d in self.sources]
        )
        for rel in filter(None, listing.split("\0")):
            if re.search(r"\.(o|elf|ll)$", rel):
                raise SalamTestError(
                    f"tracked build product in sources: {rel}"
                )
            dest = os.path.join(src, os.path.relpath(rel, BENCH_DIR))
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.copy2(os.path.join(config.base_dir, rel), dest)
            copied[rel] = sha256_file(dest)
        if not copied:
            raise SalamTestError(f"no tracked sources under {self.sources}")
        return copied

    def _check_header(self, tools, src, env):
        header = os.path.join(
            src, self.bench, self.bench + "_clstr_hw_defines.h"
        )
        before = sha256_file(header)
        builder = os.path.join(
            config.base_dir,
            "util/SALAM-tools/SALAM-Configurator/systembuilder.py",
        )
        subprocess.run(
            [
                tools.python,
                builder,
                "--sys-name",
                self.bench,
                "--bench-path",
                self.bench,
                "--config-name",
                "config.yml",
            ],
            env=dict(env, ACC_BENCH_PATH=src),
            check=True,
            capture_output=True,
        )
        if sha256_file(header) != before:
            raise SalamTestError(
                f"systembuilder changed the tracked header for {self.bench}"
            )


def replace_once(path, old, new):
    with open(path) as f:
        text = f.read()
    count = text.count(old)
    if count != 1:
        raise SalamTestError(f"expected one {old!r} in {path}, found {count}")
    with open(path, "w") as f:
        f.write(text.replace(old, new))


def enable_check(bench):
    def edit(src):
        replace_once(
            os.path.join(src, bench, "defines.h"),
            "// #define CHECK",
            "#define CHECK",
        )

    return edit


def run_fs(workload, wall, ticks):
    # Runs configs/SALAM/fs.py on a built workload.

    def test(params):
        gem5 = params.fixtures["salam-gem5"]
        wl = params.fixtures[workload]
        outdir = params.fixtures[constants.tempdir_fixture_name].path
        command = [
            gem5.tools.timeout,
            "--kill-after=30",
            str(wall),
            gem5.path,
            "-d",
            outdir,
            "-re",
            "--silent-redirect",
            os.path.join(config.base_dir, "configs/SALAM/fs.py"),
            "--kernel=" + wl.elf,
            "--accpath=" + wl.accpath,
            "--accbench=" + wl.bench,
            "--acccfg=config.yml",
            "--abs-max-tick=" + str(ticks),
        ] + FS_ARGS
        log_call(
            params.log,
            command,
            time=params.time,
            env=gem5.tools.env(extra={"M5_PATH": config.base_dir}),
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

    return test


class NoMatchFileRegex(MatchFileRegex):
    # Fails if the pattern is found in any of the files.

    def test(self, params):
        tempdir = params.fixtures[constants.tempdir_fixture_name].path
        for fname in self.filenames:
            if self.parse_file(os.path.join(tempdir, fname)):
                raise AssertionError(f"unexpected match in {fname}")
