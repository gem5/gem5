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


def _run_logged(step, argv, env, log_path):
    # Run a build step, keeping its output for the failure report.
    with open(log_path, "a") as logf:
        logf.write("$ " + " ".join(argv) + "\n")
        logf.flush()
        result = subprocess.run(
            argv, env=env, stdout=logf, stderr=subprocess.STDOUT
        )
    if result.returncode != 0:
        with open(log_path) as logf:
            tail = "".join(logf.readlines()[-30:])
        raise SalamTestError(
            f"{step} failed ({result.returncode}); see {log_path}:\n{tail}"
        )


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
        "out = g.generate_sources(model)\n"
        "print(json.dumps({k: hashlib.sha256(v.encode()).hexdigest()"
        " for k, v in out.items()}))\n"
    )
    text = _output(
        [tools.python, "-c", script, generator, profile], env=tools.env()
    )
    return json.loads(text)


def _expected_classes(tools, profile):
    generator = os.path.join(config.base_dir, "util/SALAM-tools/hw_generator")
    script = (
        "import json, sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "import SALAMGeneratedSources as g\n"
        "model = g.load_generated_sources(g.load_source_config(sys.argv[2]))\n"
        "print(json.dumps(model.fu_class_names + model.instruction_class_names))\n"
    )
    return json.loads(
        _output(
            [tools.python, "-c", script, generator, profile], env=tools.env()
        )
    )


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
        missing = sorted(set(expected) - set(built))
        changed = sorted(
            k for k in set(built) & set(expected) if built[k] != expected[k]
        )
        # An earlier profile's files may remain. Current-profile contents
        # and the executable's registered classes are what must match.
        if missing or changed:
            raise SalamTestError(
                "generated SALAM sources do not match profile "
                f"{self.profile}: missing={missing[:5]} changed={changed[:5]}"
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
                json.dumps(_expected_classes(self.tools, self.profile)),
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
        _run_logged(
            "make",
            [tools.make, "-C", os.path.join(src, self.bench), "build"],
            env,
            self.build_log,
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
        _run_logged(
            "systembuilder",
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
            dict(env, ACC_BENCH_PATH=src),
            os.path.join(self.root, "systembuilder.log"),
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


def _outdir(params, subdir=None):
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    return os.path.join(outdir, subdir) if subdir else outdir


def _fs_command(
    params,
    workload,
    wall,
    ticks,
    fs_args=(),
    gem5_args=(),
    wrapper=None,
    subdir=None,
):
    gem5 = params.fixtures["salam-gem5"]
    wl = params.fixtures[workload]
    outdir = _outdir(params, subdir)
    command = [
        gem5.tools.timeout,
        "--kill-after=30",
        str(wall),
        gem5.path,
        "-d",
        outdir,
        "-re",
        "--silent-redirect",
        *gem5_args,
        *([wrapper] if wrapper else []),
        os.path.join(config.base_dir, "configs/SALAM/fs.py"),
        "--kernel=" + wl.elf,
        "--accpath=" + wl.accpath,
        "--accbench=" + wl.bench,
        "--acccfg=config.yml",
        "--abs-max-tick=" + str(ticks),
    ] + FS_ARGS
    command += fs_args
    env = gem5.tools.env(extra={"M5_PATH": config.base_dir})
    return command, env, outdir


def run_fs(
    workload,
    wall,
    ticks,
    fs_args=(),
    gem5_args=(),
    wrapper=None,
    subdir=None,
):
    # Runs configs/SALAM/fs.py on a built workload; fs_args are extra
    # fs.py options, such as --param overrides, gem5_args are gem5
    # options, such as --debug-flags, and wrapper is a config script that
    # runs fs.py. subdir, if given, is the output directory within the
    # test's temporary directory.

    def test(params):
        command, env, _outdir = _fs_command(
            params, workload, wall, ticks, fs_args, gem5_args, wrapper, subdir
        )
        log_call(
            params.log,
            command,
            time=params.time,
            env=env,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

    return test


STDLIB_EXAMPLE = "configs/example/gem5_library/salam/arm-baremetal-salam.py"
NO_SOURCE_TREE = os.path.join(os.path.dirname(__file__), "no_source_tree.py")


def run_stdlib(
    workload, wall, ticks, gem5_args=(), subdir=None, config_args=()
):
    # Runs the standard library SALAM example on a built workload, with
    # the source tree's configs/ and util/ off the module path.

    def test(params):
        gem5 = params.fixtures["salam-gem5"]
        wl = params.fixtures[workload]
        command = [
            gem5.tools.timeout,
            "--kill-after=30",
            str(wall),
            gem5.path,
            "-d",
            _outdir(params, subdir),
            "-re",
            "--silent-redirect",
            *gem5_args,
            NO_SOURCE_TREE,
            os.path.join(config.base_dir, STDLIB_EXAMPLE),
            "--binary",
            wl.elf,
            "--accpath",
            wl.accpath,
            "--acccfg",
            "config.yml",
            "--max-ticks",
            str(ticks),
            *config_args,
        ]
        log_call(
            params.log,
            command,
            time=params.time,
            env=gem5.tools.env(),
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

    return test


def run_fs_rejected(
    workload, wall, ticks, regex, fs_args=(), config_regex=None
):
    # Runs fs.py and requires gem5 to stop with an error matching regex,
    # not by a timeout. config_regex, if given, must match config.ini
    # first: the configuration really holds what is being rejected.

    def test(params):
        command, env, outdir = _fs_command(
            params, workload, wall, ticks, fs_args
        )
        result = subprocess.run(command, env=env)
        if result.returncode in (0, 124, 137):
            raise AssertionError(
                f"expected gem5 to reject the run, got {result.returncode}"
            )
        if config_regex is not None:
            ini = os.path.join(outdir, "config.ini")
            if not os.path.isfile(ini) or not re.search(
                config_regex, open(ini).read(), re.MULTILINE
            ):
                raise SalamTestError(
                    f"precondition not met: no {config_regex!r} in config.ini"
                )
        for name in ("simerr.txt", "simout.txt"):
            path = os.path.join(outdir, name)
            if os.path.isfile(path) and re.search(
                regex, open(path).read(), re.MULTILINE
            ):
                return
        raise AssertionError(f"no {regex!r} in gem5 output")

    return test


def read_stats(outdir):
    # Every statistics dump in stats.txt, as a list of {name: value}.
    dumps = []
    with open(os.path.join(outdir, "stats.txt")) as f:
        for line in f:
            if "Begin Simulation Statistics" in line:
                dumps.append({})
            elif dumps and line.strip() and not line.startswith("-"):
                fields = line.split()
                try:
                    dumps[-1][fields[0]] = float(fields[1])
                except (IndexError, ValueError):
                    pass
    return dumps


def read_simout(params):
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    with open(os.path.join(outdir, "simout.txt")) as f:
        return outdir, f.read()


class CountFileRegex(MatchFileRegex):
    # Requires exactly count matching lines across the files.

    def __init__(self, regex, filenames, count):
        super().__init__(regex, filenames)
        self.count = count

    def test(self, params):
        tempdir = params.fixtures[constants.tempdir_fixture_name].path
        found = 0
        for fname in self.filenames:
            with open(os.path.join(tempdir, fname)) as f:
                found += sum(
                    1 for line in f if any(r.search(line) for r in self.regex)
                )
        if found != self.count:
            patterns = [r.pattern for r in self.regex]
            raise AssertionError(
                f"{patterns!r}: {found} lines, not {self.count}"
            )


class NoMatchFileRegex(MatchFileRegex):
    # Fails if the pattern is found in any of the files.

    def test(self, params):
        tempdir = params.fixtures[constants.tempdir_fixture_name].path
        for fname in self.filenames:
            if self.parse_file(os.path.join(tempdir, fname)):
                raise AssertionError(f"unexpected match in {fname}")


HARNESS_DIR = "tests/gem5/salam/harness"
HARNESS_FROM_BFS = ("sw/boot.s", "sw/boot.ld", "sw/isr.c")


def _copy_tracked(tools, paths, dest_for):
    # Copy tracked files under paths; dest_for maps a tracked path to
    # its destination, or None to skip it.
    copied = {}
    listing = _output(
        [tools.git, "-C", config.base_dir, "ls-files", "-z", "--"] + paths
    )
    for rel in filter(None, listing.split("\0")):
        dest = dest_for(rel)
        if dest is None:
            continue
        if re.search(r"\.(o|elf|ll)$", rel):
            raise SalamTestError(f"tracked build product in sources: {rel}")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        shutil.copy2(os.path.join(config.base_dir, rel), dest)
        copied[rel] = sha256_file(dest)
    return copied


class SalamProfileFixture(Fixture):
    # Writes a copy of the default hardware profile, changed by
    # edit(profile_dir), so that config_path is its salam-hw-config.yml.
    # A SalamGem5Fixture for config_path then builds it.

    def __init__(self, config_path, edit):
        super().__init__(name="salam-profile")
        self.config_path = config_path
        self.edit = edit

    def setup(self, testitem):
        root = os.path.dirname(self.config_path)
        if os.path.basename(self.config_path) != "salam-hw-config.yml":
            raise SalamTestError(f"unexpected profile path {self.config_path}")
        shutil.rmtree(root, ignore_errors=True)
        shutil.copytree(
            os.path.dirname(os.path.join(config.base_dir, DEFAULT_PROFILE)),
            root,
        )
        self.edit(root)


class SalamHarnessFixture(Fixture):
    # Builds the single-accelerator kernel harness around one kernel.
    #
    # kernel: a .c file, or a .ll.in template, in harness/kernels.
    # inputs: int32 values written to IN, or C expressions over the
    # generated defines (such as "IN + 8"); n_out: OUT words printed.
    # hw_config: optional cycle overrides for the "kernel" profile.
    # cache_size: if set, the accelerator gets a Cache var of that size.
    # clocks: if set, the cluster's Clocks item.
    # host_options: macros for the host program, such as
    #   {"STATS_RESET_AFTER_START": 1, "STATS_DUMPS_AT_END": 2}.
    # ir_patterns: regexes the generated kernel.ll must contain.

    def __init__(
        self,
        name,
        gem5,
        tempdir,
        kernel,
        inputs,
        n_out,
        runs=1,
        hw_config=None,
        ir_patterns=(),
        cache_size=None,
        clocks=None,
        host_options=None,
    ):
        super().__init__(name=name)
        self.gem5 = gem5
        self.tempdir = tempdir
        self.kernel = kernel
        self.inputs = list(inputs)
        self.n_out = n_out
        self.runs = runs
        self.hw_config = hw_config
        self.ir_patterns = ir_patterns
        self.cache_size = cache_size
        self.clocks = clocks
        self.host_options = dict(host_options or {})
        self.bench = "harness"

    def setup(self, testitem):
        tools = self.gem5.tools
        self.root = os.path.join(self.tempdir.path, "workload")
        src = os.path.join(self.root, "src")
        self.accpath = os.path.join(src, self.bench)
        bfs = os.path.join(BENCH_DIR, "bfs") + "/"
        copied = _copy_tracked(
            tools,
            [os.path.join(BENCH_DIR, "common"), HARNESS_DIR]
            + [bfs + rel for rel in HARNESS_FROM_BFS],
            lambda rel: self._dest(src, rel),
        )
        self._write_inputs()
        if self.cache_size is not None:
            self._add_cache()
        if self.clocks is not None:
            self._add_clocks()
        shim = os.path.join(self.root, "bin")
        os.makedirs(shim)
        os.symlink(tools.clang, os.path.join(shim, "clang"))
        env = tools.env(extra={"M5_PATH": config.base_dir}, path_prefix=[shim])
        self.build_env = env
        builder = os.path.join(
            config.base_dir,
            "util/SALAM-tools/SALAM-Configurator/systembuilder.py",
        )
        _run_logged(
            "systembuilder",
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
            dict(env, ACC_BENCH_PATH=src),
            os.path.join(self.root, "systembuilder.log"),
        )
        self._place_kernel(tools)
        self.build_log = os.path.join(self.root, "build.log")
        _run_logged(
            "make",
            [tools.make, "-C", self.accpath, "build"],
            env,
            self.build_log,
        )
        # The timing overrides do not affect the header or the build.
        if self.hw_config is not None:
            self._write_hw_config()
        self.ir = os.path.join(self.accpath, "hw", "kernel.ll")
        with open(self.ir) as f:
            ir_text = f.read()
        for pattern in self.ir_patterns:
            if not re.search(pattern, ir_text, re.MULTILINE):
                raise SalamTestError(
                    f"reproducer precondition not met: {pattern!r} is not "
                    f"in {self.kernel}'s IR"
                )
        self.elf = os.path.join(self.accpath, "sw", "main.elf")
        self.manifest = {
            "toolchain": tools.describe(),
            "env": env,
            "kernel": self.kernel,
            "inputs": self.inputs,
            "hw_config": self.hw_config,
            "cache_size": self.cache_size,
            "clocks": self.clocks,
            "host_options": self.host_options,
            "copied": copied,
            "outputs": {
                "hw/kernel.ll": sha256_file(self.ir),
                "sw/main.elf": sha256_file(self.elf),
            },
            "gem5_sha256": sha256_file(self.gem5.path),
        }
        with open(os.path.join(self.root, "manifest.json"), "w") as f:
            json.dump(self.manifest, f, indent=1, sort_keys=True)

    def _dest(self, src, rel):
        if rel.startswith(HARNESS_DIR + "/kernels/"):
            return None
        if rel.startswith(HARNESS_DIR + "/"):
            return os.path.join(
                src, self.bench, os.path.relpath(rel, HARNESS_DIR)
            )
        bfs = os.path.join(BENCH_DIR, "bfs") + "/"
        if rel.startswith(bfs):
            return os.path.join(src, self.bench, os.path.relpath(rel, bfs))
        return os.path.join(src, os.path.relpath(rel, BENCH_DIR))

    def _write_inputs(self):
        path = os.path.join(self.accpath, "sw", "inputs.h")
        values = (
            ", ".join(
                f"(int32_t)({v if isinstance(v, str) else int(v)})"
                for v in self.inputs
            )
            or "0"
        )
        with open(path, "w") as f:
            f.write(
                f"#define N_IN {len(self.inputs)}\n"
                f"#define N_OUT {self.n_out}\n"
                f"#define N_RUNS {self.runs}\n"
                + "".join(
                    f"#define {name} {value}\n"
                    for name, value in sorted(self.host_options.items())
                )
                + f"static const int32_t inputs[] = {{{values}}};\n"
            )

    def _add_cache(self):
        import yaml

        path = os.path.join(self.accpath, "config.yml")
        with open(path) as f:
            document = yaml.safe_load(f)
        cache = {"Name": "Cache", "Type": "Cache", "Size": self.cache_size}
        for item in document["acc_cluster"]:
            if "Accelerator" in item:
                item["Accelerator"].append({"Var": [cache]})
        with open(path, "w") as f:
            f.write("---\n")
            yaml.safe_dump(document, f, sort_keys=False)

    def _add_clocks(self):
        import yaml

        path = os.path.join(self.accpath, "config.yml")
        with open(path) as f:
            document = yaml.safe_load(f)
        document["acc_cluster"].append({"Clocks": dict(self.clocks)})
        with open(path, "w") as f:
            f.write("---\n")
            yaml.safe_dump(document, f, sort_keys=False)

    def _write_hw_config(self):
        import yaml

        path = os.path.join(self.accpath, "config.yml")
        with open(path) as f:
            document = yaml.safe_load(f)
        document["hw_config"] = {"kernel": self.hw_config}
        with open(path, "w") as f:
            f.write("---\n")
            yaml.safe_dump(document, f, sort_keys=False)

    def _place_kernel(self, tools):
        source = os.path.join(
            config.base_dir, HARNESS_DIR, "kernels", self.kernel
        )
        hw = os.path.join(self.accpath, "hw")
        if self.kernel.endswith(".c"):
            shutil.copy2(source, os.path.join(hw, "kernel.c"))
            return
        if not self.kernel.endswith(".ll.in"):
            raise SalamTestError(f"unknown kernel kind: {self.kernel}")
        header = os.path.join(self.accpath, "harness_clstr_hw_defines.h")
        with open(header) as f:
            defines = dict(
                re.findall(r"#define (\w+) (0x[0-9a-fA-F]+)", f.read())
            )
        ptr = "i32*" if tools.llvm_major < 15 else "ptr"
        with open(source) as f:
            text = f.read()
        text = text.replace("{PTR_I32}", ptr)
        text = text.replace(
            "{PTR_I8}", "i8*" if tools.llvm_major < 15 else "ptr"
        )
        text = text.replace(
            "{PTR_AS1}",
            (
                "i8 addrspace(1)*"
                if tools.llvm_major < 15
                else "ptr addrspace(1)"
            ),
        )
        # {IN}, {OUT}: base addresses; {IN2}, {OUT1}: word offsets.
        text = re.sub(
            r"\{(IN|OUT)(\d*)\}",
            lambda m: str(
                int(defines[m.group(1)], 16) + 4 * int(m.group(2) or 0)
            ),
            text,
        )
        with open(os.path.join(hw, "kernel.ll"), "w") as f:
            f.write(text)
