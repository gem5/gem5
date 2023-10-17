#!/usr/bin/env python3
#
# Copyright 2018 Google, Inc.
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

import argparse
import collections
import difflib
import functools
import inspect
import itertools
import json
import multiprocessing.pool
import os
import re
import subprocess
import sys

script_path = os.path.abspath(inspect.getfile(inspect.currentframe()))
script_dir = os.path.dirname(script_path)
config_path = os.path.join(script_dir, "config.py")
# Parent directories if checked out as part of gem5.
systemc_dir = os.path.dirname(script_dir)
src_dir = os.path.dirname(systemc_dir)
checkout_dir = os.path.dirname(src_dir)

systemc_rel_path = "systemc"
tests_rel_path = os.path.join(systemc_rel_path, "tests")
json_rel_path = os.path.join(tests_rel_path, "tests.json")


def scons(*args):
    args = ["scons", "--with-systemc-tests"] + list(args)
    subprocess.check_call(args)


class Test:
    def __init__(self, target, suffix, build_dir, props):
        self.target = target
        self.suffix = suffix
        self.build_dir = build_dir
        self.props = {}

        for key, val in props.items():
            self.set_prop(key, val)

    def set_prop(self, key, val):
        setattr(self, key, val)
        self.props[key] = val

    def dir(self):
        return os.path.join(self.build_dir, tests_rel_path, self.path)

    def src_dir(self):
        return os.path.join(script_dir, self.path)

    def expected_returncode_file(self):
        return os.path.join(self.src_dir(), "expected_returncode")

    def golden_dir(self):
        return os.path.join(self.src_dir(), "golden")

    def bin(self):
        return ".".join([self.name, self.suffix])

    def full_path(self):
        return os.path.join(self.dir(), self.bin())

    def m5out_dir(self):
        return os.path.join(self.dir(), "m5out." + self.suffix)

    def returncode_file(self):
        return os.path.join(self.m5out_dir(), "returncode")


test_phase_classes = {}


class TestPhaseMeta(type):
    def __init__(cls, name, bases, d):
        if not d.pop("abstract", False):
            test_phase_classes[d["name"]] = cls

        super().__init__(name, bases, d)


class TestPhaseBase(metaclass=TestPhaseMeta):
    abstract = True

    def __init__(self, main_args, *args):
        self.main_args = main_args
        self.args = args

    def __lt__(self, other):
        return self.number < other.number


class CompilePhase(TestPhaseBase):
    name = "compile"
    number = 1

    def run(self, tests):
        targets = list([test.full_path() for test in tests])

        parser = argparse.ArgumentParser()
        parser.add_argument("-j", type=int, default=0)
        args, leftovers = parser.parse_known_args(self.args)
        if args.j == 0:
            self.args = ("-j", str(self.main_args.j)) + self.args

        scons_args = (
            ["--directory", self.main_args.scons_dir, "USE_SYSTEMC=1"]
            + list(self.args)
            + targets
        )
        scons(*scons_args)


class RunPhase(TestPhaseBase):
    name = "execute"
    number = 2

    def run(self, tests):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--timeout",
            type=int,
            metavar="SECONDS",
            help="Time limit for each run in seconds, 0 to disable.",
            default=60,
        )
        parser.add_argument(
            "-j",
            type=int,
            default=0,
            help="How many tests to run in parallel.",
        )
        args = parser.parse_args(self.args)

        timeout_cmd = [
            "timeout",
            "--kill-after",
            str(args.timeout * 2),
            str(args.timeout),
        ]

        def run_test(test):
            cmd = []
            if args.timeout:
                cmd.extend(timeout_cmd)
            cmd.extend(
                [
                    os.path.abspath(test.full_path()),
                    "-rd",
                    os.path.abspath(test.m5out_dir()),
                    "--listener-mode=off",
                    "--quiet",
                    os.path.abspath(config_path),
                ]
            )
            # Ensure the output directory exists.
            if not os.path.exists(test.m5out_dir()):
                os.makedirs(test.m5out_dir())
            try:
                subprocess.check_call(cmd, cwd=os.path.dirname(test.dir()))
            except subprocess.CalledProcessError as error:
                returncode = error.returncode
            else:
                returncode = 0
            with open(test.returncode_file(), "w") as rc:
                rc.write("%d\n" % returncode)

        j = self.main_args.j if args.j == 0 else args.j

        runnable = filter(lambda t: not t.compile_only, tests)
        if j == 1:
            list(map(run_test, runnable))
        else:
            tp = multiprocessing.pool.ThreadPool(j)
            list(map(lambda t: tp.apply_async(run_test, (t,)), runnable))
            tp.close()
            tp.join()


class Checker:
    def __init__(self, ref, test, tag):
        self.ref = ref
        self.test = test
        self.tag = tag

    def check(self):
        with open(self.test) as test_f, open(self.ref) as ref_f:
            return test_f.read() == ref_f.read()


def tagged_filt(tag, num):
    return (
        r"\n{}: \({}{}\) .*\n(In file: .*\n)?" r"(In process: [\w.]* @ .*\n)?"
    ).format(tag, tag[0], num)


def error_filt(num):
    return tagged_filt("Error", num)


def warning_filt(num):
    return tagged_filt("Warning", num)


def info_filt(num):
    return tagged_filt("Info", num)


class DiffingChecker(Checker):
    def __init__(self, ref, test, tag, out_dir):
        super().__init__(ref, test, tag)
        self.out_dir = out_dir

    def is_bytes_mode(self):
        return False

    def do_diff(self, ref_lines, test_lines, ref_file, test_file):
        return difflib.unified_diff(
            ref_lines, test_lines, fromfile=ref_file, tofile=test_file
        )

    def diffing_check(self, ref_lines, test_lines):
        test_file = os.path.basename(self.test)
        ref_file = os.path.basename(self.ref)

        diff_file = ".".join([ref_file, "diff"])
        diff_path = os.path.join(self.out_dir, diff_file)
        if test_lines != ref_lines:
            flag = "wb" if self.is_bytes_mode() else "w"
            with open(diff_file, flag) as diff_f:
                for line in self.do_diff(
                    ref_lines, test_lines, ref_file, test_file
                ):
                    diff_f.write(line)
            return False
        else:
            if os.path.exists(diff_path):
                os.unlink(diff_path)
            return True


class LogChecker(DiffingChecker):
    def merge_filts(*filts):
        filts = map(lambda f: "(" + f + ")", filts)
        filts = "|".join(filts)
        return re.compile(filts.encode("utf-8"), flags=re.MULTILINE)

    def is_bytes_mode(self):
        return True

    def do_diff(self, ref_lines, test_lines, ref_file, test_file):
        return difflib.diff_bytes(
            difflib.unified_diff,
            ref_lines,
            test_lines,
            fromfile=ref_file.encode("utf-8"),
            tofile=test_file.encode("utf-8"),
        )

    # The reporting mechanism will print the actual filename when running in
    # gem5, and the "golden" output will say "<removed by verify.py>". We want
    # to strip out both versions to make comparing the output sensible.
    in_file_filt = r"^In file: ((<removed by verify\.pl>)|([a-zA-Z0-9.:_/]*))$"

    ref_filt = merge_filts(
        r"^\nInfo: /OSCI/SystemC: Simulation stopped by user.\n",
        r"^SystemC Simulation\n",
        r"^\nInfo: \(I804\) /IEEE_Std_1666/deprecated: "
        + r"You can turn off(.*\n){7}",
        r"^\nInfo: \(I804\) /IEEE_Std_1666/deprecated: \n"
        + r"    sc_clock\(const char(.*\n){3}",
        warning_filt(540),
        warning_filt(571),
        info_filt(804),
        info_filt(704),
        in_file_filt,
    )
    test_filt = merge_filts(
        r"^/.*:\d+: ",
        r"^Global frequency set at \d* ticks per second\n",
        r".*info: Entering event queue @ \d*\.  Starting simulation\.\.\.\n",
        r".*warn: Ignoring request to set stack size\.\n",
        r"^.*warn: No dot file generated. Please install pydot "
        + r"to generate the dot file and pdf.\n",
        info_filt(804),
        in_file_filt,
    )

    def apply_filters(self, data, filts):
        re.sub(filt, b"", data)

    def check(self):
        with open(self.test, "rb") as test_f, open(self.ref, "rb") as ref_f:
            test = re.sub(self.test_filt, b"", test_f.read())
            ref = re.sub(self.ref_filt, b"", ref_f.read())
            return self.diffing_check(
                ref.splitlines(True), test.splitlines(True)
            )


class VcdChecker(DiffingChecker):
    def check(self):
        with open(self.test) as test_f, open(self.ref) as ref_f:
            ref = ref_f.read().splitlines(True)
            test = test_f.read().splitlines(True)
            # Strip off the first seven lines of the test output which are
            # date and version information.
            test = test[7:]

            return self.diffing_check(ref, test)


class GoldenDir:
    def __init__(self, path, platform):
        self.path = path
        self.platform = platform

        contents = os.listdir(path)
        suffix = "." + platform
        suffixed = filter(lambda c: c.endswith(suffix), contents)
        bases = map(lambda t: t[: -len(platform)], suffixed)
        common = filter(lambda t: not t.startswith(tuple(bases)), contents)

        self.entries = {}

        class Entry:
            def __init__(self, e_path):
                self.used = False
                self.path = os.path.join(path, e_path)

            def use(self):
                self.used = True

        for entry in contents:
            self.entries[entry] = Entry(entry)

    def entry(self, name):
        def match(n):
            return (n == name) or n.startswith(name + ".")

        matches = {n: e for n, e in self.entries.items() if match(n)}

        for match in matches.values():
            match.use()

        platform_name = ".".join([name, self.platform])
        if platform_name in matches:
            return matches[platform_name].path
        if name in matches:
            return matches[name].path
        else:
            return None

    def unused(self):
        items = self.entries.items()
        items = list(filter(lambda i: not i[1].used, items))

        items.sort()
        sources = []
        i = 0
        while i < len(items):
            root = items[i][0]
            sources.append(root)
            i += 1
            while i < len(items) and items[i][0].startswith(root):
                i += 1
        return sources


class VerifyPhase(TestPhaseBase):
    name = "verify"
    number = 3

    def reset_status(self):
        self._passed = []
        self._failed = {}

    def passed(self, test):
        self._passed.append(test)

    def failed(self, test, cause, note=""):
        test.set_prop("note", note)
        self._failed.setdefault(cause, []).append(test)

    def print_status(self):
        total_passed = len(self._passed)
        total_failed = sum(map(len, self._failed.values()))
        print()
        print(f"Passed: {total_passed:4} - Failed: {total_failed:4}")

    def write_result_file(self, path):
        results = {
            "passed": list(map(lambda t: t.props, self._passed)),
            "failed": {
                cause: list(map(lambda t: t.props, tests))
                for cause, tests in self._failed.items()
            },
        }
        with open(path, "w") as rf:
            json.dump(results, rf)

    def print_results(self):
        print()
        print("Passed:")
        for path in sorted(list([t.path for t in self._passed])):
            print("    ", path)

        print()
        print("Failed:")

        causes = []
        for cause, tests in sorted(self._failed.items()):
            block = "  " + cause.capitalize() + ":\n"
            for test in sorted(tests, key=lambda t: t.path):
                block += "    " + test.path
                if test.note:
                    block += " - " + test.note
                block += "\n"
            causes.append(block)

        print("\n".join(causes))

    def run(self, tests):
        parser = argparse.ArgumentParser()
        result_opts = parser.add_mutually_exclusive_group()
        result_opts.add_argument(
            "--result-file",
            action="store_true",
            help="Create a results.json file in the current directory.",
        )
        result_opts.add_argument(
            "--result-file-at",
            metavar="PATH",
            help="Create a results json file at the given path.",
        )
        parser.add_argument(
            "--no-print-results",
            action="store_true",
            help="Don't print a list of tests that passed or failed",
        )
        args = parser.parse_args(self.args)

        self.reset_status()

        runnable = filter(lambda t: not t.compile_only, tests)
        compile_only = filter(lambda t: t.compile_only, tests)

        for test in compile_only:
            if os.path.exists(test.full_path()):
                self.passed(test)
            else:
                self.failed(test, "compile failed")

        for test in runnable:
            with open(test.returncode_file()) as rc:
                returncode = int(rc.read())

            expected_returncode = 0
            if os.path.exists(test.expected_returncode_file()):
                with open(test.expected_returncode_file()) as erc:
                    expected_returncode = int(erc.read())

            if returncode == 124:
                self.failed(test, "time out")
                continue
            elif returncode != expected_returncode:
                if expected_returncode == 0:
                    self.failed(test, "abort")
                else:
                    self.failed(test, "missed abort")
                continue

            out_dir = test.m5out_dir()

            Diff = collections.namedtuple("Diff", "ref, test, tag, ref_filter")

            diffs = []

            gd = GoldenDir(test.golden_dir(), "linux64")

            missing = []
            log_file = ".".join([test.name, "log"])
            log_path = gd.entry(log_file)
            simout_path = os.path.join(out_dir, "simout.txt")
            if not os.path.exists(simout_path):
                missing.append("log output")
            elif log_path:
                diffs.append(
                    LogChecker(log_path, simout_path, log_file, out_dir)
                )

            for name in gd.unused():
                test_path = os.path.join(out_dir, name)
                ref_path = gd.entry(name)
                if not os.path.exists(test_path):
                    missing.append(name)
                elif name.endswith(".vcd"):
                    diffs.append(
                        VcdChecker(ref_path, test_path, name, out_dir)
                    )
                else:
                    diffs.append(Checker(ref_path, test_path, name))

            if missing:
                self.failed(test, "missing output", " ".join(missing))
                continue

            failed_diffs = list(filter(lambda d: not d.check(), diffs))
            if failed_diffs:
                tags = map(lambda d: d.tag, failed_diffs)
                self.failed(test, "failed diffs", " ".join(tags))
                continue

            self.passed(test)

        if not args.no_print_results:
            self.print_results()

        self.print_status()

        result_path = None
        if args.result_file:
            result_path = os.path.join(os.getcwd(), "results.json")
        elif args.result_file_at:
            result_path = args.result_file_at

        if result_path:
            self.write_result_file(result_path)


parser = argparse.ArgumentParser(description="SystemC test utility")

parser.add_argument(
    "build_dir",
    metavar="BUILD_DIR",
    help="The build directory (ie. build/ARM).",
)

parser.add_argument(
    "--update-json",
    action="store_true",
    help="Update the json manifest of tests.",
)

parser.add_argument(
    "--flavor",
    choices=["debug", "opt", "fast"],
    default="opt",
    help="Flavor of binary to test.",
)

parser.add_argument(
    "--list", action="store_true", help="List the available tests"
)

parser.add_argument(
    "-j",
    type=int,
    default=1,
    help="Default level of parallelism, can be overriden "
    "for individual stages",
)

parser.add_argument(
    "-C",
    "--scons-dir",
    metavar="SCONS_DIR",
    default=checkout_dir,
    help="Directory to run scons from",
)

filter_opts = parser.add_mutually_exclusive_group()
filter_opts.add_argument(
    "--filter",
    default="True",
    help="Python expression which filters tests based on their properties",
)
filter_opts.add_argument(
    "--filter-file",
    default=None,
    type=argparse.FileType("r"),
    help="Same as --filter, but read from a file",
)


def collect_phases(args):
    phase_groups = [
        list(g)
        for k, g in itertools.groupby(args, lambda x: x != "--phase")
        if k
    ]
    main_args = parser.parse_args(phase_groups[0][1:])
    phases = []
    names = []
    for group in phase_groups[1:]:
        name = group[0]
        if name in names:
            raise RuntimeException(f"Phase {name} specified more than once")
        phase = test_phase_classes[name]
        phases.append(phase(main_args, *group[1:]))
    phases.sort()
    return main_args, phases


main_args, phases = collect_phases(sys.argv)

if len(phases) == 0:
    phases = [
        CompilePhase(main_args),
        RunPhase(main_args),
        VerifyPhase(main_args),
    ]


json_path = os.path.join(main_args.build_dir, json_rel_path)

if main_args.update_json:
    scons("--directory", main_args.scons_dir, os.path.join(json_path))

with open(json_path) as f:
    test_data = json.load(f)

    if main_args.filter_file:
        f = main_args.filter_file
        filt = compile(f.read(), f.name, "eval")
    else:
        filt = compile(main_args.filter, "<string>", "eval")

    filtered_tests = {
        target: props
        for (target, props) in test_data.items()
        if eval(filt, dict(props))
    }

    if len(filtered_tests) == 0:
        print("All tests were filtered out.")
        exit()

    if main_args.list:
        for target, props in sorted(filtered_tests.items()):
            print(f"{target}.{main_args.flavor}")
            for key, val in props.items():
                print(f"    {key}: {val}")
        print(f"Total tests: {len(filtered_tests)}")
    else:
        tests_to_run = list(
            [
                Test(target, main_args.flavor, main_args.build_dir, props)
                for target, props in sorted(filtered_tests.items())
            ]
        )

        for phase in phases:
            phase.run(tests_to_run)
