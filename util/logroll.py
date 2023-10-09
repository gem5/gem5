#! /usr/bin/env python3
#
# Copyright 2021 Google Inc.
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
import copy
import signal
import sys
import unittest.mock

parser = argparse.ArgumentParser(
    description="""Circular buffer for text output.

    To capture the rolling last 25 lines of output from command "command":
    command | logroll.py -n 25

    While that's running, to see the most recent 25 lines of output without
    interrupting "command", send SIGUSR1 to the logroll.py process:
    kill -s USR1 ${PID of logroll.py}"""
)
parser.add_argument(
    "-n",
    "--lines",
    default=10,
    type=int,
    help="Maximum number of lines to buffer at a time.",
)
parser.add_argument(
    "file",
    nargs="?",
    default=sys.stdin,
    type=argparse.FileType("r", encoding="UTF-8"),
    help="File to read from, default is stdin",
)

args = parser.parse_args()


def dump_lines(lines, idx):
    for line in lines[idx:]:
        print(line, end="")
    for line in lines[:idx]:
        print(line, end="")


def dump_and_exit(lines, idx):
    dump_lines(lines, idx)
    sys.exit(0)


def main(target, incoming):
    idx = 0
    lines = []
    last_idx = target - 1

    signal.signal(signal.SIGUSR1, lambda num, frame: dump_lines(lines, idx))
    signal.signal(signal.SIGINT, lambda num, frame: dump_and_exit(lines, idx))

    for line in incoming:
        lines.append(line)
        if idx == last_idx:
            idx = 0
            break
        else:
            idx += 1

    for lines[idx] in incoming:
        idx = 0 if idx == last_idx else idx + 1

    dump_lines(lines, idx)


if __name__ == "__main__":
    main(target=args.lines, incoming=args.file)


# Unit tests #


class CopyingMock(unittest.mock.MagicMock):
    def __call__(self, *args, **kwargs):
        args = copy.deepcopy(args)
        kwargs = copy.deepcopy(kwargs)
        return super(CopyingMock, self).__call__(*args, **kwargs)


class TestLogroll(unittest.TestCase):
    # Test data.
    lines2 = ["First line", "Second line"]
    lines3 = ["First line", "Second line", "Third line"]
    lines8 = [
        "First line",
        "Second line",
        "Third line",
        "Fourth line",
        "Fifth line",
        "Sixth line",
        "Seventh line",
        "Eigth line",
    ]

    # Generator which returns lines like a file object would.
    def line_gen(self, lines):
        for line in lines:
            yield line

    # Generator like above, but which simulates a signal midway through.
    def signal_line_gen(self, lines, pos, sig_dict, signal):
        # Return the first few lines.
        for line in lines[:pos]:
            yield line

        # Simulate receiving the signal.
        self.assertIn(signal, sig_dict)
        if signal in sig_dict:
            # Pas in junk for the num and frame arguments.
            sig_dict[signal](None, None)

        # Return the remaining lines.
        for line in lines[pos:]:
            yield line

    # Set up a mock of signal.signal to record handlers in a dict.
    def mock_signal_dict(self, mock):
        signal_dict = {}

        def signal_signal(num, action):
            signal_dict[num] = action

        mock.side_effect = signal_signal
        return signal_dict

    # Actual test methods.
    def test_filling_dump_lines(self):
        with unittest.mock.patch("builtins.print") as mock_print:
            dump_lines(self.lines2, len(self.lines2))
            calls = list(
                [unittest.mock.call(line, end="") for line in self.lines2]
            )
            mock_print.assert_has_calls(calls)

    def test_full_dump_lines(self):
        with unittest.mock.patch("builtins.print") as mock_print:
            dump_lines(self.lines2, 0)
            calls = list(
                [unittest.mock.call(line, end="") for line in self.lines2]
            )
            mock_print.assert_has_calls(calls)

    def test_offset_dump_lines(self):
        with unittest.mock.patch("builtins.print") as mock_print:
            dump_lines(self.lines3, 1)
            calls = [
                unittest.mock.call(self.lines3[1], end=""),
                unittest.mock.call(self.lines3[2], end=""),
                unittest.mock.call(self.lines3[0], end=""),
            ]
            mock_print.assert_has_calls(calls)

    def test_dump_and_exit(self):
        with unittest.mock.patch(
            "sys.exit"
        ) as mock_sys_exit, unittest.mock.patch(
            __name__ + ".dump_lines", new_callable=CopyingMock
        ) as mock_dump_lines:
            idx = 1
            dump_and_exit(self.lines3, idx)
            mock_dump_lines.assert_called_with(self.lines3, idx)
            mock_sys_exit.assert_called_with(0)

    def test_filling_main(self):
        with unittest.mock.patch("builtins.print") as mock_print:
            main(5, self.line_gen(self.lines3))
            calls = list(
                [unittest.mock.call(line, end="") for line in self.lines3]
            )
            mock_print.assert_has_calls(calls)

    def test_full_main(self):
        with unittest.mock.patch("builtins.print") as mock_print:
            main(5, self.line_gen(self.lines8))
            calls = list(
                [unittest.mock.call(line, end="") for line in self.lines8[-5:]]
            )
            mock_print.assert_has_calls(calls)

    def test_sigusr1_filling_main(self):
        with unittest.mock.patch(
            "signal.signal"
        ) as mock_signal, unittest.mock.patch(
            __name__ + ".dump_lines", new_callable=CopyingMock
        ) as mock_dump_lines:

            signal_dict = self.mock_signal_dict(mock_signal)

            main(
                4,
                self.signal_line_gen(
                    self.lines8, 3, signal_dict, signal.SIGUSR1
                ),
            )

            mock_dump_lines.assert_has_calls(
                [
                    unittest.mock.call(self.lines8[0:3], 3 % 4),
                    unittest.mock.call(self.lines8[-4:], len(self.lines8) % 4),
                ]
            )

    def test_sigint_filling_main(self):
        with unittest.mock.patch(
            "signal.signal"
        ) as mock_signal, unittest.mock.patch(
            __name__ + ".dump_lines", new_callable=CopyingMock
        ) as mock_dump_lines:

            signal_dict = self.mock_signal_dict(mock_signal)

            with self.assertRaises(SystemExit):
                main(
                    4,
                    self.signal_line_gen(
                        self.lines8, 3, signal_dict, signal.SIGINT
                    ),
                )

            mock_dump_lines.assert_has_calls(
                [unittest.mock.call(self.lines8[0:3], 3 % 4)]
            )

    def test_sigusr1_full_main(self):
        with unittest.mock.patch(
            "signal.signal"
        ) as mock_signal, unittest.mock.patch(
            __name__ + ".dump_lines", new_callable=CopyingMock
        ) as mock_dump_lines:

            signal_dict = self.mock_signal_dict(mock_signal)

            main(
                4,
                self.signal_line_gen(
                    self.lines8, 5, signal_dict, signal.SIGUSR1
                ),
            )

            mock_dump_lines.assert_has_calls(
                [
                    unittest.mock.call(
                        self.lines8[4:5] + self.lines8[1:4], 5 % 4
                    ),
                    unittest.mock.call(self.lines8[-4:], len(self.lines8) % 4),
                ]
            )

    def test_sigint_full_main(self):
        with unittest.mock.patch(
            "signal.signal"
        ) as mock_signal, unittest.mock.patch(
            __name__ + ".dump_lines", new_callable=CopyingMock
        ) as mock_dump_lines:

            signal_dict = self.mock_signal_dict(mock_signal)

            with self.assertRaises(SystemExit):
                main(
                    4,
                    self.signal_line_gen(
                        self.lines8, 5, signal_dict, signal.SIGINT
                    ),
                )

            mock_dump_lines.assert_has_calls(
                [
                    unittest.mock.call(
                        self.lines8[4:5] + self.lines8[1:4], 5 % 4
                    )
                ]
            )
