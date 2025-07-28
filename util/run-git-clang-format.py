#!/usr/bin/env python3
#
# Copyright (c) 2025 Arm Limited
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

"""run-git-clang-format.py

Wrapper script to allow `pre-commit` to call `git-clang-format` on a
git commit.

By using `git-clang-format`, `clang-format` will only be run on
changed lines in staged files. This is intended to be an optional step
to ease the code review process for gem5 developers, and may
optionally be disabled.

This wrapper script adds the following functionality over calling
`git-clang-format` directly:

- Allows the user to control which, if any, files are re-formatted
  using environment variables. See the `argparse` description string
  for details.

- Designed to be minimally invasive when run in `--pre-commit` mode.
  It aims to catch all possible errors (including missing
  `clang-format` tools) and fail silently on tool errors to avoid
  blocking a commit.

The script can also be used without the `--pre-commit` option on the
command line to manually re-format staged files.

To support in-place checking of a PR in GitHub Actions, the
`--ci-pr-base-commit` option and `GEM5_CI_PR_BASE_COMMIT` environment
variable are also supported. If either of these are set to the
hash/branch-name/tag-name of the base commit of a PR, then
`git-clang-format` will be run in-place on the PR, and the changed
lines between `GEM5_CI_PR_BASE_COMMIT` and HEAD will be re-formatted.

"""

import argparse
import functools
import logging
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import (
    Iterable,
    Optional,
)


def run(
    *command: str,
    cwd: Optional[Path] = None,
    check: bool = True,
) -> Optional[subprocess.CompletedProcess]:
    """Safe call to run an external process.

    Checks possible exceptions, logs any error messages, and returns
    either the subprocess result, or None.

    If `check==True` non-zero exit codes are treated as errors,
    otherwise the CompletedProcess object is returned for the caller
    to handle.

    """
    if cwd is None:
        cwd = Path.cwd()

    logger = logging.getLogger(__name__)

    try:
        if not command:
            raise Exception("No command specified.")

        logger.debug("Running command: '%s'", shlex.join(command))

        res = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=check,
            cwd=cwd,
        )

        logger.debug(
            "    Command '%s' returned %d", shlex.join(command), res.returncode
        )
        return res
    except subprocess.CalledProcessError as ex:
        logger.error(
            "Command '%s' failed with retcode %d.", ex.cmd, ex.returncode
        )
        logger.debug(
            "Command '%s' stdout:\n%s", shlex.join(command), ex.output
        )
        logger.debug(
            "Command '%s' stderr:\n%s", shlex.join(command), ex.stderr
        )
    except FileNotFoundError as ex:
        logger.error("Command '%s' not found", command[0])
    except PermissionError as ex:
        logger.error(
            "Insufficient permissions to run command '%s'",
            shlex.join(command),
        )
    except Exception as ex:
        logger.error(
            "Error running command '%s': %s", shlex.join(command), str(ex)
        )

    return None


def run_silent_exit(
    *command: str, silent_exit: bool, cwd: Optional[Path] = None
) -> Optional[subprocess.CompletedProcess]:
    """Run an external process and exit on error if `silent_exit==True`.

    If called with `silent_exit=True`, calls sys.exit(0) on failure.
    This is used for `pre-commit` integration, where only
    re-formatting should block the commit.

    If called with `silent_exit=False`, either returns the subprocess
    result, or None if something went wrong.

    """

    res = run(*command, cwd=cwd, check=True)
    if res is None and silent_exit:
        sys.exit(0)
    else:
        return res


def make_ignore_path_rel(path_str: str, repo_root: Path) -> Optional[Path]:
    """Resolve a path relative to repo_root, and check whether it exists.

    Logs any errors appropriately and returns the relative path if
    possible, or `None` if the relative path could not be constructed.

    """

    logger = logging.getLogger(__name__)

    if not path_str:
        return None
    try:
        path = Path(path_str)
        path = (
            path.resolve()
            if path.is_absolute()
            else (repo_root / path).resolve()
        )
        if not path.exists():
            logger.warning("Path '%s' does not exist. Ignoring.", path)
            return None
        path = path.relative_to(repo_root)
        return path
    except ValueError:
        logger.warning(
            "Path '%s' not found relative to root directory '%s'. Ignoring.",
            path_str,
            repo_root,
        )
    return None


if __name__ == "__main__":
    DISABLE_ENV_VAR_NAME: str = "GEM5_NO_GIT_CLANG_FORMAT"
    IGNORE_ENV_VAR_NAME: str = "GEM5_GIT_CLANG_FORMAT_IGNORE"
    CI_PR_BASE_COMMIT_ENV_VAR_NAME: str = "GEM5_CI_PR_BASE_COMMIT"

    PROGRAM_DESCRIPTION_LINES: list[str] = [
        "Run `git-clang-format` on the current staged changes.",
        "",
        "Changes in individual files can be ignored using the `--ignore` "
        "option or by setting the environment variable "
        f"`{IGNORE_ENV_VAR_NAME}` to a colon-separated list of "
        f"files. Set environment variable `{DISABLE_ENV_VAR_NAME}` "
        "to any value to disable `git-clang-format` globally."
        "",
        "When running from `pre-commit`, specify the `--pre-commit` ",
        "option to suppress output and return appropriate exit codes.",
        "",
        "When running in a CI context, specify the `--ci-pr-base-commit` ",
        f"option or set the `{CI_PR_BASE_COMMIT_ENV_VAR_NAME}` environment ",
        "variable to the hash/branch-name/tag-name of the base commit of ",
        "the PR. This will run the formatting checks in-place from the ",
        "specified branch to HEAD.",
    ]
    PROGRAM_DESCRIPTION: str = "\n".join(PROGRAM_DESCRIPTION_LINES)

    parser = argparse.ArgumentParser(
        description=PROGRAM_DESCRIPTION,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--pre-commit",
        action="store_true",
        help="Indicates the script is being run from pre-commit.",
    )
    parser.add_argument(
        "--ignore",
        "-i",
        action="append",
        default=list(),
        help="Ignore this file.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the diff to stdout but do not modify any files.",
    )
    parser.add_argument(
        "--ci-pr-base-commit",
        type=str,
        default=None,
        help="Run the git-clang-format checks in a CI context, from this commit to HEAD.",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Output verbose debugging messages to stderr.",
    )
    args = parser.parse_args()

    # Set up logging
    #
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)7s: %(message)s",
        stream=sys.stderr,
    )
    logger = logging.getLogger(__name__)
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    elif args.pre_commit:
        logger.setLevel(logging.CRITICAL + 1)

    # Get the repository root directory
    #
    res = run_silent_exit(
        "git", "rev-parse", "--show-toplevel", silent_exit=args.pre_commit
    )
    if res is None:
        logger.error("Not a git repository, or git tools not available.")
        sys.exit(2)
    repo_root = Path(res.stdout.strip()).resolve()
    logger.debug("Repository root: '%s'", repo_root)
    make_ignore_path = functools.partial(
        make_ignore_path_rel, repo_root=repo_root
    )

    # Generate a list of ignored files, and convert to git pathspecs.
    #
    # The list combines ignored files specified on the command line
    # and via the environment variable. Missing files are removed.
    # Finally, `git` is queried to get a list of staged files, and
    # ignored files are removed to generate a git pathspec for the
    # operation. This is necessary because `git-clang-format` does not
    # support an ignored file list directly.
    #
    # Also check the environment variable for a global disable.
    #
    if DISABLE_ENV_VAR_NAME in os.environ:
        logger.debug(
            "git-clang-format disabled globally from environment variable."
        )
        sys.exit(0)

    ci_pr_base_commit: Optional[str] = os.environ.get(
        CI_PR_BASE_COMMIT_ENV_VAR_NAME, None
    )
    if ci_pr_base_commit is None:
        ci_pr_base_commit = args.ci_pr_base_commit
    if ci_pr_base_commit is not None:
        logger.debug(
            "Running in CI mode with base commit id '%s'", ci_pr_base_commit
        )
    else:
        logger.debug("Running in normal (staged-changes) mode")

    disabled_files: set[Optional[Path]] = {
        make_ignore_path(path) for path in args.ignore
    }
    disabled_files.update(
        {
            make_ignore_path(path)
            for path in os.environ.get(IGNORE_ENV_VAR_NAME, "").split(":")
        }
    )
    disabled_files.discard(None)
    if disabled_files:
        logger.debug(
            "Disabled files (missing removed):\n%s",
            "\n".join(f"'{path}'" for path in disabled_files),
        )
    else:
        logger.debug("No disabled files.")

    pathspecs: list[str] = []
    if disabled_files:
        res = run_silent_exit(
            "git",
            "diff",
            "--staged",
            "--name-only",
            silent_exit=args.pre_commit,
            cwd=repo_root,
        )
        if res is not None and res.returncode == 0:
            changed_files: set[Optional[Path]] = {
                make_ignore_path(path) for path in res.stdout.split("\n")
            }
            changed_files.discard(None)
            if changed_files:
                logger.debug(
                    "Found list of changed files:\n%s",
                    "\n".join(f"'{path}'" for path in changed_files),
                )
            else:
                logger.debug("No changed files.")
            changed_files = changed_files.difference(disabled_files)
            pathspecs = [str(path) for path in changed_files]
            if pathspecs:
                logger.debug(
                    "Generated pathspec:\n%s",
                    "\n".join(f"'{path}'" for path in pathspecs),
                )
            else:
                logger.debug("Pathspecs empty.")

    if disabled_files and not pathspecs:
        logger.warning("All changed files have been ignored. Doing nothing.")
        sys.exit(0)

    # Run `git-clang-format`
    #
    if ci_pr_base_commit is not None:
        # Run git-clang-format in 'CI' context
        command = [
            "git-clang-format",
            "--quiet",
            "--style=file",
            f"--commit={ci_pr_base_commit}",
        ]
    else:
        # Run git-clang-format in normal (staged-changes) context
        command = [
            "git-clang-format",
            "--staged",
            "--quiet",
            "--style=file",
        ]
    if args.dry_run:
        command.append("--diff")
    if pathspecs:
        command.extend(["--"] + pathspecs)

    res = run(
        *command,
        cwd=repo_root,
        check=False,
    )

    if res is not None:
        if args.dry_run and res.returncode == 1:
            print(res.stdout.strip())
        if (not args.pre_commit) and res.returncode != 0:
            print(res.stderr.strip())
        sys.exit(res.returncode)
