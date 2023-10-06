#!/usr/bin/env python
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
import datetime
import subprocess
import sys

import git_filter_repo
import update_copyright

parser = argparse.ArgumentParser(
    description="""Update copyright headers on files of a range of commits.

This can be used to easily update copyright headers at once on an entire
patchset before submitting.

Only files touched by the selected commits are updated.

Only existing copyrights for the selected holder are updated, new
notices are never automatically added if not already present.

The size of the changes is not taken into account, every touched file gets
updated. If you want to undo that for a certain file because the change to
it is trivial, you need to manually rebase and undo the copyright change
for that file.

Example usage with an organization alias such as `arm`:

```
python3 -m pip install --user --requirement \
  gem5/util/update_copyright/requirements.txt
./update-copyright.py -o arm HEAD~3
```

The above would act on the 3 last commits (HEAD~2, HEAD~ and HEAD),
leaving HEAD~3 unchanged, and doing updates such as:

```
- * Copyright (c) 2010, 2012-2013, 2015,2017-2019 ARM Limited
+ * Copyright (c) 2010, 2012-2013, 2015,2017-2020 ARM Limited
```

If the organization is not in the alias list, you can also explicitly give
the organization string as in:

```
./update-copyright.py HEAD~3 'ARM Limited'
```

which is equivalent to the previous invocation.
""",
    formatter_class=argparse.RawTextHelpFormatter,
)
parser.add_argument(
    "start",
    nargs="?",
    help="The commit before the last commit to be modified",
)
parser.add_argument("org-string", nargs="?", help="Copyright holder name")
parser.add_argument(
    "-o",
    "--org",
    choices=("arm",),
    help="Alias for known organizations",
)
args = parser.parse_args()


def error(msg):
    print("error: " + msg, file=sys.stderr)
    sys.exit(1)


# The existing safety checks are too strict, so we just disable them
# with force, and do our own checks to not overwrite uncommited changes
# checks.
# https://github.com/newren/git-filter-repo/issues/159
if subprocess.call(["git", "diff", "--staged", "--quiet"]):
    error("uncommitted changes")
if subprocess.call(["git", "diff", "--quiet"]):
    error("unstaged changes")

# Handle CLI arguments.
if args.start is None:
    error("the start argument must be given")
if args.org is None and getattr(args, "org-string") is None:
    error("either --org or org-string must be given")
if args.org is not None and getattr(args, "org-string") is not None:
    error("both --org and org-string given")
if args.org is not None:
    org_bytes = update_copyright.org_alias_map[args.org]
else:
    org_bytes = getattr(args, "org-string").encode()

# Call git_filter_repo.
# Args deduced from:
# print(git_filter_repo.FilteringOptions.parse_args(['--refs', 'HEAD',
# '--force'], error_on_empty=False))
filter_repo_args = git_filter_repo.FilteringOptions.default_options()
filter_repo_args.force = True
filter_repo_args.partial = True
filter_repo_args.refs = [f"{args.start}..HEAD"]
filter_repo_args.repack = False
filter_repo_args.replace_refs = "update-no-add"


def blob_callback(blob, callback_metadata, org_bytes):
    blob.data = update_copyright.update_copyright(
        blob.data,
        datetime.datetime.now().year,
        org_bytes,
    )


git_filter_repo.RepoFilter(
    filter_repo_args,
    blob_callback=lambda x, y: blob_callback(x, y, org_bytes),
).run()
