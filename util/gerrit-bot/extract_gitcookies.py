# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
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


def parse_gitcookies_line(raw):
    # if this is a line from .gitcookies, the delimiter is `\t`
    auth_info = raw.strip().split("\t")
    if len(auth_info) < 7:
        # if this is a line from auth script, the delimiter is `,`
        auth_info = raw.strip().split(",")
    if len(auth_info) != 7:
        return None, None
    auth_info = auth_info[-1]
    auth_info = auth_info[4:].split("=")
    username = auth_info[0]
    password = auth_info[1]
    return username, password


def parse_gitcookies(input_path):
    username_password_dict = {}
    with open(input_path) as input_stream:
        for line in input_stream:
            username, password = parse_gitcookies_line(line)
            if not username:
                continue
            username_password_dict[username] = password
    return username_password_dict


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Extract username and password from .gitcookies"
            "or from the script used to write .gitcookies file"
        ),
    )
    parser.add_argument(
        "input",
        help=("Path to a .gitcookies file or a file with a similar format"),
    )
    parser.add_argument("output", help="Path to the output file")
    args = parser.parse_args()
    username_password_dict = parse_gitcookies(args.input)
    with open(args.output, "w") as output_stream:
        for username, password in username_password_dict.items():
            output_stream.write(f"{username}\n{password}\n")
