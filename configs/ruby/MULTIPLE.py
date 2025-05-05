# Copyright 2024 (c) Jason Lowe-Power
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

import importlib

from m5.defines import buildEnv


def define_options(parser):
    print("**************IMPORTANT*******************************")
    print("This gem5 binary is configured with multiple protocols.")
    print("To use this gem5 binary, you must use the standard library.")
    print("The Ruby.py file is deprecated.")
    print("You can use --protocol to specify the protocol you want to use.")
    print("IMPORTANT: THIS WILL BE REMOVED IN THE FUTURE.")

    available_protocols = [
        protocol[len("RUBY_PROTOCOL_") :]
        for protocol in buildEnv.keys()
        if protocol.startswith("RUBY_PROTOCOL_")
    ]

    parser.add_argument(
        "--protocol",
        type=str,
        help="Specify the protocol you want to use.",
        required=True,
        choices=available_protocols,
    )

    # Note: we can't rely on the options yet because they haven't been parsed.
    from sys import argv

    found = False
    for arg in argv:
        if arg.startswith("--protocol"):
            found = True
            if "=" in arg:
                protocol = arg[len("--protocol=") :]
            else:
                protocol = argv[argv.index(arg) + 1]
    if not found:
        print(
            "ERROR: You must specify a protocol with --protocol <protocol>. "
            "or --protocol=<protocol>"
        )
        exit(1)

    protocol_module = importlib.import_module(f".{protocol}", package="ruby")
    protocol_module.define_options(parser)
    buildEnv["PROTOCOL"] = protocol


# Note: This function isn't used because buildEnv["PROTOCOL"] is set
# in define_options. However, this is here just in case.
def create_system(options, *args, **kwargs):
    protocol = options.protocol
    print("Using protocol: ", protocol)
    print("WARNING: The Ruby.py file is deprecated.")

    protocol_module = importlib.import_module(f".{protocol}", package="ruby")
    (cpu_sequencers, dir_cntrls, topology) = protocol_module.create_system(
        options, *args, **kwargs
    )

    return (cpu_sequencers, dir_cntrls, topology)
