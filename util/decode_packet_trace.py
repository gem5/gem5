#!/usr/bin/env python3

# Copyright (c) 2013-2014 ARM Limited
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

# This script is used to dump protobuf packet traces to ASCII
# format.

import os
import subprocess
import sys

import protolib
from google.protobuf import json_format
from google.protobuf.descriptor import FieldDescriptor

util_dir = os.path.dirname(os.path.realpath(__file__))
# Make sure the proto definitions are up to date.
subprocess.check_call(["make", "--quiet", "-C", util_dir, "packet_pb2.py"])
import packet_pb2


def _get_field_value(packet, field):
    if field.is_repeated:
        values = getattr(packet, field.name)
        return list(values) if values else None

    if field.has_presence and not packet.HasField(field.name):
        return None

    return getattr(packet, field.name)


def _decode_packet(proto_in):
    size, pos = protolib._DecodeVarint32(proto_in)
    if pos == 0:
        return None
    if size <= 0:
        raise ValueError(f"Invalid packet size {size}")

    buf = proto_in.read(size)
    if len(buf) != size:
        raise ValueError(
            f"Unexpected EOF while reading packet payload (expected {size} bytes, "
            f"got {len(buf)})"
        )

    packet = packet_pb2.Packet()
    packet.ParseFromString(buf)
    return packet


def _write_header(ascii_out, header):
    try:
        header_dict = json_format.MessageToDict(
            header,
            preserving_proto_field_name=True,
            including_default_value_fields=True,
        )
    except TypeError:
        # Fallback for older protobuf versions that lack
        # including_default_value_fields.
        header_dict = json_format.MessageToDict(
            header, preserving_proto_field_name=True
        )
        for field in header.DESCRIPTOR.fields:
            if field.is_repeated:
                header_dict.setdefault(field.name, [])
            elif not header.HasField(field.name):
                header_dict[field.name] = field.default_value

    def _render(item, indent=0, list_item=False):
        space = "  " * indent
        bullet = "- " if list_item else ""
        if isinstance(item, dict):
            ascii_out.write(f"# {space}{bullet}{{\n")
            for k, v in item.items():
                if isinstance(v, (dict, list)):
                    ascii_out.write(f"# {space}  {k}: ")
                    _render(v, indent + 1)
                else:
                    ascii_out.write(f"# {space}  {k}: {v}\n")
            ascii_out.write(f"# {space}}}\n")
        elif isinstance(item, list):
            ascii_out.write(f"# {space}{bullet}[\n")
            for v in item:
                _render(v, indent + 1, list_item=True)
            ascii_out.write(f"# {space}]\n")
        else:
            ascii_out.write(f"# {space}{bullet}{item}\n")

    ascii_out.write("# header {\n")
    for k, v in header_dict.items():
        if isinstance(v, (dict, list)):
            ascii_out.write(f"#   {k}: ")
            _render(v, 2 if isinstance(v, dict) else 1)
        else:
            ascii_out.write(f"#   {k}: {v}\n")
    ascii_out.write("# }\n")


def _dump_packets(proto_in, ascii_out, packet_fields):
    num_packets = 0

    while True:
        try:
            packet = _decode_packet(proto_in)
        except Exception as exc:
            location = getattr(proto_in, "tell", lambda: -1)()
            print(
                f"Error decoding packet {num_packets} at offset {location}: {exc}",
                file=sys.stderr,
            )
            exit(-1)
        if packet is None:
            break
        num_packets += 1

        row = []
        for field in packet_fields:
            value = _get_field_value(packet, field)
            if value is None:
                row.append("NULL")
            elif field.is_repeated:
                row.append("|".join(str(entry) for entry in value))
            else:
                row.append(str(value))

        ascii_out.write(",".join(row) + "\n")

    return num_packets


def main():
    if len(sys.argv) != 3:
        print("Usage: ", sys.argv[0], " <protobuf input> <ASCII output>")
        exit(-1)

    # Open the file in read mode
    proto_in = protolib.openFileRd(sys.argv[1])

    try:
        ascii_out = open(sys.argv[2], "w")
    except OSError:
        print("Failed to open ", sys.argv[2], " for writing")
        exit(-1)

    # Read the magic number in 4-byte Little Endian
    magic_number = proto_in.read(4).decode()

    if magic_number != "gem5":
        print("Unrecognized file", sys.argv[1])
        exit(-1)

    print("Parsing packet header")

    # Add the packet header
    header = packet_pb2.PacketHeader()
    protolib.decodeMessage(proto_in, header)

    _write_header(ascii_out, header)

    print("Parsing packets")

    packet_fields = list(packet_pb2.Packet.DESCRIPTOR.fields)
    header_columns = []
    for field in packet_fields:
        header_columns.append(field.name)

    ascii_out.write("# " + ", ".join(header_columns) + "\n")

    num_packets = _dump_packets(proto_in, ascii_out, packet_fields)

    print("Parsed packets:", num_packets)

    # We're done
    ascii_out.close()
    proto_in.close()


if __name__ == "__main__":
    main()
