/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Hansson
 */

#include "base/misc.hh"
#include "proto/protoio.hh"

using namespace std;
using namespace google::protobuf;

ProtoOutputStream::ProtoOutputStream(const string& filename) :
    fileStream(filename.c_str(), ios::out | ios::binary | ios::trunc),
    zeroCopyStream(NULL), gzipStream(NULL), codedStream(NULL)
{
    if (!fileStream.good())
        panic("Could not open %s for writing\n", filename);

    // Wrap the output file in a zero copy stream, that in turn is
    // wrapped in a gzip stream if the filename ends with .gz. The
    // latter stream is in turn wrapped in a coded stream
    zeroCopyStream = new io::OstreamOutputStream(&fileStream);
    if (filename.find_last_of('.') != string::npos &&
        filename.substr(filename.find_last_of('.') + 1) == "gz") {
        gzipStream = new io::GzipOutputStream(zeroCopyStream);
        codedStream = new io::CodedOutputStream(gzipStream);
    } else {
        codedStream = new io::CodedOutputStream(zeroCopyStream);
    }

    // Use the ASCII characters gem5 as our magic number and write it
    // to the file
    const uint32_t magic_number = 0x356d6567;
    codedStream->WriteLittleEndian32(magic_number);

    // Note that each type of stream (packet, instruction etc) should
    // add its own header and perform the appropriate checks
}

ProtoOutputStream::~ProtoOutputStream()
{
    delete codedStream;
    // As the compression is optional, see if the stream exists
    if (gzipStream != NULL)
        delete gzipStream;
    delete zeroCopyStream;
    fileStream.close();
}

void
ProtoOutputStream::write(const Message& msg)
{
    // Write the size of the message to the stream
    codedStream->WriteVarint32(msg.ByteSize());

    // Write the message itself to the stream
    if (!msg.SerializeToCodedStream(codedStream))
        panic("Unable to write message to coded stream\n");
}
