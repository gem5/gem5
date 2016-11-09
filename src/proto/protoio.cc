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

#include "proto/protoio.hh"

#include "base/misc.hh"

using namespace std;
using namespace google::protobuf;

ProtoOutputStream::ProtoOutputStream(const string& filename) :
    fileStream(filename.c_str(), ios::out | ios::binary | ios::trunc),
    wrappedFileStream(NULL), gzipStream(NULL), zeroCopyStream(NULL)
{
    if (!fileStream.good())
        panic("Could not open %s for writing\n", filename);

    // Wrap the output file in a zero copy stream, that in turn is
    // wrapped in a gzip stream if the filename ends with .gz. The
    // latter stream is in turn wrapped in a coded stream
    wrappedFileStream = new io::OstreamOutputStream(&fileStream);
    if (filename.find_last_of('.') != string::npos &&
        filename.substr(filename.find_last_of('.') + 1) == "gz") {
        gzipStream = new io::GzipOutputStream(wrappedFileStream);
        zeroCopyStream = gzipStream;
    } else {
        zeroCopyStream = wrappedFileStream;
    }

    // Write the magic number to the file
    io::CodedOutputStream codedStream(zeroCopyStream);
    codedStream.WriteLittleEndian32(magicNumber);

    // Note that each type of stream (packet, instruction etc) should
    // add its own header and perform the appropriate checks
}

ProtoOutputStream::~ProtoOutputStream()
{
    // As the compression is optional, see if the stream exists
    if (gzipStream != NULL)
        delete gzipStream;
    delete wrappedFileStream;
    fileStream.close();
}

void
ProtoOutputStream::write(const Message& msg)
{
    // Due to the byte limit of the coded stream we create it for
    // every single mesage (based on forum discussions around the size
    // limitation)
    io::CodedOutputStream codedStream(zeroCopyStream);

    // Write the size of the message to the stream
    codedStream.WriteVarint32(msg.ByteSize());

    // Write the message itself to the stream
    msg.SerializeWithCachedSizes(&codedStream);
}

ProtoInputStream::ProtoInputStream(const string& filename) :
    fileStream(filename.c_str(), ios::in | ios::binary), fileName(filename),
    useGzip(false),
    wrappedFileStream(NULL), gzipStream(NULL), zeroCopyStream(NULL)
{
    if (!fileStream.good())
        panic("Could not open %s for reading\n", filename);

    // check the magic number to see if this is a gzip stream
    unsigned char bytes[2];
    fileStream.read((char*) bytes, 2);
    useGzip = fileStream.good() && bytes[0] == 0x1f && bytes[1] == 0x8b;

    // seek to the start of the input file and clear any flags
    fileStream.clear();
    fileStream.seekg(0, ifstream::beg);

    createStreams();
}

void
ProtoInputStream::createStreams()
{
    // All streams should be NULL at this point
    assert(wrappedFileStream == NULL && gzipStream == NULL &&
           zeroCopyStream == NULL);

    // Wrap the input file in a zero copy stream, that in turn is
    // wrapped in a gzip stream if the filename ends with .gz. The
    // latter stream is in turn wrapped in a coded stream
    wrappedFileStream = new io::IstreamInputStream(&fileStream);
    if (useGzip) {
        gzipStream = new io::GzipInputStream(wrappedFileStream);
        zeroCopyStream = gzipStream;
    } else {
        zeroCopyStream = wrappedFileStream;
    }

    uint32_t magic_check;
    io::CodedInputStream codedStream(zeroCopyStream);
    if (!codedStream.ReadLittleEndian32(&magic_check) ||
        magic_check != magicNumber)
        panic("Input file %s is not a valid gem5 proto format.\n",
              fileName);
}

void
ProtoInputStream::destroyStreams()
{
    // As the compression is optional, see if the stream exists
    if (gzipStream != NULL) {
        delete gzipStream;
        gzipStream = NULL;
    }
    delete wrappedFileStream;
    wrappedFileStream = NULL;

    zeroCopyStream = NULL;
}


ProtoInputStream::~ProtoInputStream()
{
    destroyStreams();
    fileStream.close();
}


void
ProtoInputStream::reset()
{
    destroyStreams();
    // seek to the start of the input file and clear any flags
    fileStream.clear();
    fileStream.seekg(0, ifstream::beg);
    createStreams();
}

bool
ProtoInputStream::read(Message& msg)
{
    // Read a message from the stream by getting the size, using it as
    // a limit when parsing the message, then popping the limit again
    uint32_t size;

    // Due to the byte limit of the coded stream we create it for
    // every single mesage (based on forum discussions around the size
    // limitation)
    io::CodedInputStream codedStream(zeroCopyStream);
    if (codedStream.ReadVarint32(&size)) {
        io::CodedInputStream::Limit limit = codedStream.PushLimit(size);
        if (msg.ParseFromCodedStream(&codedStream)) {
            codedStream.PopLimit(limit);
            // All went well, the message is parsed and the limit is
            // popped again
            return true;
        } else {
            panic("Unable to read message from coded stream %s\n",
                  fileName);
        }
    }

    return false;
}
