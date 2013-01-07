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


/**
 * @file
 * Declaration of a wrapper for protobuf output streams.
 */

#ifndef __PROTO_PROTOIO_HH__
#define __PROTO_PROTOIO_HH__

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>

#include <fstream>

/**
 * A ProtoOutputStream wraps a coded stream, potentially with
 * compression, based on looking at the file name. Writing to the
 * stream is done to enable interaction with the file on a per-message
 * basis to avoid having to deal with huge data structures. The latter
 * is made possible by encoding the length of each message in the
 * stream.
 */
class ProtoOutputStream
{

  public:

    /**
     * Create an output stream for a given file name. If the filename
     * ends with .gz then the file will be compressed accordinly.
     *
     * @param filename Path to the file to create or truncate
     */
    ProtoOutputStream(const std::string& filename);

    /**
     * Destruct the output stream, and also flush and close the
     * underlying file streams and coded streams.
     */
    ~ProtoOutputStream();

    /**
     * Write a message to the stream, preprending it with the message
     * size.
     *
     * @param msg Message to write to the stream
     */
    void write(const google::protobuf::Message& msg);

  private:

    /// Underlying file output stream
    std::ofstream fileStream;

    /// Zero Copy stream wrapping the STL output stream
    google::protobuf::io::OstreamOutputStream* zeroCopyStream;

    /// Optional Gzip stream to wrap the Zero Copy stream
    google::protobuf::io::GzipOutputStream* gzipStream;

    /// Top-level coded stream that messages are written to
    google::protobuf::io::CodedOutputStream* codedStream;

};

#endif //__PROTO_PROTOIO_HH
