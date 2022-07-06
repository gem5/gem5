/*
 * Copyright 2022 Google, Inc.
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
 */

#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include <unistd.h>

#include "shared_memory_client.hh"

/**
 * Consider that
 * 1. you have a SimpleMemory that spans from 0x0 to 0x10000
 * 2. you have a SharedMemoryServer with server_path=ram.sock
 * 3. The SHM server and the Mem is under the same System
 *
 * You should see a unix socket m5out/ram.sock, and you should be able to run
 * our example with commands like:
 * `./shared_memory_client_example m5out/ram.sock 0x1000 0x1010`
 *
 * The example will use the client to map the range 0x1000-0x1010 of the
 * SimpleMemory into the address space of the example. As a result, the example
 * will be able to access the backing store of the SimpleMemory with just a
 * normal pointer.
 */

static void PrintAsHexString(char *buffer, uint64_t size);

int
main(int argc, char *argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <shm_sock_path> <start> <end>"
                  << std::endl;
        return 1;
    }

    // Each SharedMemoryServer in gem5 will create a unix socket at the
    // location specified in its server_path parameter, and the socket can be
    // used by the client to communicate with the server.
    gem5::util::memory::SharedMemoryClient shm_client(argv[1]);

    // Before request access to the simulated physical memory in gem5, we need
    // to first determine what's the range we'd like to access.
    uint64_t start = std::stoull(argv[2], nullptr, 0);
    uint64_t end = std::stoull(argv[3], nullptr, 0);

    // One thing important is that, non-align request is not supported now, so
    // we'll need to ensure the start address is on page boundary.
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size < 0) {
        std::cerr << "Cannot determine page size" << std::endl;
        return 1;
    }
    if (start % page_size != 0) {
        std::cerr << "Start address must be aligned" << std::endl;
        return 1;
    }

    // The Map request, if success, will return a void* pointer, which can then
    // be used as a backdoor to the physical memory range in gem5. If there's
    // any error, nullptr will be returned.
    char *mem = reinterpret_cast<char *>(shm_client.MapMemory(start, end));
    uint64_t size = end - start + 1;
    if (mem == nullptr) {
        std::cerr << "Unable to map memory" << std::endl;
        return 1;
    }

    // A simple use case that print and randomly fill the memory.
    std::cout << "Content was: ";
    PrintAsHexString(mem, size);
    // Override content with random value.
    std::random_device rand_dev;
    std::uniform_int_distribution<int> rand_dist(0, 255);
    for (uint64_t i = 0; i < size; ++i) {
        mem[i] = rand_dist(rand_dev);
    }
    std::cout << "Content is: ";
    PrintAsHexString(mem, size);
}

static void
PrintAsHexString(char *buffer, uint64_t size)
{
    for (uint64_t i = 0; i < size; ++i) {
        std::cout << std::setw(2) << std::setfill('0') << std::hex
                  << (static_cast<int>(buffer[i]) & 0xff);
    }
    std::cout << std::endl;
}
