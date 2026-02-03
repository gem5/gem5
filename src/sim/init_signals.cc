/*
 * Copyright (c) 2012, 2015 ARM Limited
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
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
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

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#if defined(__FreeBSD__)
#include <sys/param.h>

#endif

#include "sim/init_signals.hh"

#include "base/atomicio.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "debug/ExternalSignal.hh"
#include "sim/async.hh"
#include "sim/backtrace.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

// Use an separate stack for fatal signal handlers

static bool
setupAltStack()
{
    const auto stack_size = 2 * SIGSTKSZ;
    static uint8_t *fatal_sig_stack = new uint8_t[stack_size];
    stack_t stack;
#if defined(__FreeBSD__) && (__FreeBSD_version < 1100097)
    stack.ss_sp = (char *)fatal_sig_stack;
#else
    stack.ss_sp = fatal_sig_stack;
#endif
    stack.ss_size = stack_size;
    stack.ss_flags = 0;

    return sigaltstack(&stack, NULL) == 0;
}

static void
installSignalHandler(int signal, void (*handler)(int sigtype),
                     int flags = SA_RESTART,
                     struct sigaction *old_sa = NULL)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = handler;
    sa.sa_flags = flags;

    if (sigaction(signal, &sa, old_sa) == -1)
        panic("Failed to setup handler for signal %i\n", signal);
}

static void
raiseFatalSignal(int signo)
{
    // The signal handler should have been reset and unmasked (it was
    // registered with SA_RESETHAND | SA_NODEFER), just raise the
    // signal again to invoke the default handler.
    STATIC_ERR("For more info on how to address this issue, please visit "
        "https://www.gem5.org/documentation/general_docs/common-errors/ \n\n");
    pthread_kill(pthread_self(), signo);

    // Something is really wrong if the process is alive at this
    // point, manually try to exit it.
    STATIC_ERR("Failed to execute default signal handler!\n");
    _exit(127);
}

/// Stats signal handler.
void
dumpStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

void
dumprstStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
    async_statreset = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

/// Exit signal handler.
void
exitNowHandler(int sigtype)
{
    async_event = true;
    async_exit = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

/// Abort signal handler.
void
abortHandler(int sigtype)
{
    const EventQueue *const eq(curEventQueue());
    if (eq) {
        ccprintf(std::cerr, "Program aborted at tick %llu\n",
                eq->getCurTick());
    } else {
        STATIC_ERR("Program aborted\n\n");
    }

    print_backtrace();
    raiseFatalSignal(sigtype);
}

/// Segmentation fault signal handler.
static void
segvHandler(int sigtype)
{
    STATIC_ERR("gem5 has encountered a segmentation fault!\n\n");

    print_backtrace();
    raiseFatalSignal(SIGSEGV);
}

// Handle SIGIO
static void
ioHandler(int sigtype)
{
    async_event = true;
    async_io = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

/**
 * @brief Handles signals from external processes by processing JSON data
 * through shared memory
 *
 * This handler processes JSON messages with the following structure:
 * {
 *     "id": <numeric_id>,
 *     "payload": {
 *         "key1": "value1",
 *         "key2": "value2",
 *         ...
 *     }
 * }
 *
 * Requirements for valid JSON input:
 * - Must have an "id" field with a numeric value
 * - Must have a "payload" object containing key-value pairs
 * - All keys and values in the payload must be quoted strings
 * - Keys must be valid string identifiers
 * - Whitespace and newlines are allowed between elements
 *
 * Example valid inputs:
 * {
 *     "id": 123,
 *     "payload": {
 *         "command": "pause",
 *         "reason": "checkpoint"
 *     }
 * }
 *
 * {
 *     "id": 456,
 *     "payload": {
 *         "exit_code": "0",
 *         "message": "normal_termination",
 *         "timestamp": "12345678"
 *     }
 * }
 *
 * @param sigtype Signal type that triggered this handler
 * @note The handler communicates completion by writing "done" to shared
 *       memory
 * @note Maximum message size is limited to 4096 bytes
 */
static void
externalProcessHandler(int sigtype)
{
    async_event = true;
    async_hypercall = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();

}

void
processExternalSignal(void)
{
    std::string shared_mem_name_str = "/shared_gem5_signal_mem_" +
        std::to_string(getpid());
    const char* shared_mem_name = shared_mem_name_str.c_str();
    const std::size_t shared_mem_size = 4096;

    int shm_fd = shm_open(shared_mem_name, O_RDWR, 0666); //0666 = rw-rw-rw-
    if (shm_fd == -1) {
        DPRINTF(ExternalSignal, "Error: Unable to open shared memory: %s\n",
                std::strerror(errno));
        return;
    }

    void* shm_ptr = mmap(0, shared_mem_size, PROT_READ | PROT_WRITE,
        MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        DPRINTF(ExternalSignal, "Error: Unable to map shared memory: %s\n",
                std::strerror(errno));
        close(shm_fd);
        return;
    }

    char full_payload[shared_mem_size];
    std::memcpy(full_payload, shm_ptr, shared_mem_size);
    full_payload[shared_mem_size - 1] = '\0';  // Ensure null-termination

    DPRINTF(ExternalSignal, "Received signal from external "
            "process with payload: '%s'\n", full_payload);

    // process payload json with string processing
    std::string full_payload_str = full_payload;
    std::map<std::string, std::string> payload_map;

    // Find ID field
    std::string id_field("\"id\":");
    std::size_t id_pos = full_payload_str.find(id_field);
    if (id_pos == std::string::npos) {
        munmap(shm_ptr, shared_mem_size);
        close(shm_fd);
        warn("Error: No message ID found in external processes payload\n");
        return;
    }

    // Skip past "id":
    id_pos += id_field.length();

    // Skip whitespace
    while (id_pos < full_payload_str.length() && isspace(id_pos)) {
        id_pos++;
    }

    // Find end of number (comma or closing brace)
    std::size_t id_end = full_payload_str.find_first_of(",}", id_pos);
    if (id_end == std::string::npos) {
        munmap(shm_ptr, shared_mem_size);
        close(shm_fd);
        warn("Error: Invalid ID format in external processes payload\n");
        return;
    }

    std::string message_id_str = full_payload_str.substr(id_pos,
                                                         id_end - id_pos);
    // Trim whitespace
    while (!message_id_str.empty() && isspace(message_id_str.back())) {
        message_id_str.pop_back();
    }

    long long id_value = std::stoll(message_id_str);
    if (id_value < 0) {
        munmap(shm_ptr, shared_mem_size);
        close(shm_fd);
        warn("External Process Handler Error: Invalid ID format "
             "- must be a valid non-negative 64-bit integer\n");
        return;
    }
    uint64_t hypercall_id = static_cast<uint64_t>(id_value);

    // parse the payload. Start looking for key-value pairs after `"payload":`
    std::string payload_key = "\"payload\":";
    std::size_t payload_pos = full_payload_str.find(payload_key) +
                                payload_key.length();

    // Skip opening brace of payload object
    payload_pos = full_payload_str.find('{', payload_pos) + 1;

    while (payload_pos < full_payload_str.length() &&
           full_payload_str[payload_pos] != '}') {
        // Skip whitespace and commas
        payload_pos = full_payload_str.find_first_not_of(", \n\r\t",
                                                         payload_pos);
        if (payload_pos == std::string::npos ||
            full_payload_str[payload_pos] == '}') {
            break;
        }

        // Extract key (must be quoted)
        std::string key = extractStringFromJSON(full_payload_str, "\"", "\"",
            payload_pos);

        // Skip to the value after the colon
        payload_pos = full_payload_str.find(":", payload_pos);
        if (payload_pos == std::string::npos) {
            break;
        }
        payload_pos++; // Move past the colon

        // Skip whitespace before the value
        payload_pos = full_payload_str.find_first_not_of(" \n\r\t",
                                                         payload_pos);
        if (payload_pos == std::string::npos) {
            break;
        }

        // Extract value - handle both quoted and unquoted values
        std::string value;
        if (full_payload_str[payload_pos] == '"') {
            value = extractStringFromJSON(full_payload_str, "\"", "\"",
                                          payload_pos);
        } else {
            // For unquoted values, read until comma or closing brace
            std::size_t value_end = full_payload_str.find_first_of(",}",
                                                            payload_pos);
            if (value_end == std::string::npos) {
                break;
            }
            value = full_payload_str.substr(payload_pos,
                                            value_end - payload_pos);
            // Trim whitespace
            while (!value.empty() && isspace(value.back())) {
                value.pop_back();
            }
            payload_pos = value_end;
        }

        if (!key.empty() && !value.empty()) {
            payload_map[key] = value;
            DPRINTF(ExternalSignal, "Parsed key-value pair: %s: %s\n",
                                    key, value);
        }
    }

    // put a "done" message into the shared memory so the transmitter knows to
    // close and unlink the memory on its end.
    char done_msg[shared_mem_size] = "done";
    done_msg[shared_mem_size - 1] = '\0';  // Ensure null-termination
    std::memcpy(shm_ptr, done_msg, sizeof(done_msg));

    munmap(shm_ptr, shared_mem_size);
    close(shm_fd);

    exitSimLoopWithHypercall("Handling external signal!", 0, curTick(), 0,
                             payload_map, hypercall_id, false);
}

std::string
extractStringFromJSON(std::string& full_str, std::string start_str,
    std::string end_str, std::size_t& search_start)
{
    // Find the starting position
    std::size_t start = full_str.find(start_str, search_start);
    if (start == std::string::npos) {
        return "";
    }
    start += start_str.size();

    // Skip whitespace after start marker
    while (start < full_str.length() && isspace(full_str[start])) {
        start++;
    }

    // Find the ending position
    std::size_t end = full_str.find(end_str, start);
    if (end == std::string::npos) {
        return "";
    }

    // Trim whitespace before end marker
    while (end > start && isspace(full_str[end - 1])) {
        end--;
    }

    // Update search position to continue after this value
    search_start = end + end_str.size();

    return full_str.substr(start, end - start);
}

/*
 * M5 can do several special things when various signals are sent.
 * None are mandatory.
 */
void
initSignals()
{
    // Floating point exceptions may happen on misspeculated paths, so
    // ignore them
    signal(SIGFPE, SIG_IGN);

    // Dump intermediate stats
    installSignalHandler(SIGUSR1, dumpStatsHandler);

    // Dump intermediate stats and reset them
    installSignalHandler(SIGUSR2, dumprstStatsHandler);

    // Print the current cycle number and a backtrace on abort. Make
    // sure the signal is unmasked and the handler reset when a signal
    // is delivered to be able to invoke the default handler.
    installSignalHandler(SIGABRT, abortHandler, SA_RESETHAND | SA_NODEFER);

    // Setup a SIGSEGV handler with a private stack
    if (setupAltStack()) {
        installSignalHandler(SIGSEGV, segvHandler,
                             SA_RESETHAND | SA_NODEFER | SA_ONSTACK);
    } else {
        warn("Failed to setup stack for SIGSEGV handler, "
             "using default signal handler.\n");
    }

    // Install a SIGIO handler to handle asynchronous file IO. See the
    // PollQueue class.
    installSignalHandler(SIGIO, ioHandler);
}

void initSigCont()
{
    installSignalHandler(SIGCONT, externalProcessHandler);
}

struct sigaction old_int_sa;

void initSigInt()
{
    // Exit cleanly on Interrupt (Ctrl-C)
    installSignalHandler(SIGINT, exitNowHandler, SA_RESTART, &old_int_sa);
}

void restoreSigInt()
{
    // Restore the old SIGINT handler
    sigaction(SIGINT, &old_int_sa, NULL);
}


} // namespace gem5
