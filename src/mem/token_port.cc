/*
 * Copyright (c) 2016-2020 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthew Poremba
 */


#include "mem/token_port.hh"

#include "base/trace.hh"
#include "debug/TokenPort.hh"

void
TokenMasterPort::bind(Port &peer)
{
    MasterPort::bind(peer);
}

void
TokenMasterPort::recvTokens(int num_tokens)
{
    panic_if(!tokenManager, "TokenManager not set for %s.\n", name());

    tokenManager->recvTokens(num_tokens);
}

bool
TokenMasterPort::haveTokens(int num_tokens)
{
    panic_if(!tokenManager, "TokenManager not set for %s.\n", name());

    return tokenManager->haveTokens(num_tokens);
}

void
TokenMasterPort::acquireTokens(int num_tokens)
{
    panic_if(!tokenManager, "TokenManager not set for %s.\n", name());

    tokenManager->acquireTokens(num_tokens);
}

void
TokenMasterPort::setTokenManager(TokenManager *_tokenManager)
{
    tokenManager = _tokenManager;
}

void
TokenSlavePort::sendTokens(int num_tokens)
{
    fatal_if(!tokenMasterPort, "Tried sendTokens to non-token master!\n");

    // Send tokens to a master
    tokenMasterPort->recvTokens(num_tokens);
}

void
TokenSlavePort::bind(Port& peer)
{
    // TokenSlavePort is allowed to bind to either TokenMasterPort or a
    // MasterPort as fallback. If the type is a MasterPort, tokenMasterPort
    // is set to nullptr to indicate tokens should not be exchanged.
    auto *token_master_port = dynamic_cast<TokenMasterPort*>(&peer);
    auto *master_port = dynamic_cast<MasterPort*>(&peer);
    if (!token_master_port && !master_port) {
        fatal("Attempt to bind port %s to unsupported slave port %s.",
              name(), peer.name());
    } else if (token_master_port) {
        // slave port keeps track of the master port
        tokenMasterPort = token_master_port;

        // master port also keeps track of slave port
        tokenMasterPort->bind(*this);
    } else if (master_port) {
        tokenMasterPort = nullptr;
    }
}

void
TokenSlavePort::unbind()
{
    SlavePort::slaveUnbind();
    tokenMasterPort = nullptr;
}

void
TokenSlavePort::recvRespRetry()
{
    // fallback to QueuedSlavePort-like impl for now
    panic_if(respQueue.empty(),
             "Attempted to retry a response when no retry was queued!\n");

    PacketPtr pkt = respQueue.front();
    bool success = SlavePort::sendTimingResp(pkt);

    if (success) {
        respQueue.pop_front();
    }
}

bool
TokenSlavePort::sendTimingResp(PacketPtr pkt)
{
    bool success = SlavePort::sendTimingResp(pkt);

    if (!success) {
        respQueue.push_back(pkt);
    }

    return success;
}

TokenManager::TokenManager(int init_tokens)
{
    availableTokens = init_tokens;
    maxTokens = init_tokens;
}

int
TokenManager::getMaxTokenCount() const
{
    return maxTokens;
}

void
TokenManager::recvTokens(int num_tokens)
{
    availableTokens += num_tokens;

    DPRINTF(TokenPort, "Received %d tokens, have %d\n",
                       num_tokens, availableTokens);

    panic_if(availableTokens > maxTokens,
             "More tokens available than the maximum after recvTokens!\n");
}

bool
TokenManager::haveTokens(int num_tokens)
{
    return (availableTokens >= num_tokens);
}

void
TokenManager::acquireTokens(int num_tokens)
{
    panic_if(!haveTokens(num_tokens),
             "Attempted to acquire more tokens than are available!\n");

    availableTokens -= num_tokens;

    DPRINTF(TokenPort, "Acquired %d tokens, have %d\n",
                       num_tokens, availableTokens);
}
