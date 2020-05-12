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
 */

#ifndef __MEM_TOKEN_PORT_HH__
#define __MEM_TOKEN_PORT_HH__

#include "mem/port.hh"
#include "sim/clocked_object.hh"

class TokenManager;
class TokenSlavePort;

class TokenMasterPort : public MasterPort
{
  private:
    /* Manager to track tokens between this token port pair. */
    TokenManager *tokenManager;

  public:
    TokenMasterPort(const std::string& name, SimObject* owner,
                    PortID id = InvalidPortID) :
        MasterPort(name, owner, id), tokenManager(nullptr)
    { }

    /**
     * Bind this master port to slave port. Called by the slave port in
     * this token implementation.
     */
    void bind(Port &peer) override;

    /**
     * Unbind port. Handled by slave port in token implementation.
     */
    void unbind() override {}

    /**
     * Receive tokens returned by the slave port. This increments the number
     * or available tokens across the port.
     */
    void recvTokens(int num_tokens);

    /**
     * Query if there are at least num_tokens tokens available to acquire.
     */
    bool haveTokens(int num_tokens);

    /**
     * Acquire tokens by decrementing the number of available tokens across
     * the port. This does the opposite of recvTokens.
     */
    void acquireTokens(int num_tokens);

    /**
     * Specify a token manger, which will handle tracking of tokens for a
     * TokenMasterPort/SlaveMasterPort pair.
     */
    void setTokenManager(TokenManager *_tokenManager);
};

class TokenSlavePort : public SlavePort
{
  private:
    TokenMasterPort *tokenMasterPort;

    std::deque<PacketPtr> respQueue;

    void recvRespRetry() override;

  public:
    TokenSlavePort(const std::string& name, ClockedObject *owner,
                   PortID id = InvalidPortID) :
        SlavePort(name, owner, id), tokenMasterPort(nullptr)
    { }
    ~TokenSlavePort() { }

    /**
     * Bind this slave port to a master port. This also does the mirror
     * action and bainds the master port to the slave port as well as
     * binding the base class types.
     */
    void bind(Port &peer) override;

    /**
     * Unbind this slave port and associated master port.
     */
    void unbind() override;

    /**
     * Return num_tokens tokens back to the master port.
     */
    void sendTokens(int num_tokens);

    bool sendTimingResp(PacketPtr pkt);

    /* There is no storage here so the packet will not be found. */
    bool trySatisfyFunctional(PacketPtr) { return false; }
};

class TokenManager
{
  protected:
    /* Maximum tokens possible */
    int maxTokens;

    /* Number of currently available tokens */
    int availableTokens;

  public:
    TokenManager(int init_tokens);
    ~TokenManager() { }

    /**
     * Return the maximum possible tokens.
     */
    int getMaxTokenCount() const;

    /**
     * Increment the number of available tokens by num_tokens.
     */
    void recvTokens(int num_tokens);

    /**
     * Query is num_tokens tokens are available.
     */
    bool haveTokens(int num_tokens);

    /**
     * Decrement the number of available tokens by num_tokens.
     */
    void acquireTokens(int num_tokens);
};

#endif
