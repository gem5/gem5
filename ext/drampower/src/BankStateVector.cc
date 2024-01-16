/*
 * Copyright (c) <YOU OR YOUR COMPANY>
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

#include <vector>

#include "BankStateVector.h"

using namespace std;
using namespace Data;

BankStateVector::BankStateVector()
{

}

size_t BankStateVector::size()
{
    return bank_state.size();
}

void BankStateVector::SetSize(int64_t size)
{
    bank_state.resize(static_cast<size_t>(size), BANK_PRECHARGED);
    _num_active_banks = 0;
}

void BankStateVector::clear()
{
    _num_active_banks = 0;
    bank_state.clear();
}

BankStateVector& BankStateVector::operator&= (BankStateVector& anotherBankVector)
{
    for (int i = 0; i < bank_state.size(); i++)
    {
        bank_state[i] = anotherBankVector.GetByIndex(i);
    }
    _num_active_banks = anotherBankVector.get_num_active_banks();
    return *this;
}

void BankStateVector::SetByIndex(BankState newState, int index)
{
    if (bank_state[index] != newState)
    {
        if (newState == BANK_ACTIVE){
            _num_active_banks++;
        }
        else{
            _num_active_banks--;
        }
        bank_state[index] = newState;
    }
}

BankState BankStateVector::GetByIndex(unsigned index)
{
    return bank_state[index];
}

unsigned BankStateVector::get_num_active_banks()
{
    return _num_active_banks;
}
