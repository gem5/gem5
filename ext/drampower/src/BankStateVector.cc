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
        if (newState == BANK_ACTIVE)
        {
            _num_active_banks++;
        }
        else
        {
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
