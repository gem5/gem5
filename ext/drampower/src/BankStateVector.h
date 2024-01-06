#include <vector>
#include <iostream>

using namespace std;

namespace Data{
enum BankState 
{
    BANK_PRECHARGED = 0,
    BANK_ACTIVE
};

class BankStateVector
{
    public:
        BankStateVector();
        void SetSize(int64_t);
        void clear();
        BankStateVector& operator&= (BankStateVector& bankVector);
        void SetByIndex(BankState, int);
        BankState GetByIndex (unsigned);
        unsigned get_num_active_banks();
        size_t size();

    private:
        std::vector<BankState> bank_state;
        unsigned _num_active_banks;
};
}