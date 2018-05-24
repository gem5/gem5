#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc"
using namespace sc_core;
using namespace sc_dt;
using namespace std;

#include "tlm.h"
#include "tlm_utils/peq_with_cb_and_phase.h"
#include "tlm_utils/peq_with_get.h"


SC_MODULE(Test_peq_with_cb)
{

  SC_CTOR(Test_peq_with_cb)
  : m_peq(this, &Test_peq_with_cb::peq_cb), flag1(true), flag2(true)
  {
    SC_THREAD(thread);
  }

  void thread()
  {
    section = 1;

    tlm::tlm_generic_payload *trans;
    tlm::tlm_phase phase;
    for (int i = 0; i < 50; i++)
    {
      trans = new tlm::tlm_generic_payload;
      trans->set_address(i);
      m_peq.notify( *trans, phase, sc_time(rand() % 100, SC_NS) );
    }
    wait(50, SC_NS);

    m_peq.cancel_all();
    cout << "cancel_all\n";
    section = 2;

    for (int i = 100; i < 150; i++)
    {
      trans = new tlm::tlm_generic_payload;
      trans->set_address(i);
      m_peq.notify( *trans, phase, sc_time(rand() % 100, SC_NS) );
    }
    wait(50, SC_NS);
    m_peq.cancel_all();
    cout << "cancel_all\n";

    wait(50, SC_NS);
  }

  void peq_cb(tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase)
  {
    sc_time t = sc_time_stamp();
    sc_dt::uint64 adr = trans.get_address();
    sc_assert( section == 1 || section == 2 );
    if (section == 1)
    {
      if (flag1) cout << "Called peq_cb with section = " << section << "\n";
      flag1 = false;
      sc_assert( t >= sc_time(0, SC_NS) && t <= sc_time(50, SC_NS) );
      sc_assert( adr >= 0 && adr < 50);
    }
    else if (section == 2)
    {
      if (flag2) cout << "Called peq_cb with section = " << section << "\n";
      flag2 = false;
      sc_assert( t >= sc_time(50, SC_NS) && t <= sc_time(100, SC_NS) );
      sc_assert( adr >= 100 && adr < 150);
    }
  }

  int section;
  tlm_utils::peq_with_cb_and_phase<Test_peq_with_cb> m_peq;
  bool flag1, flag2;
};


SC_MODULE(Test_peq_with_get)
{

  SC_CTOR(Test_peq_with_get)
  : m_peq("foo"), flag3(true), flag4(true)
  {
    SC_THREAD(thread);
    SC_THREAD(ass_end_thread);
  }

  void thread()
  {
    wait(1000, SC_NS);

    section = 3;

    tlm::tlm_generic_payload *trans;
    for (int i = 0; i < 50; i++)
    {
      trans = new tlm::tlm_generic_payload;
      trans->set_address(i);
      m_peq.notify( *trans, sc_time(rand() % 100, SC_NS) );
    }
    wait(50, SC_NS);

    m_peq.cancel_all();
    cout << "cancel_all\n";
    section = 4;

    for (int i = 100; i < 150; i++)
    {
      trans = new tlm::tlm_generic_payload;
      trans->set_address(i);
      m_peq.notify( *trans, sc_time(rand() % 100, SC_NS) );
    }
    wait(50, SC_NS);
    m_peq.cancel_all();
    cout << "cancel_all\n";

    wait(50, SC_NS);
  }

  void ass_end_thread()
  {
    tlm::tlm_generic_payload *trans;
    for(;;)
    {
      wait(m_peq.get_event());
      while( (trans = m_peq.get_next_transaction()) )
      {
        sc_time t = sc_time_stamp();
        sc_dt::uint64 adr = trans->get_address();
        sc_assert( section == 3 || section == 4 );
        if (section == 3)
        {
          if (flag3) cout << "Called get_next_transaction with section = " << section << "\n";
          flag3 = false;
          sc_assert( t >= sc_time(1000, SC_NS) && t <= sc_time(1050, SC_NS) );
          sc_assert( adr >= 0 && adr < 50);
        }
        else if (section == 4)
        {
          if (flag4) cout << "Called get_next_transaction with section = " << section << "\n";
          flag4 = false;
          sc_assert( t >= sc_time(1050, SC_NS) && t <= sc_time(1100, SC_NS) );
          sc_assert( adr >= 100 && adr < 150);
        }
      }
    }
  }

  int section;
  tlm_utils::peq_with_get<tlm::tlm_generic_payload> m_peq;
  bool flag3, flag4;
};


int sc_main(int argc, char* argv[])
{
  cout << "Unit test for cancel_all()\n";
  Test_peq_with_cb  cb("test_peq_with_cb");
  Test_peq_with_get get("test_peq_with_get");
  sc_start();
  return 0;
}

