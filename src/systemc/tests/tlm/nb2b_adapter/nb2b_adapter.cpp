
// Unit test for nb2b adapter in simple_target_socket, PEQ, and instance-specific extensions
// Checks for bug in original version of simple_target_socket

#include <iomanip>

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "systemc"
using namespace sc_core;
using namespace sc_dt;
using namespace std;

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/multi_passthrough_initiator_socket.h"
#include "tlm_utils/multi_passthrough_target_socket.h"
#include "tlm_utils/peq_with_cb_and_phase.h"
#include "tlm_utils/instance_specific_extensions.h"

#include "mm.h"


int rand_ps()
{
  int n = rand() % 100;
  n = n * n * n;
  return n / 100;
}


struct Initiator: sc_module
{
  tlm_utils::simple_initiator_socket<Initiator> socket;

  SC_CTOR(Initiator)
  : socket("socket")
  , request_in_progress(0)
  , m_peq(this, &Initiator::peq_cb)
  {
    socket.register_nb_transport_bw(this, &Initiator::nb_transport_bw);

    SC_THREAD(thread_process);
  }

  void thread_process()
  {
    tlm::tlm_generic_payload* trans;
    tlm::tlm_phase phase;
    sc_time delay;

    trans = m_mm.allocate();
    trans->acquire();

    int adr = 0;
    data[0] = adr;

    trans->set_command( tlm::TLM_WRITE_COMMAND );
    trans->set_address( adr );
    trans->set_data_ptr( reinterpret_cast<unsigned char*>(&data[0]) );
    trans->set_data_length( 4 );
    trans->set_streaming_width( 4 );
    trans->set_byte_enable_ptr( 0 );
    trans->set_dmi_allowed( false );
    trans->set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );

    socket->b_transport( *trans, delay );

    trans->release();

    for (int i = 0; i < 5000; i++)
    {
      int adr = rand();
      tlm::tlm_command cmd = static_cast<tlm::tlm_command>(rand() % 2);
      if (cmd == tlm::TLM_WRITE_COMMAND) data[i % 16] = adr;

      // Grab a new transaction from the memory manager
      trans = m_mm.allocate();
      trans->acquire();

      trans->set_command( cmd );
      trans->set_address( adr );
      trans->set_data_ptr( reinterpret_cast<unsigned char*>(&data[i % 16]) );
      trans->set_data_length( 4 );
      trans->set_streaming_width( 4 );
      trans->set_byte_enable_ptr( 0 );
      trans->set_dmi_allowed( false );
      trans->set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );

      if (request_in_progress)
        wait(end_request_event);
      request_in_progress = trans;
      phase = tlm::BEGIN_REQ;

      delay = sc_time(rand_ps(), SC_PS);

      tlm::tlm_sync_enum status;
      status = socket->nb_transport_fw( *trans, phase, delay );
      previous_time = sc_time_stamp() + delay;

      if (status == tlm::TLM_UPDATED)
      {
        m_peq.notify( *trans, phase, delay );
      }
      else if (status == tlm::TLM_COMPLETED)
      {
        request_in_progress = 0;

        check_transaction( *trans );

        trans->release();
      }
      wait( sc_time(rand_ps(), SC_PS) );
    }
  }

  virtual tlm::tlm_sync_enum nb_transport_bw( tlm::tlm_generic_payload& trans,
                                              tlm::tlm_phase& phase, sc_time& delay )
  {
    if (sc_time_stamp() + delay < previous_time)
      SC_REPORT_FATAL("TLM-2", "nb_transport_bw called with decreasing timing annotation");
    previous_time = sc_time_stamp() + delay;

    m_peq.notify( trans, phase, delay );
    return tlm::TLM_ACCEPTED;
  }

  void peq_cb(tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase)
  {
    if (phase == tlm::END_REQ || (&trans == request_in_progress && phase == tlm::BEGIN_RESP))
    {
      request_in_progress = 0;
      end_request_event.notify();
    }
    else if (phase == tlm::BEGIN_REQ || phase == tlm::END_RESP)
      SC_REPORT_FATAL("TLM-2", "Illegal transaction phase received by initiator");

    if (phase == tlm::BEGIN_RESP)
    {
      check_transaction( trans );

      tlm::tlm_phase fw_phase = tlm::END_RESP;
      sc_time delay = sc_time(rand_ps(), SC_PS);
      socket->nb_transport_fw( trans, fw_phase, delay );
      previous_time = sc_time_stamp() + delay;

      trans.release();
    }
  }

  void check_transaction(tlm::tlm_generic_payload& trans)
  {
    if ( trans.is_response_error() )
    {
      char txt[100];
      sprintf(txt, "Transaction returned with error, response status = %s",
                   trans.get_response_string().c_str());
      SC_REPORT_ERROR("TLM-2", txt);
    }

    tlm::tlm_command cmd = trans.get_command();
    sc_dt::uint64    adr = trans.get_address();
    int*             ptr = reinterpret_cast<int*>( trans.get_data_ptr() );

    if (cmd == tlm::TLM_READ_COMMAND)
      sc_assert( *ptr == -int(adr) );
  }

  mm   m_mm;
  int  data[16];
  tlm::tlm_generic_payload* request_in_progress;
  sc_event end_request_event;
  tlm_utils::peq_with_cb_and_phase<Initiator> m_peq;
  sc_time previous_time;
};


// Dumb interconnect that simply routes transactions through

struct Interconnect: sc_module
{
  tlm_utils::multi_passthrough_target_socket<Interconnect, 32>    targ_socket;
  tlm_utils::multi_passthrough_initiator_socket<Interconnect, 32> init_socket;

  Interconnect(sc_module_name _name, unsigned int _offset)
  : sc_module(_name)
  , targ_socket("targ_socket")
  , init_socket("init_socket")
  , offset(_offset)
  {
    targ_socket.register_b_transport              (this, &Interconnect::b_transport);
    targ_socket.register_nb_transport_fw          (this, &Interconnect::nb_transport_fw);
    targ_socket.register_get_direct_mem_ptr       (this, &Interconnect::get_direct_mem_ptr);
    targ_socket.register_transport_dbg            (this, &Interconnect::transport_dbg);
    init_socket.register_nb_transport_bw          (this, &Interconnect::nb_transport_bw);
    init_socket.register_invalidate_direct_mem_ptr(this, &Interconnect::invalidate_direct_mem_ptr);
  }

  void end_of_elaboration()
  {
    if ( targ_socket.size() != init_socket.size() )
      SC_REPORT_ERROR("TLM-2", "#initiators != #targets in Interconnect");
  }

  virtual void b_transport( int id, tlm::tlm_generic_payload& trans, sc_time& delay )
  {
    unsigned int target = (id + offset) % init_socket.size(); // Route-through

    init_socket[target]->b_transport( trans, delay );
  }


  struct route_extension: tlm_utils::instance_specific_extension<route_extension>
  {
    int id;
  };

  tlm_utils::instance_specific_extension_accessor accessor;


  virtual tlm::tlm_sync_enum nb_transport_fw( int id, tlm::tlm_generic_payload& trans,
                                              tlm::tlm_phase& phase, sc_time& delay )
  {
    route_extension* ext = 0;
    if (phase == tlm::BEGIN_REQ)
    {
      ext = new route_extension;
      ext->id = id;
      accessor(trans).set_extension(ext);
    }

    unsigned int target = (id + offset) % init_socket.size(); // Route-through

    tlm::tlm_sync_enum status;
    status = init_socket[target]->nb_transport_fw( trans, phase, delay );

    if (status == tlm::TLM_COMPLETED)
    {
      accessor(trans).clear_extension(ext);
      delete ext;
    }

    return status;
  }

  virtual bool get_direct_mem_ptr( int id, tlm::tlm_generic_payload& trans,
                                           tlm::tlm_dmi& dmi_data)
  {
    unsigned int target = (id + offset) % init_socket.size(); // Route-through

    bool status = init_socket[target]->get_direct_mem_ptr( trans, dmi_data );

    return status;
  }

  virtual unsigned int transport_dbg( int id, tlm::tlm_generic_payload& trans )
  {
    unsigned int target = (id + offset) % init_socket.size(); // Route-through

    return init_socket[target]->transport_dbg( trans );
  }


  virtual tlm::tlm_sync_enum nb_transport_bw( int id, tlm::tlm_generic_payload& trans,
                                              tlm::tlm_phase& phase, sc_time& delay )
  {
    route_extension* ext = 0;
    accessor(trans).get_extension(ext);
    sc_assert(ext);

    tlm::tlm_sync_enum status;
    status = targ_socket[ ext->id ]->nb_transport_bw( trans, phase, delay );

    if (status == tlm::TLM_COMPLETED)
    {
      accessor(trans).clear_extension(ext);
      delete ext;
    }

    return status;
  }

  virtual void invalidate_direct_mem_ptr( int id, sc_dt::uint64 start_range,
                                                  sc_dt::uint64 end_range )
  {
    for (unsigned int i = 0; i < targ_socket.size(); i++)
      targ_socket[i]->invalidate_direct_mem_ptr(start_range, end_range);
  }

  unsigned int offset;
};


struct Target: sc_module
{
  tlm_utils::simple_target_socket<Target> socket;

  SC_CTOR(Target)
  : socket("socket")
  {
    socket.register_b_transport    (this, &Target::b_transport);
  }

  virtual void b_transport( tlm::tlm_generic_payload& trans, sc_time& delay )
  {
    execute_transaction(trans);
  }


  void execute_transaction(tlm::tlm_generic_payload& trans)
  {
    tlm::tlm_command cmd = trans.get_command();
    sc_dt::uint64    adr = trans.get_address();
    unsigned char*   ptr = trans.get_data_ptr();
    unsigned int     len = trans.get_data_length();
    unsigned char*   byt = trans.get_byte_enable_ptr();
    unsigned int     wid = trans.get_streaming_width();

    if (byt != 0) {
      trans.set_response_status( tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE );
      return;
    }
    if (len > 4 || wid < len) {
      trans.set_response_status( tlm::TLM_BURST_ERROR_RESPONSE );
      return;
    }

    if ( cmd == tlm::TLM_READ_COMMAND )
    {
      *reinterpret_cast<int*>(ptr) = -int(adr);
    }
    else if ( cmd == tlm::TLM_WRITE_COMMAND )
    {
      sc_assert( *reinterpret_cast<unsigned int*>(ptr) == adr );
    }

    trans.set_response_status( tlm::TLM_OK_RESPONSE );
  }

};


SC_MODULE(Top)
{
  Initiator    *initiator1;
  Initiator    *initiator2;
  Interconnect *interconnect;
  Target       *target1;
  Target       *target2;

  SC_CTOR(Top)
  {
    initiator1   = new Initiator   ("initiator1");
    initiator2   = new Initiator   ("initiator2");
    interconnect = new Interconnect("interconnect", 1);
    target1      = new Target      ("target1");
    target2      = new Target      ("target2");

    initiator1->socket.bind(interconnect->targ_socket);
    initiator2->socket.bind(interconnect->targ_socket);
    interconnect->init_socket.bind(target1->socket);
    interconnect->init_socket.bind(target2->socket);
  }
};


int sc_main(int argc, char* argv[])
{
  cout << "Unit test for nb2b adapter, PEQ, and instance-specific extensions. Should remain silent\n";

  Top top("top");
  sc_start();
  return 0;
}

