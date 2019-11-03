
// Unit test for tlm_generic_payload update_original_from() method

#include <iomanip>

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "systemc"
using namespace sc_core;
using namespace sc_dt;
using namespace std;

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

#include "mm.h"

struct my_extension: tlm::tlm_extension<my_extension>
{
  my_extension() {}

  virtual tlm_extension_base* clone() const
  {
    my_extension* ext = new my_extension;
    ext->len = len;
    ext->bel = bel;
    ext->ptr = ptr;
    ext->byt = byt;
    return ext;
  }

  virtual void copy_from(tlm_extension_base const &ext)
  {
    len = static_cast<my_extension const &>(ext).len;
    bel = static_cast<my_extension const &>(ext).bel;
    ptr = static_cast<my_extension const &>(ext).ptr;
    byt = static_cast<my_extension const &>(ext).byt;
  }

  unsigned int len;
  unsigned int bel;
  unsigned char* ptr;
  unsigned char* byt;
};


struct Initiator: sc_module
{
  tlm_utils::simple_initiator_socket<Initiator> socket;

  typedef unsigned char uchar;

  SC_CTOR(Initiator)
  : socket("socket")
  {
    SC_THREAD(thread_process);
    data = new uchar[32];
    byte_enable = new uchar[32];
  }

  void thread_process()
  {
    tlm::tlm_generic_payload* trans;
    sc_time delay = SC_ZERO_TIME;

    for (int i = 0; i < 1000000; i++)
    {
      int addr = 0;
      tlm::tlm_command cmd = static_cast<tlm::tlm_command>(rand() % 2);

      unsigned int len = rand() % 33;
      unsigned int bel = rand() % (len + 1);

      // Fix sizes when performing a speed test
      //cmd = tlm::TLM_READ_COMMAND;
      //len = 32;
      //bel = 8;


      for (unsigned int i = 0; i < len; i++)
        if (cmd == tlm::TLM_WRITE_COMMAND)
        {
          data[i] = rand() % 256;
        }
        else
          data[i] = 0x99;

      trans = m_mm.allocate();
      trans->acquire();

      trans->set_command( cmd );
      trans->set_address( addr );
      trans->set_data_ptr( data );
      trans->set_data_length( len );
      trans->set_streaming_width( len );

      trans->set_byte_enable_length( bel );

      for (unsigned int i = 0; i < bel; i++)
      {
        byte_enable[i] = (rand() % 2) ? 0xff : 0;
      }

      if (bel)
        trans->set_byte_enable_ptr( byte_enable );
      else
        trans->set_byte_enable_ptr( 0 );

      trans->set_dmi_allowed( false );
      trans->set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );

      // Add sticky extension diagnostics
      my_extension* ext;
      trans->get_extension(ext);
      if (!ext)
      {
        ext = new my_extension;
        trans->set_extension(ext);
      }

      ext->len = len;
      ext->bel = bel;
      ext->ptr = data;
      ext->byt = byte_enable;

      socket->b_transport( *trans, delay );  // Blocking transport call

      if ( trans->is_response_error() )
        SC_REPORT_ERROR("TLM-2", "Response error from b_transport");

      //cout << "cmd = " << cmd << ", len = " << len << ", bel = " << bel << endl;
      //cout << hex << setw(2) << setfill('0') << (unsigned int)data[i];

      if (cmd == tlm::TLM_READ_COMMAND)
      {
        for (unsigned int i = 0; i < len; i++)
          if (bel)
            if (byte_enable[i % bel])
              sc_assert( data[i] == ext->ptr[i] );
            else
              sc_assert( data[i] == 0x99 );
          else
            sc_assert( data[i] == ext->ptr[i] );

      }
      trans->release();
    }
  }

  mm  m_mm;  // Memory manager
  unsigned char* data; // 32 bytes
  unsigned char* byte_enable;
};


struct Interconnect: sc_module
{
  tlm_utils::simple_target_socket   <Interconnect> targ_socket;
  tlm_utils::simple_initiator_socket<Interconnect> init_socket;

  SC_CTOR(Interconnect)
  : targ_socket("targ_socket")
  , init_socket("init_socket")
  {
    targ_socket.register_b_transport(this, &Interconnect::b_transport);
  }

  virtual void b_transport( tlm::tlm_generic_payload& trans, sc_time& delay )
  {
    unsigned int     bel = trans.get_byte_enable_length();

    trans2.set_data_ptr( data );
    if (bel)
      trans2.set_byte_enable_ptr( byte_enable );
    trans2.deep_copy_from(trans);

    init_socket->b_transport( trans2, delay );

    trans.update_original_from( trans2 );
  }

  tlm::tlm_generic_payload trans2;
  unsigned char data[32];
  unsigned char byte_enable[32];
};


struct Target: sc_module
{
  tlm_utils::simple_target_socket<Target> socket;

  SC_CTOR(Target)
  : socket("socket")
  {
    socket.register_b_transport(this, &Target ::b_transport);

    typedef unsigned char uchar;
    data = new uchar[32];
  }


  virtual void b_transport( tlm::tlm_generic_payload& trans, sc_time& delay )
  {
    tlm::tlm_command cmd = trans.get_command();
    unsigned char*   ptr = trans.get_data_ptr();
    unsigned int     len = trans.get_data_length();
    unsigned char*   byt = trans.get_byte_enable_ptr();
    unsigned int     bel = trans.get_byte_enable_length();

    my_extension* ext;
    trans.get_extension(ext);
    sc_assert( ext );

    sc_assert( len == ext->len );
    sc_assert( bel == ext->bel );
    for (unsigned int i = 0; i < bel; i++)
      sc_assert( byt[i] == ext->byt[i] );
    for (unsigned int i = 0; i < len; i++)
      sc_assert( ptr[i] == ext->ptr[i] );

    if (cmd == tlm::TLM_READ_COMMAND)
    {
      for (unsigned int i = 0; i < len; i++)
      {
        data[i] = rand() % 256;
        ptr[i]  = data[i];
      }
      ext->ptr = data;
    }

    trans.set_dmi_allowed( true );
    trans.set_response_status( tlm::TLM_OK_RESPONSE );
  }

  unsigned char* data; // 32 bytes
};


SC_MODULE(Top)
{
  Initiator    *initiator;
  Interconnect *interconnect;
  Target       *target;

  SC_CTOR(Top)
  {
    // Instantiate components
    initiator    = new Initiator   ("initiator");
    interconnect = new Interconnect("interconnect");
    target       = new Target      ("target");

    // One initiator is bound directly to one target with no intervening bus

    // Bind initiator socket to target socket
    initiator    ->socket     .bind( interconnect->targ_socket );
    interconnect ->init_socket.bind( target      ->socket );
  }
};


int sc_main(int argc, char* argv[])
{
  cout << "Unit test for update_original_from(). Should remain silent\n";

  Top top("top");
  sc_start();
  return 0;
}

