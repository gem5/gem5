/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

#ifndef TLM_UTILS_MULTI_SOCKET_BASES_H_INCLUDED_
#define TLM_UTILS_MULTI_SOCKET_BASES_H_INCLUDED_

#include <tlm>
#include "tlm_utils/convenience_socket_bases.h"

#include <map>

namespace tlm_utils {

template <typename signature>
struct fn_container{
  signature function;
};

#define TLM_DEFINE_FUNCTOR(name) \
template <typename MODULE, typename TRAITS> \
inline TLM_RET_VAL static_##name( void* mod \
                                       , void* fn \
                                       , int index \
                                       , TLM_FULL_ARG_LIST) \
{ \
  typedef fn_container<TLM_RET_VAL (MODULE::*)(int, TLM_FULL_ARG_LIST)> fn_container_type; \
  MODULE* tmp_mod=static_cast<MODULE*>(mod); \
  fn_container_type* tmp_cb =static_cast<fn_container_type*> (fn); \
  return (tmp_mod->*(tmp_cb->function))(index, TLM_ARG_LIST_WITHOUT_TYPES); \
}\
\
template <typename MODULE, typename TRAITS> \
inline void delete_fn_container_of_##name(void* fn) \
{ \
  typedef fn_container<TLM_RET_VAL (MODULE::*)(int, TLM_FULL_ARG_LIST)> fn_container_type; \
  fn_container_type* tmp_cb =static_cast<fn_container_type*> (fn); \
  if (tmp_cb) delete tmp_cb;\
} \
\
template <typename TRAITS> \
class name##_functor{ \
public: \
  typedef typename TRAITS::tlm_payload_type payload_type; \
  typedef typename TRAITS::tlm_phase_type   phase_type; \
  typedef TLM_RET_VAL (*call_fn)(void*,void*, int, TLM_FULL_ARG_LIST); \
  typedef void (*del_fn)(void*); \
\
  name##_functor(): m_fn(0), m_del_fn(0), m_mod(0), m_mem_fn(0){} \
  ~name##_functor(){if (m_del_fn) (*m_del_fn)(m_mem_fn);}  \
\
  template <typename MODULE> \
  void set_function(MODULE* mod, TLM_RET_VAL (MODULE::*cb)(int, TLM_FULL_ARG_LIST)){ \
    typedef fn_container<TLM_RET_VAL (MODULE::*)(int, TLM_FULL_ARG_LIST)> fn_container_type; \
    m_fn=&static_##name<MODULE,TRAITS>;\
    m_del_fn=&delete_fn_container_of_##name<MODULE,TRAITS>;\
    m_del_fn(m_mem_fn); \
    fn_container_type* tmp= new fn_container_type(); \
    tmp->function=cb; \
    m_mod=static_cast<void*>(mod); \
    m_mem_fn=static_cast<void*>(tmp); \
  } \
  \
  TLM_RET_VAL operator()(int index, TLM_FULL_ARG_LIST){ \
    return m_fn(m_mod,m_mem_fn, index, TLM_ARG_LIST_WITHOUT_TYPES); \
  } \
\
  bool is_valid(){return (m_mod!=0 && m_mem_fn!=0 && m_fn!=0);}\
\
protected: \
  call_fn m_fn;\
  del_fn m_del_fn; \
  void* m_mod; \
  void* m_mem_fn; \
private: \
  name##_functor& operator=(const name##_functor&); \
}


#define TLM_RET_VAL tlm::tlm_sync_enum
#define TLM_FULL_ARG_LIST typename TRAITS::tlm_payload_type& txn, typename TRAITS::tlm_phase_type& ph, sc_core::sc_time& t
#define TLM_ARG_LIST_WITHOUT_TYPES txn,ph,t
TLM_DEFINE_FUNCTOR(nb_transport);
#undef TLM_RET_VAL
#undef TLM_FULL_ARG_LIST
#undef TLM_ARG_LIST_WITHOUT_TYPES

#define TLM_RET_VAL void
#define TLM_FULL_ARG_LIST typename TRAITS::tlm_payload_type& txn, sc_core::sc_time& t
#define TLM_ARG_LIST_WITHOUT_TYPES txn,t
TLM_DEFINE_FUNCTOR(b_transport);
#undef TLM_RET_VAL
#undef TLM_FULL_ARG_LIST
#undef TLM_ARG_LIST_WITHOUT_TYPES

#define TLM_RET_VAL unsigned int
#define TLM_FULL_ARG_LIST typename TRAITS::tlm_payload_type& txn
#define TLM_ARG_LIST_WITHOUT_TYPES txn
TLM_DEFINE_FUNCTOR(debug_transport);
#undef TLM_RET_VAL
#undef TLM_FULL_ARG_LIST
#undef TLM_ARG_LIST_WITHOUT_TYPES

#define TLM_RET_VAL bool
#define TLM_FULL_ARG_LIST typename TRAITS::tlm_payload_type& txn, tlm::tlm_dmi& dmi
#define TLM_ARG_LIST_WITHOUT_TYPES txn,dmi
TLM_DEFINE_FUNCTOR(get_dmi_ptr);
#undef TLM_RET_VAL
#undef TLM_FULL_ARG_LIST
#undef TLM_ARG_LIST_WITHOUT_TYPES

#define TLM_RET_VAL void
#define TLM_FULL_ARG_LIST sc_dt::uint64 l, sc_dt::uint64 u
#define TLM_ARG_LIST_WITHOUT_TYPES l,u
TLM_DEFINE_FUNCTOR(invalidate_dmi);
#undef TLM_RET_VAL
#undef TLM_FULL_ARG_LIST
#undef TLM_ARG_LIST_WITHOUT_TYPES

#undef TLM_DEFINE_FUNCTOR

/*
This class implements the fw interface.
It allows to register a callback for each of the fw interface methods.
The callbacks simply forward the fw interface call, but add the id (an int)
of the callback binder to the signature of the call.
*/
template <typename TYPES>
class callback_binder_fw
  : public tlm::tlm_fw_transport_if<TYPES>
  , protected convenience_socket_cb_holder
{
  public:
    //typedefs according to the used TYPES class
    typedef typename TYPES::tlm_payload_type              transaction_type;
    typedef typename TYPES::tlm_phase_type                phase_type;  
    typedef tlm::tlm_sync_enum                            sync_enum_type;
  
    //typedefs for the callbacks
    typedef nb_transport_functor<TYPES>    nb_func_type;
    typedef b_transport_functor<TYPES>     b_func_type;
    typedef debug_transport_functor<TYPES> debug_func_type;
    typedef get_dmi_ptr_functor<TYPES>     dmi_func_type;

    //ctor: an ID is needed to create a callback binder
    callback_binder_fw(multi_socket_base* owner, int id)
      : convenience_socket_cb_holder(owner), m_id(id)
      , m_nb_f(0), m_b_f(0), m_dbg_f(0), m_dmi_f(0)
      , m_caller_port(0)
    {}

    //the nb_transport method of the fw interface
    sync_enum_type nb_transport_fw(transaction_type& txn,
                                phase_type& p,
                                sc_core::sc_time& t){
      //check if a callback is registered
      if (m_nb_f && m_nb_f->is_valid()) {
        return (*m_nb_f)(m_id, txn, p, t); //do the callback
      }

      display_error("Call to nb_transport_fw without a registered callback for nb_transport_fw.");
      return tlm::TLM_COMPLETED;
    }

    //the b_transport method of the fw interface
    void b_transport(transaction_type& trans,sc_core::sc_time& t){
      //check if a callback is registered
      if (m_b_f && m_b_f->is_valid()) {
        (*m_b_f)(m_id, trans,t); //do the callback
        return;
      }

      display_error("Call to b_transport without a registered callback for b_transport.");
    }
    
    //the DMI method of the fw interface
    bool get_direct_mem_ptr(transaction_type& trans, tlm::tlm_dmi&  dmi_data){
      //check if a callback is registered
      if (m_dmi_f && m_dmi_f->is_valid()) {
        return (*m_dmi_f)(m_id, trans,dmi_data); //do the callback
      }

      dmi_data.allow_none();
      dmi_data.set_start_address(0x0);
      dmi_data.set_end_address((sc_dt::uint64)-1);
      return false;
    }
    
    //the debug method of the fw interface
    unsigned int transport_dbg(transaction_type& trans){
      //check if a callback is registered
      if (m_dbg_f && m_dbg_f->is_valid()) {
        return (*m_dbg_f)(m_id, trans); //do the callback
      }

      return 0;
    }
    
    //the SystemC standard callback register_port:
    // - called when a port if bound to the interface
    // - allowd to find out who is bound to that callback binder
    void register_port(sc_core::sc_port_base& b, const char* /*name*/){
      m_caller_port=&b;
    }
    
    //register callbacks for all fw interface methods at once
    void set_callbacks(nb_func_type& cb1, b_func_type& cb2, dmi_func_type& cb3, debug_func_type& cb4){
      m_nb_f=&cb1;
      m_b_f=&cb2;
      m_dmi_f=&cb3;
      m_dbg_f=&cb4;
    }
    
    //getter method to get the port that is bound to that callback binder
    // NOTE: this will only return a valid value at end of elaboration
    //  (but not before end of elaboration!)
    sc_core::sc_port_base* get_other_side(){return m_caller_port;}
    
  private:
    //the ID of the callback binder
    int m_id; 
    
    //the callbacks
    nb_func_type* m_nb_f; 
    b_func_type*  m_b_f;
    debug_func_type* m_dbg_f;
    dmi_func_type* m_dmi_f;
    
    //the port bound to that callback binder
    sc_core::sc_port_base* m_caller_port;   
};

/*
This class implements the bw interface.
It allows to register a callback for each of the bw interface methods.
The callbacks simply forward the bw interface call, but add the id (an int)
of the callback binder to the signature of the call.
*/
template <typename TYPES>
class callback_binder_bw
  : public tlm::tlm_bw_transport_if<TYPES>
  , protected convenience_socket_cb_holder
{
  public:
    //typedefs according to the used TYPES class
    typedef typename TYPES::tlm_payload_type              transaction_type;
    typedef typename TYPES::tlm_phase_type                phase_type;  
    typedef tlm::tlm_sync_enum                            sync_enum_type;
  
    //typedefs for the callbacks
    typedef nb_transport_functor<TYPES>   nb_func_type;
    typedef invalidate_dmi_functor<TYPES> dmi_func_type;

    //ctor: an ID is needed to create a callback binder
    callback_binder_bw(multi_socket_base* owner, int id)
      : convenience_socket_cb_holder(owner), m_id(id)
      , m_nb_f(0), m_dmi_f(0) {}

    //the nb_transport method of the bw interface
    sync_enum_type nb_transport_bw(transaction_type& txn,
                                phase_type& p,
                                sc_core::sc_time& t){
      //check if a callback is registered
      if (m_nb_f && m_nb_f->is_valid()) {
        return (*m_nb_f)(m_id, txn, p, t); //do the callback
      }

      display_error("Call to nb_transport_bw without a registered callback for nb_transport_bw");
      return tlm::TLM_COMPLETED;
    }
    
    //the DMI method of the bw interface
    void invalidate_direct_mem_ptr(sc_dt::uint64 l, sc_dt::uint64 u){
      //check if a callback is registered
      if (m_dmi_f && m_dmi_f->is_valid()) {
        (*m_dmi_f)(m_id,l,u); //do the callback
      }
    }

    //register callbacks for all bw interface methods at once
    void set_callbacks(nb_func_type& cb1, dmi_func_type& cb2){
      m_nb_f=&cb1;
      m_dmi_f=&cb2;
    }
    
  private:
    //the ID of the callback binder
    int m_id;
    //the callbacks
    nb_func_type* m_nb_f;
    dmi_func_type* m_dmi_f;
};

/*
This class forms the base for multi initiator sockets,
with fewer template parameters than the multi_init_base.
This class is implementation-defined.
*/
template <typename TYPES = tlm::tlm_base_protocol_types>
class multi_init_base_if {
public:
  //this method shall return a vector of the callback binders of multi initiator socket
  virtual std::vector<callback_binder_bw<TYPES>* >& get_binders()=0;
  //this method shall return a vector of all target interfaces bound to this multi init socket
  virtual std::vector<tlm::tlm_fw_transport_if<TYPES>*>& get_sockets()=0;
protected:
  virtual ~multi_init_base_if() {}
};

/*
This class forms the base for multi initiator sockets.
It enforces a multi initiator socket to implement all functions
needed to do hierarchical bindings.
*/
template <unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types,
          unsigned int N=0,
          sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND>
class multi_init_base
  : public tlm::tlm_initiator_socket<BUSWIDTH, TYPES, N, POL>
  , public multi_init_base_if<TYPES>
  , protected multi_socket_base
{
public:
  //typedef for the base type: the standard tlm initiator socket
  typedef tlm::tlm_initiator_socket<BUSWIDTH, TYPES, N, POL> base_type;
  
  //this method shall disable the code that does the callback binding
  // that registers callbacks to binders
  virtual void disable_cb_bind()=0;
  
  //this method shall return the multi_init_base to which the
  // multi_init_base is bound hierarchically
  //  If the base is not bound hierarchically it shall return a pointer to itself
  virtual multi_init_base* get_hierarch_bind()=0;
  
  virtual tlm::tlm_socket_category get_socket_category() const
  {
    return tlm::TLM_MULTI_INITIATOR_SOCKET;
  }

  //ctor and dtor
  virtual ~multi_init_base(){}
  multi_init_base():base_type(sc_core::sc_gen_unique_name("multi_init_base")){}
  multi_init_base(const char* name):base_type(name){}

private:
  const sc_core::sc_object* get_socket() const { return this; }
};

/*
This class forms the base for multi target sockets,
with fewer template parameters than the multi_target_base.
This class is implementation-defined.
*/
template <typename TYPES = tlm::tlm_base_protocol_types>
class multi_target_base_if {
public:
  //this method shall return a vector of the callback binders of multi initiator socket
  virtual std::vector<callback_binder_fw<TYPES>* >& get_binders()=0;
  
  //this method shall return a map of all multi initiator sockets that are
  // bound to this multi target the key of the map is the index at which the
  // multi initiator i bound, while the value is the interface of the multi
  // initiator socket that is bound at that index
  virtual std::map<unsigned int, tlm::tlm_bw_transport_if<TYPES>*>& get_multi_binds()=0;
protected:
  virtual ~multi_target_base_if() {}
};

/*
This class forms the base for multi target sockets.
It enforces a multi target socket to implement all functions
needed to do hierarchical bindings.
*/
template <unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types,
          unsigned int N=0,
          sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND>
class multi_target_base
  : public tlm::tlm_target_socket<BUSWIDTH, TYPES, N, POL>
  , public multi_target_base_if<TYPES>
  , protected multi_socket_base
{
public:
  //typedef for the base type: the standard tlm target socket
  typedef tlm::tlm_target_socket<BUSWIDTH, TYPES, N, POL > base_type;
  
  //this method shall return the multi_init_base to which the
  // multi_init_base is bound hierarchically
  //  If the base is not bound hierarchically it shall return a pointer to itself                                                
  virtual multi_target_base* get_hierarch_bind()=0;
  
  //this method shall inform the multi target socket that it is bound
  // hierarchically and to which other multi target socket it is bound hierarchically
  virtual void set_hierarch_bind(multi_target_base*)=0;

  virtual tlm::tlm_socket_category get_socket_category() const
  {
    return tlm::TLM_MULTI_TARGET_SOCKET;
  }

  //ctor and dtor
  virtual ~multi_target_base(){}
  multi_target_base():base_type(sc_core::sc_gen_unique_name("multi_target_base")){}
  multi_target_base(const char* name):base_type(name){}

private:
  const sc_core::sc_object* get_socket() const { return this; }
};

/*
All multi sockets must additionally derive from this class.
It enforces a multi socket to implement a function 
needed to do multi init to multi target bindings.
*/
template <typename TYPES>
class multi_to_multi_bind_base{
public:
  virtual ~multi_to_multi_bind_base(){}
  virtual tlm::tlm_fw_transport_if<TYPES>* get_last_binder(tlm::tlm_bw_transport_if<TYPES>*)=0;
};

} // namespace tlm_utils
#endif // TLM_UTILS_MULTI_SOCKET_BASES_H_INCLUDED_
