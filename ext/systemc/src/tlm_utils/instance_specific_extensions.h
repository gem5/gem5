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
 
/*
Instance specific extensions, are extension that only a single instance of a module
may access. They are invisible to all other modules; they are private to this
instance so to speak.

As they are only of value to a certain instance, this instance knows very well
when it needs them and when it does not need them any longer (usually when
a transaction passes through a module for the last time).
It does not have to care if anyone else in the system may still have a
reference to the transaction as this one is not able to access the extension
anyway.
Therefore the instance is obliged to call set_extension when it wants to add a
private extension and clear_extension when it does not need it any more.

To get access to an instance specifc extension the module must own a so called
instance_specific_extension_accessor that provides the exclusive access rights.
Assuming the instance_specific_extension_accessor of a given module is called m_accessor
and the transaction of which the private extension is about to be accessed
is called txn, then the calls have to be

m_accessor(txn).set_extension(...);
or
m_accessor(txn).clear_extension(...);

The owner of the private extension is responsible to allocate/deallocate
the extension before/after setting/clearing the extension.
*/
 
#ifndef __INSTANCE_SPECIFIC_EXTENSIONS_H__
#define __INSTANCE_SPECIFIC_EXTENSIONS_H__

#include <tlm>

namespace tlm_utils {

//Helper to do the numbering of private extension accessors
inline unsigned int max_num_ispex_accessors(bool increment=false)
{
    static unsigned int max_num = 0;
    if (increment) ++max_num;
    return max_num;
}

//Helper to do the index generation for private extensions
inline unsigned int max_num_ispex(bool increment=false)
{
    static unsigned int max_num = 0;
    if (increment) ++max_num;
    return max_num;
}

//The private extension base. Similar to normal extension base, but without clone and free
class ispex_base
{
public:
    virtual ~ispex_base() {}
protected:
    static unsigned int register_private_extension()
    {
        return (max_num_ispex(true) - 1);
    };
};

//The templated private extension. Similar to normal extension
template <typename T>
class
instance_specific_extension : public ispex_base{
public:
    virtual ~instance_specific_extension() {}
    const static unsigned int priv_id;
};

template <typename T>
const
unsigned int instance_specific_extension<T>::priv_id = ispex_base::register_private_extension();


//this thing is basically a snippet of the generic_payload
// it contains all the extension specific code (the extension API so to speak)
// the differences are:
// - it calls back to its owner whenever a real (==non-NULL) extension gets set for the first time
// - it calls back to its owner whenever a living (==non-NULL) extension gets cleared
template<typename U>
class instance_specific_extensions_per_accessor{
public:

  typedef void (U::*cb)();

  instance_specific_extensions_per_accessor(U* container, cb inc, cb dec): m_container(container), m_inc(inc), m_dec(dec){
  }

  template <typename T> T* set_extension(T* ext)
  {
      resize_extensions();
      T* tmp = static_cast<T*>(m_extensions[T::priv_id]);
      m_extensions[T::priv_id] = static_cast<ispex_base*>(ext);
      if (!tmp && ext) (m_container->*m_inc)();
      return tmp;
  }
  // non-templatized version with manual index:
  ispex_base* set_extension(unsigned int index,
                                    ispex_base* ext)
  {
      resize_extensions();
      ispex_base* tmp = m_extensions[index];
      m_extensions[index] = ext;
      if (!tmp && ext) (m_container->*m_inc)();
      return tmp;
  }

  // Check for an extension, ext will point to 0 if not present
  template <typename T> void get_extension(T*& ext) const
  {
      ext = static_cast<T*>(m_extensions[T::priv_id]);
  }
  // Non-templatized version:
   ispex_base* get_extension(unsigned int index) const
  {
      return m_extensions[index];
  }

  // Clear extension, the argument is needed to find the right index:
  template <typename T> void clear_extension(const T* ext)
  {
      resize_extensions();
      if (m_extensions[T::priv_id]) (m_container->*m_dec)();
      m_extensions[T::priv_id] = static_cast<ispex_base*>(0);
  }
  // Non-templatized version with manual index
  void clear_extension(unsigned int index)
  {
      if (index < m_extensions.size())
      {
          if (m_extensions[index]) (m_container->*m_dec)();
          m_extensions[index] = static_cast<ispex_base*>(0);
      }
  }

  // Make sure the extension array is large enough. Can be called once by
  // an initiator module (before issuing the first transaction) to make
  // sure that the extension array is of correct size. This is only needed
  // if the initiator cannot guarantee that the generic payload object is
  // allocated after C++ static construction time.
  void resize_extensions()
  {
      m_extensions.expand(max_num_ispex());
  }
  
private:
  tlm::tlm_array<ispex_base*> m_extensions;
  U* m_container;
  cb m_inc, m_dec;
  
};

class instance_specific_extension_container;


//the pool for the container, plain as can be
class instance_specific_extension_container_pool{
  friend class instance_specific_extension_carrier;
  friend class instance_specific_extension_container;
  instance_specific_extension_container_pool() : unused(NULL){}
  inline ~instance_specific_extension_container_pool();
  inline static instance_specific_extension_container_pool& get_ispexcont_pool(){ static instance_specific_extension_container_pool tmp; return tmp;}
  inline instance_specific_extension_container* create();
  inline void free(instance_specific_extension_container*);
  
  instance_specific_extension_container* unused;
};

class instance_specific_extension_carrier;

//this thing contains the vector of extensions per accessor
//which can be really large so this one should be pool allocated
// therefore it keeps a use_count of itself to automatically free itself
// - to this end it provides callbacks to the extensions per accessor
//   to increment and decrement the use_count
class instance_specific_extension_container{
  friend class instance_specific_extension_container_pool;
  friend class instance_specific_extension_accessor;
  friend class instance_specific_extension_carrier;
  
  instance_specific_extension_container(): use_count(0), next(NULL){resize();}
  
  void resize(){
    m_ispex_per_accessor.resize(max_num_ispex_accessors());
    for (unsigned int i=0; i<m_ispex_per_accessor.size(); i++) {
      m_ispex_per_accessor[i]=new instance_specific_extensions_per_accessor<instance_specific_extension_container>(this, 
                                                                                   &instance_specific_extension_container::inc_use_count, 
                                                                                   &instance_specific_extension_container::dec_use_count
                                                                                   );
      m_ispex_per_accessor[i]->resize_extensions();
    }
  }
 
  ~instance_specific_extension_container(){
    for (unsigned int i=0; i<m_ispex_per_accessor.size(); i++) delete m_ispex_per_accessor[i];
  }
 
  void inc_use_count(){use_count++;}
  inline void dec_use_count();
  
  std::vector<instance_specific_extensions_per_accessor<instance_specific_extension_container>* > m_ispex_per_accessor; 
  unsigned int use_count;
  tlm::tlm_generic_payload* my_txn;
  instance_specific_extension_carrier* my_carrier;
  instance_specific_extension_container* next; //for pooling
};


inline instance_specific_extension_container_pool::~instance_specific_extension_container_pool(){
  while(unused) { instance_specific_extension_container* tmp=unused; unused=unused->next; delete tmp;}
}

instance_specific_extension_container* instance_specific_extension_container_pool::create(){
  if (!unused) {unused=new instance_specific_extension_container();}
  instance_specific_extension_container* tmp=unused;
  unused=unused->next;
  return tmp; 
}

void instance_specific_extension_container_pool::free(instance_specific_extension_container* cont){
  cont->next=unused;
  unused=cont;
}

//This is the class that actually sits in the extension array
//we keep this small since that one gets allocated and deallocated all the times
class instance_specific_extension_carrier: public tlm::tlm_extension<instance_specific_extension_carrier>{
  friend class instance_specific_extension_accessor;

public:
  instance_specific_extension_carrier(){
    m_container=instance_specific_extension_container_pool::get_ispexcont_pool().create();
    m_container->my_carrier=this;
  }
  
  virtual tlm::tlm_extension_base* clone() const {
    //we don't clone since private info is instance specific and associated to a given txn (the original)
    //so the deep copied txn will be virgin in terms of private info
    return NULL;
  }
  void copy_from(tlm::tlm_extension_base const &){return;}
  void free(){return;}
private:
  instance_specific_extension_container* m_container;
};

inline void instance_specific_extension_container::dec_use_count(){
  if ((--use_count)==0) { //if this container isn't used any more
    instance_specific_extension_container_pool::get_ispexcont_pool().free(this);  //we send it back to our pool
    //we have to do that manually, as we cannot rely on the fact that there is MM in the txn
    my_txn->clear_extension(my_carrier); //and remove it from the transaction's extension array
    delete my_carrier;
  }
}


//This class 'hides' all the instance specific extension stuff from the user
// he instantiates one of those (e.g. instance_specific_extension_accessor extAcc;) and can then access
// the private extensions
//    extAcc(txn).extensionAPIFnCall()
//  where extensionAPIFnCall is set_extension, get_extension, clear_extension,...
class instance_specific_extension_accessor{
public:
  instance_specific_extension_accessor(): m_index(max_num_ispex_accessors(true)-1){}
  
  template<typename T>
  inline instance_specific_extensions_per_accessor<instance_specific_extension_container>& operator()(T& txn){
    instance_specific_extension_carrier* carrier;
    txn.get_extension(carrier);
    if (!carrier){
      carrier=new instance_specific_extension_carrier();
      carrier->m_container->my_txn=&txn;
      txn.set_extension(carrier);
    }
    return *(carrier->m_container->m_ispex_per_accessor[m_index]);
  }
  
protected:
  unsigned int m_index;
};

}

#endif
