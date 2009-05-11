
#ifndef ABSTRACTREPLACEMENTPOLICY_H
#define ABSTRACTREPLACEMENTPOLICY_H

#include "mem/ruby/common/Global.hh"

class AbstractReplacementPolicy {

public:

  AbstractReplacementPolicy(Index num_sets, Index assoc);
  virtual ~AbstractReplacementPolicy();

  /* touch a block. a.k.a. update timestamp */
  virtual void touch(Index set, Index way, Time time) = 0;

  /* returns the way to replace */
  virtual Index getVictim(Index set) const = 0;

  /* get the time of the last access */
  Time getLastAccess(Index set, Index way);

 protected:
  unsigned int m_num_sets;       /** total number of sets */
  unsigned int m_assoc;          /** set associativity */
  Time **m_last_ref_ptr;         /** timestamp of last reference */
};

inline
AbstractReplacementPolicy::AbstractReplacementPolicy(Index num_sets, Index assoc)
{
  m_num_sets = num_sets;
  m_assoc = assoc;
  m_last_ref_ptr = new Time*[m_num_sets];
  for(unsigned int i = 0; i < m_num_sets; i++){
    m_last_ref_ptr[i] = new Time[m_assoc];
    for(unsigned int j = 0; j < m_assoc; j++){
      m_last_ref_ptr[i][j] = 0;
    }
  }
}

inline
AbstractReplacementPolicy::~AbstractReplacementPolicy()
{
  if(m_last_ref_ptr != NULL){
    for(unsigned int i = 0; i < m_num_sets; i++){
      if(m_last_ref_ptr[i] != NULL){
        delete[] m_last_ref_ptr[i];
      }
    }
    delete[] m_last_ref_ptr;
  }
}

inline
Time AbstractReplacementPolicy::getLastAccess(Index set, Index way)
{
  return m_last_ref_ptr[set][way];
}

#endif // ABSTRACTREPLACEMENTPOLICY_H
