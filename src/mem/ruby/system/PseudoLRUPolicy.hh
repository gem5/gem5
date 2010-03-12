
#ifndef PSEUDOLRUPOLICY_H
#define PSEUDOLRUPOLICY_H

#include "mem/ruby/system/AbstractReplacementPolicy.hh"

/**
 * Implementation of tree-based pseudo-LRU replacement
 *
 * Works for any associativity between 1 and 128.
 *
 * Also implements associativities that are not a power of 2 by
 * ignoring paths that lead to a larger index (i.e. truncating the
 * tree).  Note that when this occurs, the algorithm becomes less
 * fair, as it will favor indicies in the larger (by index) half of
 * the associative set. This is most unfair when the nearest power of
 * 2 is one below the associativy, and most fair when it is one above.
 */

class PseudoLRUPolicy : public AbstractReplacementPolicy {
 public:

  PseudoLRUPolicy(Index num_sets, Index assoc);
  ~PseudoLRUPolicy();

  void touch(Index set, Index way, Time time);
  Index getVictim(Index set) const;

 private:
  unsigned int m_effective_assoc;    /** nearest (to ceiling) power of 2 */
  unsigned int m_num_levels;         /** number of levels in the tree */
  uint64* m_trees;                   /** bit representation of the trees, one for each set */
};

inline
PseudoLRUPolicy::PseudoLRUPolicy(Index num_sets, Index assoc)
  : AbstractReplacementPolicy(num_sets, assoc)
{
  int num_tree_nodes;

  // associativity cannot exceed capacity of tree representation
  assert(num_sets > 0 && assoc > 1 && assoc <= (Index) sizeof(uint64)*4);

  m_trees = NULL;
  m_num_levels = 0;

  m_effective_assoc = 1;
  while(m_effective_assoc < assoc){
    m_effective_assoc <<= 1;  // effective associativity is ceiling power of 2
  }
  assoc = m_effective_assoc;
  while(true){
    assoc /= 2;
    if(!assoc) break;
    m_num_levels++;
  }
  assert(m_num_levels < sizeof(unsigned int)*4);
  num_tree_nodes = (1 << m_num_levels) - 1;
  m_trees = new uint64[m_num_sets];
  for(unsigned int i=0; i< m_num_sets; i++){
    m_trees[i] = 0;
  }
}

inline
PseudoLRUPolicy::~PseudoLRUPolicy()
{
  if(m_trees != NULL)
    delete[] m_trees;
}

inline
void PseudoLRUPolicy::touch(Index set, Index index, Time time){
  assert(index >= 0 && index < m_assoc);
  assert(set >= 0 && set < m_num_sets);

  int tree_index = 0;
  int node_val;
  for(int i=m_num_levels -1; i>=0; i--){
    node_val = (index >> i)&1;
    if(node_val)
      m_trees[set] |= node_val << tree_index;
    else
      m_trees[set] &= ~(1 << tree_index);
    tree_index = node_val ? (tree_index*2)+2 : (tree_index*2)+1;
  }
  m_last_ref_ptr[set][index] = time;
}

inline
Index PseudoLRUPolicy::getVictim(Index set) const {
  //  assert(m_assoc != 0);

  Index index = 0;

  int tree_index = 0;
  int node_val;
  for(unsigned int i=0;i<m_num_levels;i++){
    node_val = (m_trees[set]>>tree_index)&1;
    index += node_val?0:(m_effective_assoc >> (i+1));
    tree_index = node_val? (tree_index*2)+1 : (tree_index*2)+2;
  }
  assert(index >= 0 && index < m_effective_assoc);

  /* return either the found index or the max possible index */
  /* NOTE: this is not a fair replacement when assoc is not a power of 2 */
  return (index > (m_assoc-1)) ? m_assoc-1:index;
}

#endif // PSEUDOLRUPOLICY_H
