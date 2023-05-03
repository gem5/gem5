#ifndef _ITEMGEN_INCLUDED
#define _ITEMGEN_INCLUDED

#include <iostream>
#include <algorithm>
#include "utils.h"

namespace dataGen {

  using namespace std;

#define HASH_MAX_INT ((unsigned) 1 << 31)

  //#define HASH_MAX_LONG ((unsigned long) 1 << 63)

  template <class T> T hash(intT i);

  template <>
  intT hash<intT>(intT i) {
    return utils::hash(i) & (HASH_MAX_INT-1);}

  template <>
  uintT hash<uintT>(intT i) {
    return utils::hash(i);}

  template <>
  double hash<double>(intT i) {
    return ((double) hash<intT>(i)/((double) HASH_MAX_INT));}

  /* template <class T> T hash(long i); */

  /* template <> */
  /* long hash<long>(long i) { */
  /*   return utils::hash(i) & (HASH_MAX_INT-1);} */

  /* template <> */
  /* int hash<int>(long i) { */
  /*   return utils::hash(i) & (HASH_MAX_INT-1);} */

  /* template <> */
  /* unsigned int hash<unsigned int>(long i) { */
  /*   return utils::hash(i);} */

  /* template <> */
  /* double hash<double>(long i) { */
  /*   return ((double) hash<long>(i)/((double) HASH_MAX_INT));} */

};

#endif // _ITEMGEN_INCLUDED
