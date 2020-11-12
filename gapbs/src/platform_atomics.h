// Copyright (c) 2015, The Regents of the University of California (Regents)
// See LICENSE.txt for license details

#ifndef PLATFORM_ATOMICS_H_
#define PLATFORM_ATOMICS_H_


/*
GAP Benchmark Suite
File:   Platform Atomics
Author: Scott Beamer

Wrappers for compiler intrinsics for atomic memory operations (AMOs)
 - If not using OpenMP (serial), provides serial fallbacks
*/


#if defined _OPENMP

  #if defined __GNUC__

    // gcc/clang/icc instrinsics

    template<typename T, typename U>
    T fetch_and_add(T &x, U inc) {
      return __sync_fetch_and_add(&x, inc);
    }

    template<typename T>
    bool compare_and_swap(T &x, const T &old_val, const T &new_val) {
      return __sync_bool_compare_and_swap(&x, old_val, new_val);
    }

    template<>
    bool compare_and_swap(float &x, const float &old_val, const float &new_val) {
      return __sync_bool_compare_and_swap(reinterpret_cast<uint32_t*>(&x),
                                          reinterpret_cast<const uint32_t&>(old_val),
                                          reinterpret_cast<const uint32_t&>(new_val));
    }

    template<>
    bool compare_and_swap(double &x, const double &old_val, const double &new_val) {
      return __sync_bool_compare_and_swap(reinterpret_cast<uint64_t*>(&x),
                                          reinterpret_cast<const uint64_t&>(old_val),
                                          reinterpret_cast<const uint64_t&>(new_val));
    }

  #elif __SUNPRO_CC

    // sunCC (solaris sun studio) intrinsics
    // less general, only work for int32_t, int64_t, uint32_t, uint64_t
    // http://docs.oracle.com/cd/E19253-01/816-5168/6mbb3hr06/index.html

    #include <atomic.h>
    #include <cinttypes>

    int32_t fetch_and_add(int32_t &x, int32_t inc) {
      return atomic_add_32_nv((volatile uint32_t*) &x, inc) - inc;
    }

    int64_t fetch_and_add(int64_t &x, int64_t inc) {
      return atomic_add_64_nv((volatile uint64_t*) &x, inc) - inc;
    }

    uint32_t fetch_and_add(uint32_t &x, uint32_t inc) {
      return atomic_add_32_nv((volatile uint32_t*) &x, inc) - inc;
    }

    uint64_t fetch_and_add(uint64_t &x, uint64_t inc) {
      return atomic_add_64_nv((volatile uint64_t*) &x, inc) - inc;
    }

    bool compare_and_swap(int32_t &x, const int32_t &old_val, const int32_t &new_val) {
      return old_val == atomic_cas_32((volatile uint32_t*) &x, old_val, new_val);
    }

    bool compare_and_swap(int64_t &x, const int64_t &old_val, const int64_t &new_val) {
      return old_val == atomic_cas_64((volatile uint64_t*) &x, old_val, new_val);
    }

    bool compare_and_swap(uint32_t &x, const uint32_t &old_val, const uint32_t &new_val) {
      return old_val == atomic_cas_32((volatile uint32_t*) &x, old_val, new_val);
    }

    bool compare_and_swap(uint64_t &x, const uint64_t &old_val, const uint64_t &new_val) {
      return old_val == atomic_cas_64((volatile uint64_t*) &x, old_val, new_val);
    }

    bool compare_and_swap(float &x, const float &old_val, const float &new_val) {
      return old_val == atomic_cas_32((volatile uint32_t*) &x,
                                      (const volatile uint32_t&) old_val,
                                      (const volatile uint32_t&) new_val);
    }

    bool compare_and_swap(double &x, const double &old_val, const double &new_val) {
      return old_val == atomic_cas_64((volatile uint64_t*) &x,
                                      (const volatile uint64_t&) old_val,
                                      (const volatile uint64_t&) new_val);
    }

  #else   // defined __GNUC__ __SUNPRO_CC

    #error No atomics available for this compiler but using OpenMP

  #endif  // else defined __GNUC__ __SUNPRO_CC

#else   // defined _OPENMP

  // serial fallbacks

  template<typename T, typename U>
  T fetch_and_add(T &x, U inc) {
    T orig_val = x;
    x += inc;
    return orig_val;
  }

  template<typename T>
  bool compare_and_swap(T &x, const T &old_val, const T &new_val) {
    if (x == old_val) {
      x = new_val;
      return true;
    }
    return false;
  }

#endif  // else defined _OPENMP

#endif  // PLATFORM_ATOMICS_H_
