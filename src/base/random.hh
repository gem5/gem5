/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Mersenne twister random number generator.
 */

#ifndef __BASE_RANDOM_HH__
#define __BASE_RANDOM_HH__

#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

class Random
{
  friend class RandomTest;

  public:
    using RandomPtr = std::shared_ptr<Random>;
    using Instances = std::vector<std::weak_ptr<Random>>;

    static RandomPtr genRandom(Random* r = nullptr)
    {
        if (instances == nullptr)
            instances = new Instances();

        /*
        * If `r` is null we create a new RNG with the global seed and wrap
        * it in a `shared_ptr`. Otherwise, we wrap the existing RNG in a
        * `shared_ptr` and add it to the vector of instances.
        *
        * This is done to ensure that the RNG is not deleted while it is still
        * in use. The `shared_ptr` will keep the RNG alive until all references
        * to it are gone.
        */
        auto randpoint = r ? std::shared_ptr<Random>(r, [](Random*){})
                         : std::shared_ptr<Random>(new Random(globalSeed));
        // Check if randpoint is not already in the vector
        bool exists = false;
        for (const auto& weak_ptr : *instances) {
            if (!weak_ptr.expired() && weak_ptr.lock() == randpoint) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            instances->emplace_back(randpoint);
        }

        return randpoint;
    }

    static RandomPtr genRandom(uint32_t s)
    {
        if (instances == nullptr)
            instances = new Instances();

        auto randpoint = std::shared_ptr<Random>(new Random(s));
        // Check if randpoint is not already in the vector
        bool exists = false;
        for (const auto& weak_ptr : *instances) {
            if (!weak_ptr.expired() && weak_ptr.lock() == randpoint) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            instances->emplace_back(randpoint);
        }

        return randpoint;
    }

    static uint64_t globalSeed;

    /**
     * @ingroup api_base_utils
     */
    std::mt19937_64 gen;

  private:
    /**
     * Collection of all live instances
     * of Random to enable global
     * reseeding. We use a pointer
     * because the loader will initialize
     * it to 0x0 (it is in .bss), allowing us to avoid
     * Static Initialization Order Fiasco
     * if static Random instances are inialized
     * before the vector by having the constructors
     * of Random allocate memory for the pointer.
     * This requires that nullptr matches how
     * the loader initializes memory
     */
    static_assert(nullptr == 0x0, "nullptr is not 0x0, Random instance tracking will fail");
    static Instances* instances;

    /**
     * @ingroup api_base_utils
     * @{
     */
    Random() = delete;
    Random(uint32_t s);

    Random(const Random& rng) = delete;
    Random& operator=(const Random& rng) = delete;

    Random(Random&& rng) = delete;
    Random& operator=(Random&& rng) = delete;

  public:
    /** @} */ // end of api_base_utils
    ~Random();

    void init(uint32_t s);

    /**
     * Facility to reseed all live instances
     * and ensure future default constructed
     * instances also use the new see
     */
    static void reseedAll(uint64_t seed)
    {
        globalSeed = seed;

        if (instances == nullptr)
          return;

        for (auto rng_ptr : *instances)
            rng_ptr.lock()->init(seed);
    }

    /**
     * Use the SFINAE idiom to choose an implementation based on
     * whether the type is integral or floating point.
     *
     * @ingroup api_base_utils
     */
    template <typename T>
    typename std::enable_if_t<std::is_integral_v<T>, T>
    random()
    {
        // [0, max_value] for integer types
        return gen() % std::numeric_limits<T>::max();
    }

    /**
     * @ingroup api_base_utils
     */
    template <typename T>
    typename std::enable_if_t<std::is_floating_point_v<T>, T>
    random()
    {
        // [0, 1) for real types
        warn_once("FP random numbers are not uniformly distributed.");
        return ((T) gen()) /
          ((T) std::numeric_limits<uint64_t>::max());
    }

    /**
     * @ingroup api_base_utils
     */
    template <typename T>
    typename std::enable_if_t<std::is_integral_v<T>, T>
    random(T min, T max)
    {
        assert(min <= max);
        // + 1 to handle cases where min == max
        T r = gen() % (max - min + 1) + min;
        return r;
    }
};

} // namespace gem5

#endif // __BASE_RANDOM_HH__
