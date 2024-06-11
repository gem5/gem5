/*
 * Copyright 2021 Google, Inc.
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

#ifndef __MEM_TRANSLATION_GEN_HH__
#define __MEM_TRANSLATION_GEN_HH__

#include <iterator>
#include <memory>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/request.hh"

namespace gem5
{

class TranslationGenConstIterator;

/**
 * TranslationGen is a base class for a generator object which returns
 * information about address translations over a range of virtual addresses.
 *
 * An object which can perform translations, like an MMU or one representing an
 * in memory page table, can create one of these when asked to translate a
 * range of virtual addresses. The caller can then retrieve translation
 * information from the generator one bit at a time, until the whole range has
 * been translated and the generator has been exhausted.
 *
 * This frees the caller from having to know how far ahead a given translation
 * is good for, or in other words where they need to break up the region to
 * translate its individual parts (page boundaries, etc).
 *
 * This base class manages the bookkeeping during that process. Subclasses are
 * responsible for doing the actual translation, which is requested using the
 * "translate" virtual method.
 */
class TranslationGen
{
  public:
    /**
     * This structure represents a single, contiguous translation, or carries
     * information about whatever fault might have happened while attempting
     * it.
     */
    struct Range
    {
        Addr vaddr;
        Addr size = 0;

        Addr paddr = 0;
        // PTEs can also set the Secure/non-secure bit, so it is stored here.
        Request::Flags flags = 0;
        Fault fault = NoFault;
    };

  protected:
    /**
     * Subclasses implement this function to complete TranslationGen.
     *
     * It should accept a Range reference which will have its "vaddr" field set
     * to the virtual address to translate, and the "size" field set to the
     * remaining size of the entire region being translated.
     *
     * If there is a fault performing the translation of "vaddr", then this
     * function should set the "fault" field of range and return.
     *
     * If the translation was successful, this method should set "paddr" to the
     * corresponding physical address, and set "size" to the number of bytes
     * corresponding to the translation. Or more precisely, size should be set
     * to the maximum "N" where vaddr + n maps to paddr + n for all
     * 0 <= n <= N.
     */
    virtual void translate(Range &range) const = 0;

    Addr _start;
    Addr _size;

  public:
    /**
     * The starting virtual address and the size of the entire region being
     * translated. Subclass constructors can take whatever additional info they
     * may need, like pointers back to the object actually doing the
     * translation.
     */
    TranslationGen(Addr new_start, Addr new_size) :
        _start(new_start), _size(new_size)
    {}
    virtual ~TranslationGen() {}

    Addr start() const { return _start; }
    Addr size() const { return _size; }

    /**
     * A const iterator class is provided so this generator can be used in a
     * range based for loop. When incremented, the iterator will either
     * reattempt a translation that faulted earlier, or if there was no fault,
     * will advance by "size" bytes and perform the translation of the next
     * chunk.
     */
    friend class TranslationGenConstIterator;
    using const_iterator = TranslationGenConstIterator;

    inline const_iterator begin() const;
    inline const_iterator end() const;
};

using TranslationGenPtr = std::unique_ptr<TranslationGen>;

/**
 * An iterator for pulling "Range" instances out of a TranslationGen.
 *
 * Because only one "Range" instance is valid at a time (and is even reused),
 * only the current value of the iterator is valid. When it's incremented, all
 * prior versions of the iterator become invalid.
 *
 * The iterator only supports being incremented and getting the next Range from
 * the generator, and so is marked as a "forward" iterator.
 */
class TranslationGenConstIterator
{
  private:
    TranslationGen::Range current = {0};
    const TranslationGen *gen = nullptr;
    bool end = true;

    friend class TranslationGen;

    /** Use the vaddr of the "current" Range to update its other fields. */
    void
    update()
    {
        current.paddr = 0;
        // Set the size to however much is left, aka the maximum.
        current.size = gen->size() - (current.vaddr - gen->start());

        if (current.size == 0) {
            // Transform into the end iterator.
            end = true;
            current.vaddr = 0;
            gen = nullptr;
        } else {
            gen->translate(current);
        }
    }

    /** Construct a blank iterator, used by end(). */
    TranslationGenConstIterator() {}
    /** Construct a valid new iterator and set it's starting conditions. */
    TranslationGenConstIterator(const TranslationGen *parent, Addr start) :
        current{start}, gen(parent), end(false)
    {
        update();
    }

  public:
    using value_type = TranslationGen::Range;
    using reference = const value_type &;
    using pointer = const value_type *;
    using iterator_category = std::forward_iterator_tag;

    TranslationGenConstIterator(const TranslationGenConstIterator &other) :
        current(other.current), gen(other.gen)
    {}

    TranslationGenConstIterator &
    operator=(const TranslationGenConstIterator &other)
    {
        gen = other.gen;
        current = other.current;
        return *this;
    }

    reference operator*() { return current; }
    pointer operator->() { return &current; }

    /**
     * The increment operator, which is the main work horse of this class.
     *
     * If there was no fault, then the vaddr of the previous translation is
     * incremented by that translation's size.
     *
     * If there was a fault, then the fault is cleared and vaddr is left alone
     * so the translation can be reattempted.
     *
     * The size is then set to however much is left to translate. If that is
     * zero, then this iterator will transform into an end iterator. If not,
     * then the "translate" method of the generator is called to translate (or
     * retranslate) the current Range.
     */
    TranslationGenConstIterator &
    operator++()
    {
        panic_if(end, "Can't increment end iterator.");
        assert(gen);

        if (current.fault == NoFault) {
            // If the last translation was successful, move forward.
            current.vaddr += current.size;
        } else {
            // If not, clear out the fault and try again, assuming the
            // caller has fixed whatever the problem was.
            current.fault = NoFault;
        }

        update();

        return *this;
    }

    TranslationGenConstIterator
    operator++(int)
    {
        const auto orig(*this);
        ++*this;
        return orig;
    }

    bool
    operator==(const TranslationGenConstIterator &other) const
    {
        return other.gen == gen && other.current.vaddr == current.vaddr;
    }

    bool
    operator!=(const TranslationGenConstIterator &other) const
    {
        return !(*this == other);
    }
};

TranslationGenConstIterator
TranslationGen::begin() const
{
    return const_iterator(this, start());
}

TranslationGenConstIterator
TranslationGen::end() const
{
    return const_iterator();
}

} // namespace gem5

#endif // __MEM_TRANSLATION_GEN_HH__
