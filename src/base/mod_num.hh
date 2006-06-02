/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Steve Raasch
 */

template<class T, T MV>
class ModNum {
  private:
    T value;

    //  Compiler should optimize this
    void setValue(T n) { value = n % MV; }

  public:
    ModNum() {}
    ModNum(T n) { setValue(n); }
    ModNum(const ModNum<T, MV> &n) : value(n.value) {}

    ModNum operator=(T n) {
        setValue(n);
        return *this;
    }

    const ModNum operator=(ModNum n) {
        value = n.value;
        return *this;
    }

    //  Return the value if object used as RHS
    operator T() const { return value; }

    //
    //  Operator "+="
    //
    const ModNum<T, MV> operator+=(ModNum<T, MV> r) {
        setValue(value + r.value);
        return *this;
    }

    const ModNum<T, MV> operator+=(T r) {
        setValue(value + r);
        return *this;
    }

    //
    //  Operator "-="
    //
    const ModNum<T, MV> operator-=(ModNum<T, MV> r) {
        setValue(value - r.value);
        return *this;
    }

    const ModNum<T, MV> operator-=(T r) {
        setValue(value - r);
        return *this;
    }

    //
    //  Operator "++"
    //
    //  PREFIX (like ++a)
    const ModNum<T, MV> operator++() {
        *this += 1;
        return *this;
    }

    //  POSTFIX (like a++)
    const ModNum<T, MV> operator++(int) {
        ModNum<T, MV> rv = *this;

        *this += 1;

        return rv;
    }

    //
    //  Operator "--"
    //
    //  PREFIX (like --a)
    const ModNum<T, MV> operator--() {
        *this -= 1;
        return *this;
    }

    //  POSTFIX (like a--)
    const ModNum<T, MV> operator--(int) {
        ModNum<T, MV> rv = *this;
        *this -= 1;
        return rv;
    }
};


//
//  Define operator "+" like this to avoid creating a temporary
//
template<class T, T MV>
inline ModNum<T, MV>
operator+(ModNum<T, MV> l, ModNum<T, MV> r) {
    l += r;
    return l;
}

template<class T, T MV>
inline ModNum<T, MV>
operator+(ModNum<T, MV> l, T r) {
    l += r;
    return l;
}

template<class T, T MV>
inline ModNum<T, MV>
operator+(T l, ModNum<T, MV> r) {
    r += l;
    return r;
}


//
//  Define operator "-" like this to avoid creating a temporary
//
template<class T, T MV>
inline ModNum<T, MV>
operator-(ModNum<T, MV> l, ModNum<T, MV> r) {
    l -= r;
    return l;
}

template<class T, T MV>
inline ModNum<T, MV>
operator-(ModNum<T, MV> l, T r) {
    l -= r;
    return l;
}

template<class T, T MV>
inline ModNum<T, MV>
operator-(T l, ModNum<T, MV> r) {
    r -= l;
    return r;
}


//
//  Comparison operators
//  (all other cases are handled with conversons)
//
template<class T, T MV>
inline bool
operator<(ModNum<T, MV> l, ModNum<T, MV> r) {
    return l.value < r.value;
}

template<class T, T MV>
inline bool
operator>(ModNum<T, MV> l, ModNum<T, MV> r) {
    return l.value > r.value;
}

template<class T, T MV>
inline bool
operator==(ModNum<T, MV> l, ModNum<T, MV> r) {
    return l.value == r.value;
}

template<class T, T MV>
inline bool
operator<=(ModNum<T, MV> l, ModNum<T, MV> r) {
    return l.value <= r.value;
}

template<class T, T MV>
inline bool
operator>=(ModNum<T, MV> l, ModNum<T, MV> r) {
    return l.value >= r.value;
}


