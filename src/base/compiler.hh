/*
 * Copyright (c) 2012,2017-2018 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __BASE_COMPILER_HH__
#define __BASE_COMPILER_HH__

#include <memory>

#include "config/have_deprecated_namespace.hh"

// http://gcc.gnu.org/onlinedocs/gcc/Function-Attributes.html


/*
 * Compiler specific features.
 */

#if defined(__GNUC__) // clang or gcc.
// Mark a structure as packed, so that no padding is added to its layout. This
// padding might be added to, for instance, ensure certain fields have certain
// alignment.
#  define GEM5_PACKED [[gnu::packed]]

// Prevent a function from being inlined.
#  define GEM5_NO_INLINE [[gnu::noinline]]

// Set the visibility of a symbol.
#  define GEM5_PUBLIC [[gnu:visibility("default")]]
#  define GEM5_LOCAL [[gnu::visibility("hidden")]]
#  define GEM5_WEAK [[gnu::weak]]

// Force an alignment for a variable.
#  define GEM5_ALIGNED(alignment) [[gnu::aligned(alignment)]]

// Marker for what should be an unreachable point in the code.
#  define GEM5_UNREACHABLE __builtin_unreachable()

// To mark a branch condition as likely taken, wrap it's condition with
// GEM5_LIKELY. To mark it as likely not taken, wrap it's condition with
// GEM5_UNLIKELY. These can be replaced with the standard attributes [[likely]]
// and [[unlikely]] in c++20, although the syntax is different enough that
// we can't do that with direct substitution.
#  define GEM5_LIKELY(cond) __builtin_expect(!!(cond), 1)
#  define GEM5_UNLIKELY(cond) __builtin_expect(!!(cond), 0)

// Mark an expression-like macro as deprecated by wrapping it in some code
// which declares and uses a deprecated variable with the same name as the
// macro. The wrapping macro evaluates to the same thing as the original macro.
// The definition must be an c++ expression and not a statement because of how
// the original macro is wrapped.
#  define GEM5_DEPRECATED_MACRO(name, definition, message) \
     ([](){[[deprecated(message)]] int name{}; return name;}(), (definition))
// This version is for macros which are statement-like, which frequently use
// "do {} while (0)" to make their syntax look more like normal c++ statements.
#  define GEM5_DEPRECATED_MACRO_STMT(name, definition, message) \
     do {{definition;} GEM5_DEPRECATED_MACRO(name, ({}), message);} while (0)

// To mark a class as deprecated in favor of a new name, add a respective
// instance of this macro to the file that used to declare the old name.
// This macro should be used *after* the new class has been defined.
#  define GEM5_DEPRECATED_CLASS(old_class, new_class) \
    using old_class \
        [[deprecated("Please use the new class name: '" #new_class "'")]] = \
        new_class

// These macros should be used when namespaces are deprecated in favor of
// a new name. They should be used wherever the namespace is declared.
// Namespace deprecation is broken for GNU < 10 [1], so there is no
// deprecation warning in that case. Clang only supports it from C++17 on.
// [1] https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79817
#  if HAVE_DEPRECATED_NAMESPACE
#    define GEM5_DEPRECATED_NAMESPACE(old_namespace, new_namespace) \
       namespace new_namespace {} \
       namespace [[deprecated("Please use the new namespace: '" \
         #new_namespace "'")]] old_namespace { \
         using namespace new_namespace; \
       }
#  else
#    define GEM5_DEPRECATED_NAMESPACE(old_namespace, new_namespace) \
       namespace new_namespace {} \
       namespace old_namespace = new_namespace
#  endif

// Evaluate an expanded parameter pack in order. Multiple arguments can be
// passed in which be evaluated in order relative to each other as a group.
// The argument(s) must include a parameter pack to expand. This works because
// the elements of a brace inclosed initializer list are evaluated in order,
// as are the arguments to the comma operator, which evaluates to the last
// value. This is compiler specific because it uses variadic macros.
#define GEM5_FOR_EACH_IN_PACK(...) \
do { [[maybe_unused]] int i[] = { 0, ((void)(__VA_ARGS__), 0)... }; } while (0)

#else
#  error "Don't know what to do for your compiler."
#endif

// When a member variable may be unused, mark it with GEM5_CLASS_VAR_USED. This
// needs to be limitted to clang only since clang warns on these unused
// variables, and g++ will actually warn if you use this attribute since it
// won't do anything there.
#if defined(__clang__) // clang only.
#  define GEM5_CLASS_VAR_USED GEM5_VAR_USED
#else
#  define GEM5_CLASS_VAR_USED
#endif

// Aliases for macros using the deprecated M5 prefix.
#define M5_VAR_USED GEM5_VAR_USED
#define M5_NODISCARD GEM5_NO_DISCARD
#define M5_FALLTHROUGH GEM5_FALLTHROUGH
#define M5_ATTR_PACKED GEM5_PACKED
#define M5_NO_INLINE GEM5_NO_INLINE
#define M5_PUBLIC GEM5_PUBLIC
#define M5_LOCAL GEM5_LOCAL
#define M5_WEAK GEM5_WEAK
#define M5_ALIGNED(x) GEM5_ALIGNED(x)
#define M5_UNREACHABLE GEM5_UNREACHABLE
#define M5_LIKELY(x) GEM5_LIKELY(x)
#define M5_UNLIKELY(x) GEM5_UNLIKELY(x)
#define M5_FOR_EACH_IN_PACK(...) GEM5_FOR_EACH_IN_PACK(__VA_ARGS__)
#define M5_CLASS_VAR_USED GEM5_CLASS_VAR_USED

// Deprecated attributes which warn.
#define GEM5_FALLTHROUGH GEM5_DEPRECATED_MACRO_STMT(GEM5_FALLTHROUGH,,\
        "Please use the [[fallthrough]] attribute directly."); [[fallthrough]]
#define GEM5_DEPRECATED(message) \
     [[deprecated(message " The GEM5_DEPRECATED macro is also deprecated, "\
             "please use the [[deprecated()]] attribute directly.")]]
#define GEM5_DEPRECATED_ENUM_VAL(message) \
     [[deprecated(message " The GEM5_DEPRECATED_ENUM_VAL macro is also "\
             "deprecated, please use the [[deprecated()]] attribute "\
             "directly.")]]

// Deprecated attributes which can't be made to warn without possibly breaking
// existing code.
#define GEM5_NO_DISCARD [[nodiscard]]
#define GEM5_VAR_USED [[maybe_unused]]

#endif // __BASE_COMPILER_HH__
