/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/range.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "sim/config_node.hh"
#include "sim/configfile.hh"
#include "sim/param.hh"
#include "sim/sim_object.hh"

using namespace std;


////////////////////////////////////////////////////////////////////////
//
// BaseParam member definitions
//
////////////////////////////////////////////////////////////////////////

void
BaseParam::die(const string &err) const
{
    context->printErrorProlog(cerr);
    cerr << "  parameter '" << name << "': "
         << err << endl;
    abort();
}


////////////////////////////////////////////////////////////////////////
//
// Param<T> and VectorParam<T> member definitions
//
// We implement parsing & displaying values for various parameter
// types T using a set of overloaded functions:
//
//  - parseParam(string s, T &value) parses s into value
//  - showParam(ostream &os, T &value) displays value on os
//
// By making these independent functions, we can reuse the same code
// for type T in both Param<T> and VectorParam<T>.
//
// For enum types, the parseParam function requires additional
// arguments, in which case we must specialize the Param<T>::parse and
// VectorParam<T>::parse calls as well.
//
// Type-specific instances come first, followed by more generic
// templated versions and their instantiations.
//
////////////////////////////////////////////////////////////////////////

//
// The base implementations use to_number for parsing and '<<' for
// displaying, suitable for integer types.
//
template <class T>
bool
parseParam(const string &s, T &value)
{
    return to_number(s, value);
}

template <class T>
void
showParam(ostream &os, T const &value)
{
    os << value;
}

//
// Template specializations:
// - char (8-bit integer)
// - floating-point types
// - bool
// - string
//

// Treat 8-bit ints (chars) as ints on output, not as chars
template <>
void
showParam(ostream &os, const char &value)
{
    os << (int)value;
}


template <>
void
showParam(ostream &os, const unsigned char &value)
{
    os << (unsigned int)value;
}


// Use sscanf() for FP types as to_number() only handles integers
template <>
bool
parseParam(const string &s, float &value)
{
    return (sscanf(s.c_str(), "%f", &value) == 1);
}

template <>
bool
parseParam(const string &s, double &value)
{
    return (sscanf(s.c_str(), "%lf", &value) == 1);
}

// Be flexible about what we take for bool
template <>
bool
parseParam(const string &s, bool &value)
{
    const string &ls = to_lower(s);

    if (ls == "true" || ls == "t" || ls == "yes" || ls == "y" || ls == "1") {
        value = true;
        return true;
    }

    if (ls == "false" || ls == "f" || ls == "no" || ls == "n" || ls == "0") {
        value = false;
        return true;
    }

    return false;
}

// Display bools as strings
template <>
void
showParam(ostream &os, const bool &value)
{
    os << (value ? "true" : "false");
}


// String requires no processing to speak of
template <>
bool
parseParam(const string &s, string &value)
{
    value = s;
    return true;
}

template <>
bool
parseParam(const string &s, Range<uint32_t> &value)
{
    value = s;
    return value.valid();
}

template <>
bool
parseParam(const string &s, Range<uint64_t> &value)
{
    value = s;
    return value.valid();
}

//
// End of parseParam/showParam definitions.  Now we move on to
// incorporate them into the Param/VectorParam parse() and showValue()
// methods.
//

// These definitions for Param<T>::parse and VectorParam<T>::parse
// work for any type for which parseParam() takes only two arguments
// (i.e., all the fundamental types like int, bool, etc.), thanks to
// overloading.
template <class T>
void
Param<T>::parse(const string &s)
{
    if (parseParam(s, value)) {
        wasSet = true;
    }
    else {
        string err("could not parse \"");

        err += s;
        err += "\"";

        die(err);
    }
}

template <class T>
void
VectorParam<T>::parse(const string &s)
{
    if (s.empty()) {
        wasSet = true;
        return;
    }

    vector<string> tokens;

    tokenize(tokens, s, ' ');

    value.resize(tokens.size());

    for (int i = 0; i < tokens.size(); i++) {
        // need to parse into local variable to handle vector<bool>,
        // for which operator[] returns a special reference class
        // that's not the same as 'bool&', (since it's a packed
        // vector)
        T scalar_value;
        if (!parseParam(tokens[i], scalar_value)) {
            string err("could not parse \"");

            err += s;
            err += "\"";

            die(err);
        }

        // assign parsed value to vector
        value[i] = scalar_value;
    }

    wasSet = true;
}

// These definitions for Param<T>::showValue() and
// VectorParam<T>::showValue() work for any type where showParam()
// takes only two arguments (i.e., everything but the SimpleEnum and
// MappedEnum classes).
template <class T>
void
Param<T>::showValue(ostream &os) const
{
    showParam(os, value);
}

template <class T>
void
VectorParam<T>::showValue(ostream &os) const
{
    for (int i = 0; i < value.size(); i++) {
        if (i != 0) {
            os << " ";
        }
        showParam(os, value[i]);
    }
}


#ifdef INSURE_BUILD
#define INSTANTIATE_PARAM_TEMPLATES(type, typestr)			\
void Param<type>::showType(ostream &os) const { os << typestr; }	\
void VectorParam<type>::showType(ostream &os) const { 			\
    os << "vector of " << typestr; 					\
}									\
template Param<type>;							\
template VectorParam<type>;

#else
// instantiate all four methods (parse/show, scalar/vector) for basic
// types that can use the above templates
#define INSTANTIATE_PARAM_TEMPLATES(type, typestr)			\
template bool parseParam<type>(const string &s, type &value);		\
template void showParam<type>(ostream &os, type const &value);		\
template void Param<type>::parse(const string &);			\
template void VectorParam<type>::parse(const string &);			\
template void Param<type>::showValue(ostream &) const;			\
template void VectorParam<type>::showValue(ostream &) const;		\
void Param<type>::showType(ostream &os) const { os << typestr; }	\
void VectorParam<type>::showType(ostream &os) const {			\
    os << "vector of " << typestr;					\
}
#endif

INSTANTIATE_PARAM_TEMPLATES(unsigned long long, "ull")
INSTANTIATE_PARAM_TEMPLATES(signed long long, "sll")
INSTANTIATE_PARAM_TEMPLATES(unsigned long, "uns long")
INSTANTIATE_PARAM_TEMPLATES(signed long, "long")
INSTANTIATE_PARAM_TEMPLATES(unsigned int, "uns")
INSTANTIATE_PARAM_TEMPLATES(signed int, "int")
INSTANTIATE_PARAM_TEMPLATES(unsigned short, "uns short")
INSTANTIATE_PARAM_TEMPLATES(signed short, "short")
INSTANTIATE_PARAM_TEMPLATES(unsigned char, "uns char")
INSTANTIATE_PARAM_TEMPLATES(signed char, "char")

INSTANTIATE_PARAM_TEMPLATES(float, "float")
INSTANTIATE_PARAM_TEMPLATES(double, "double")

INSTANTIATE_PARAM_TEMPLATES(bool, "bool")
INSTANTIATE_PARAM_TEMPLATES(string, "string")

INSTANTIATE_PARAM_TEMPLATES(Range<uint64_t>, "uint64 range")
INSTANTIATE_PARAM_TEMPLATES(Range<uint32_t>, "uint32 range")

#undef INSTANTIATE_PARAM_TEMPLATES

//
// SimpleEnumParam & MappedEnumParam must specialize their parse(),
// showValue(), and showType() methods.
//

//
// SimpleEnumParam & SimpleEnumVectorParam
//
bool
parseEnumParam(const char *const *map, const int num_values,
               const string &s, int &value)
{
    for (int i = 0; i < num_values; ++i) {
        if (s == map[i]) {
            value = i;
            return true;
        }
    }

    return false;
}

void
showEnumParam(ostream &os,
              const char *const *map,  const int num_values,
              int value)
{
    assert(0 <= value && value < num_values);
    os << map[value];
}

void
showEnumType(ostream &os,
             const char *const *map,  const int num_values)
{
    os << "{" << map[0];
    for (int i = 1; i < num_values; ++i)
        os << "," << map[i];

    os << "}";
}


//
// MappedEnumParam & MappedEnumVectorParam
//
bool
parseEnumParam(const EnumParamMap *map, const int num_values,
               const string &s, int &value)
{
    for (int i = 0; i < num_values; ++i) {
        if (s == map[i].name) {
            value = map[i].value;
            return true;
        }
    }

    return false;
}

void
showEnumParam(ostream &os,
              const EnumParamMap *map, const int num_values,
              int value)
{
    for (int i = 0; i < num_values; ++i) {
        if (value == map[i].value) {
            os << map[i].name;
            return;
        }
    }

    // if we can't find a reverse mapping just print the int value
    os << value;
}

void
showEnumType(ostream &os,
             const EnumParamMap *map,  const int num_values)
{
    os << "{" << map[0].name;
    for (int i = 1; i < num_values; ++i)
        os << "," << map[i].name;

    os << "}";
}


template <class Map>
void
EnumParam<Map>::parse(const string &s)
{
    if (parseEnumParam(map, num_values, s, value)) {
        wasSet = true;
    } else {
        string err("no match for enum string \"");

        err += s;
        err += "\"";

        die(err);
    }
}

template <class Map>
void
EnumVectorParam<Map>::parse(const string &s)
{
    vector<string> tokens;

    tokenize(tokens, s, ' ');

    value.resize(tokens.size());

    for (int i = 0; i < tokens.size(); i++) {
        if (!parseEnumParam(map, num_values, tokens[i], value[i])) {
            string err("no match for enum string \"");

            err += s;
            err += "\"";

            die(err);
        }
    }

    wasSet = true;
}

template <class Map>
void
EnumParam<Map>::showValue(ostream &os) const
{
    showEnumParam(os, map, num_values, value);
}

template <class Map>
void
EnumVectorParam<Map>::showValue(ostream &os) const
{
    for (int i = 0; i < value.size(); i++) {
        if (i != 0) {
            os << " ";
        }
        showEnumParam(os, map, num_values, value[i]);
    }
}

template <class Map>
void
EnumParam<Map>::showType(ostream &os) const
{
    showEnumType(os, map, num_values);
}

template <class Map>
void
EnumVectorParam<Map>::showType(ostream &os) const
{
    os << "vector of";
    showEnumType(os, map, num_values);
}

template class EnumParam<const char *>;
template class EnumVectorParam<const char *>;

template class EnumParam<EnumParamMap>;
template class EnumVectorParam<EnumParamMap>;

////////////////////////////////////////////////////////////////////////
//
// SimObjectBaseParam methods
//
////////////////////////////////////////////////////////////////////////

bool
parseSimObjectParam(ParamContext *context, const string &s, SimObject *&value)
{
    SimObject *obj;

    if (to_lower(s) == "null") {
        // explicitly set to null by user; assume that's OK
        obj = NULL;
    }
    else {
        obj = context->resolveSimObject(s);

        if (obj == NULL)
            return false;
    }

    value = obj;
    return true;
}


void
SimObjectBaseParam::showValue(ostream &os, SimObject *value) const
{
    os << (value ? value->name() : "null");
}

void
SimObjectBaseParam::parse(const string &s, SimObject *&value)
{
    if (parseSimObjectParam(context, s, value)) {
        wasSet = true;
    }
    else {
        string err("could not resolve object name \"");

        err += s;
        err += "\"";

        die(err);
    }
}

void
SimObjectBaseParam::parse(const string &s, vector<SimObject *>&value)
{
    vector<string> tokens;

    tokenize(tokens, s, ' ');

    value.resize(tokens.size());

    for (int i = 0; i < tokens.size(); i++) {
        if (!parseSimObjectParam(context, tokens[i], value[i])) {
            string err("could not resolve object name \"");

            err += s;
            err += "\"";

            die(err);
        }
    }

    wasSet = true;
}

////////////////////////////////////////////////////////////////////////
//
// ParamContext member definitions
//
////////////////////////////////////////////////////////////////////////

list<ParamContext *> *ParamContext::ctxList = NULL;

ParamContext::ParamContext(const string &_iniSection, InitPhase _initPhase)
    : iniFilePtr(NULL),	// initialized on call to parseParams()
      iniSection(_iniSection), paramList(NULL),
      initPhase(_initPhase)
{
    // Put this context on global list for initialization
    if (initPhase != NoAutoInit) {
        if (ctxList == NULL)
            ctxList = new list<ParamContext *>();

        // keep list sorted by ascending initPhase values
        list<ParamContext *>::iterator i = ctxList->begin();
        list<ParamContext *>::iterator end = ctxList->end();
        for (; i != end; ++i) {
            if (initPhase <= (*i)->initPhase) {
                // found where we want to insert
                break;
            }
        }
        // (fall through case: insert at end)
        ctxList->insert(i, this);
    }
}


void
ParamContext::addParam(BaseParam *param)
{
    getParamList()->push_back(param);
}


void
ParamContext::parseParams(IniFile &iniFile)
{
    iniFilePtr = &iniFile;	// set object member

    ParamList::iterator i;

    for (i = getParamList()->begin(); i != getParamList()->end(); ++i) {
        string string_value;

        if (iniFile.find(iniSection, (*i)->name, string_value))
            (*i)->parse(string_value);
    }
}


// Check parameter values for validity & consistency. Default
// implementation is no-op; derive subclass & override to add
// actual functionality here.
void
ParamContext::checkParams()
{
    // nada
}


// Clean up context-related objects at end of execution. Default
// implementation is no-op; derive subclass & override to add actual
// functionality here.
void
ParamContext::cleanup()
{
    // nada
}


void
ParamContext::describeParams(ostream &os)
{
    ParamList::iterator i;

    for (i = getParamList()->begin(); i != getParamList()->end(); ++i) {
        BaseParam *p = *i;

        os << p->name << " (";
        p->showType(os);
        os << "): " << p->description << "\n";
    }
}



void
ParamContext::showParams(ostream &os)
{
    ParamList::iterator i;

    for (i = getParamList()->begin(); i != getParamList()->end(); ++i) {
        BaseParam *p = *i;

        if (p->isValid()) {
            os << p->name << "=";
            p->showValue(os);
            os << endl;
        }
        else {
            os << "// "<< p->name << " not specified" << endl;
        }
    }
}


void
ParamContext::printErrorProlog(ostream &os)
{
    os << "Parameter error in section [" << iniSection << "]: " << endl;
}

//
// Resolve an object name to a SimObject pointer.  The object will be
// created as a side-effect if necessary.  If the name contains a
// colon (e.g., "iq:IQ"), then the object is local (invisible to
// outside this context).  If there is no colon, the name needs to be
// resolved through the configuration hierarchy (only possible for
// SimObjectBuilder objects, which return non-NULL for configNode()).
//
SimObject *
ParamContext::resolveSimObject(const string &name)
{
    ConfigNode *n = getConfigNode();
    return n ? n->resolveSimObject(name) : NULL;
}


//
// static method: call parseParams() on all registered contexts
//
void
ParamContext::parseAllContexts(IniFile &iniFile)
{
    list<ParamContext *>::iterator iter;

    for (iter = ctxList->begin(); iter != ctxList->end(); ++iter) {
        ParamContext *pc = *iter;

        pc->parseParams(iniFile);
    }
}


//
// static method: call checkParams() on all registered contexts
//
void
ParamContext::checkAllContexts()
{
    list<ParamContext *>::iterator iter;

    for (iter = ctxList->begin(); iter != ctxList->end(); ++iter) {
        ParamContext *pc = *iter;

        pc->checkParams();
    }
}


//
// static method: call showParams() on all registered contexts
//
void
ParamContext::showAllContexts(ostream &os)
{
    list<ParamContext *>::iterator iter;

    for (iter = ctxList->begin(); iter != ctxList->end(); ++iter) {
        ParamContext *pc = *iter;

        os << "[" << pc->iniSection << "]" << endl;
        pc->showParams(os);
        os << endl;
    }
}


//
// static method: call cleanup() on all registered contexts
//
void
ParamContext::cleanupAllContexts()
{
    list<ParamContext *>::iterator iter;

    for (iter = ctxList->begin(); iter != ctxList->end(); ++iter) {
        ParamContext *pc = *iter;

        pc->cleanup();
    }
}


//
// static method: call describeParams() on all registered contexts
//
void
ParamContext::describeAllContexts(ostream &os)
{
    list<ParamContext *>::iterator iter;

    for (iter = ctxList->begin(); iter != ctxList->end(); ++iter) {
        ParamContext *pc = *iter;

        os << "[" << pc->iniSection << "]\n";
        pc->describeParams(os);
        os << endl;
    }
}
