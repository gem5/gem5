/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/* @file
 * Serialization Interface Declarations
 */

#ifndef __SERIALIZE_HH__
#define __SERIALIZE_HH__


#include <list>
#include <iostream>
#include <map>

#include "sim/host.hh"
#include "sim/configfile.hh"

class Serializeable;
class Checkpoint;

template <class T>
void paramOut(std::ostream &os, const std::string &name, const T& param);

template <class T>
void paramIn(Checkpoint *cp, const std::string &section,
             const std::string &name, T& param);

template <class T>
void arrayParamOut(std::ostream &os, const std::string &name,
                   const T *param, int size);

template <class T>
void arrayParamIn(Checkpoint *cp, const std::string &section,
                  const std::string &name, T *param, int size);

void
objParamIn(Checkpoint *cp, const std::string &section,
           const std::string &name, Serializeable * &param);


//
// These macros are streamlined to use in serialize/unserialize
// functions.  It's assumed that serialize() has a parameter 'os' for
// the ostream, and unserialize() has parameters 'cp' and 'section'.
#define SERIALIZE_SCALAR(scalar)	paramOut(os, #scalar, scalar)

#define UNSERIALIZE_SCALAR(scalar)	paramIn(cp, section, #scalar, scalar)

// ENUMs are like SCALARs, but we cast them to ints on the way out
#define SERIALIZE_ENUM(scalar)		paramOut(os, #scalar, (int)scalar)

#define UNSERIALIZE_ENUM(scalar)		\
 do {						\
    int tmp;					\
    paramIn(cp, section, #scalar, tmp);		\
    scalar = (typeof(scalar))tmp;		\
  } while (0)

#define SERIALIZE_ARRAY(member, size)	\
        arrayParamOut(os, #member, member, size)

#define UNSERIALIZE_ARRAY(member, size)	\
        arrayParamIn(cp, section, #member, member, size)

#define SERIALIZE_OBJPTR(objptr)	paramOut(os, #objptr, (objptr)->name())

#define UNSERIALIZE_OBJPTR(objptr)			\
  do {							\
    Serializeable *sptr;				\
    objParamIn(cp, section, #objptr, sptr);		\
    objptr = dynamic_cast<typeof(objptr)>(sptr);	\
  } while (0)

/*
 * Basic support for object serialization.
 */
class Serializeable
{
  public:

    friend class Serializer;

  protected:
    bool serialized;
    static Serializer *serializer;

    void mark();
    void nameOut(std::ostream& os);
    void nameOut(std::ostream& os, const std::string &_name);

  public:
    Serializeable();
    virtual ~Serializeable();

    // manditory virtual function, so objects must provide names
    virtual std::string name() const = 0;

    virtual void serialize(std::ostream& os) {}
    virtual void unserialize(Checkpoint *cp, const std::string &section) {}

    static Serializeable *create(Checkpoint *cp,
                                 const std::string &section);
};

class Serializer
{
    friend class Serializeable;

  protected:
    typedef std::list<Serializeable *> serlist_t;
    serlist_t objects;
    std::string file;
    std::ostream *output;
    std::ostream &out() const;

  public:
    Serializer();
    virtual ~Serializer();

  private:
    void add_object(Serializeable *obj);
    void add_objects();

  public:
    void serialize(const std::string &file);
    const std::string &filename() const { return file; }
};

//
// A SerializeableBuilder serves as an evaluation context for a set of
// parameters that describe a specific instance of a Serializeable.  This
// evaluation context corresponds to a section in the .ini file (as
// with the base ParamContext) plus an optional node in the
// configuration hierarchy (the configNode member) for resolving
// Serializeable references.  SerializeableBuilder is an abstract superclass;
// derived classes specialize the class for particular subclasses of
// Serializeable (e.g., BaseCache).
//
// For typical usage, see the definition of
// SerializeableClass::createObject().
//
class SerializeableBuilder
{
  public:

    SerializeableBuilder() {}

    virtual ~SerializeableBuilder() {}

    // Create the actual Serializeable corresponding to the parameter
    // values in this context.  This function is overridden in derived
    // classes to call a specific constructor for a particular
    // subclass of Serializeable.
    virtual Serializeable *create() = 0;
};

//
// An instance of SerializeableClass corresponds to a class derived from
// Serializeable.  The SerializeableClass instance serves to bind the string
// name (found in the config file) to a function that creates an
// instance of the appropriate derived class.
//
// This would be much cleaner in Smalltalk or Objective-C, where types
// are first-class objects themselves.
//
class SerializeableClass
{
  public:

    // Type CreateFunc is a pointer to a function that creates a new
    // simulation object builder based on a .ini-file parameter
    // section (specified by the first string argument), a unique name
    // for the object (specified by the second string argument), and
    // an optional config hierarchy node (specified by the third
    // argument).  A pointer to the new SerializeableBuilder is returned.
    typedef Serializeable *(*CreateFunc)(Checkpoint *cp,
                                         const std::string &section);

    static std::map<std::string,CreateFunc> *classMap;

    // Constructor.  For example:
    //
    // SerializeableClass baseCacheSerializeableClass("BaseCacheSerializeable",
    //                         newBaseCacheSerializeableBuilder);
    //
    SerializeableClass(const std::string &className, CreateFunc createFunc);

    // create Serializeable given name of class and pointer to
    // configuration hierarchy node
    static Serializeable *createObject(Checkpoint *cp,
                                       const std::string &section);
};

//
// Macros to encapsulate the magic of declaring & defining
// SerializeableBuilder and SerializeableClass objects
//

#define REGISTER_SERIALIZEABLE(CLASS_NAME, OBJ_CLASS)			   \
SerializeableClass the##OBJ_CLASS##Class(CLASS_NAME,			   \
                                         OBJ_CLASS::createForUnserialize);

class Checkpoint
{
  private:

    IniFile *db;
    const std::string basePath;
    const ConfigNode *configNode;
    std::map<std::string, Serializeable*> objMap;

  public:
    Checkpoint(const std::string &filename, const std::string &path,
               const ConfigNode *_configNode);

    bool find(const std::string &section, const std::string &entry,
              std::string &value);

    bool findObj(const std::string &section, const std::string &entry,
                 Serializeable *&value);
};


//
// Export checkpoint filename param so other objects can derive
// filenames from it (e.g., memory).
//
extern std::string serializeFilename;

#endif // __SERIALIZE_HH__
