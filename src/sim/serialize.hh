/*
 * Copyright (c) 2015 ARM Limited
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
 * Authors: Nathan Binkert
 *          Erik Hallnor
 *          Steve Reinhardt
 *          Andreas Sandberg
 */

/* @file
 * Serialization Interface Declarations
 */

#ifndef __SERIALIZE_HH__
#define __SERIALIZE_HH__


#include <iostream>
#include <list>
#include <map>
#include <stack>
#include <vector>

#include "base/bitunion.hh"
#include "base/types.hh"

class IniFile;
class Serializable;
class CheckpointIn;
class SimObject;
class EventQueue;

typedef std::ostream CheckpointOut;


/** The current version of the checkpoint format.
 * This should be incremented by 1 and only 1 for every new version, where a new
 * version is defined as a checkpoint created before this version won't work on
 * the current version until the checkpoint format is updated. Adding a new
 * SimObject shouldn't cause the version number to increase, only changes to
 * existing objects such as serializing/unserializing more state, changing sizes
 * of serialized arrays, etc. */
static const uint64_t gem5CheckpointVersion = 0x000000000000000f;

template <class T>
void paramOut(CheckpointOut &cp, const std::string &name, const T &param);

template <typename DataType, typename BitUnion>
void paramOut(CheckpointOut &cp, const std::string &name,
              const BitfieldBackend::BitUnionOperators<DataType, BitUnion> &p)
{
    paramOut(cp, name, p.__data);
}

template <class T>
void paramIn(CheckpointIn &cp, const std::string &name, T &param);

template <typename DataType, typename BitUnion>
void paramIn(CheckpointIn &cp, const std::string &name,
             BitfieldBackend::BitUnionOperators<DataType, BitUnion> &p)
{
    paramIn(cp, name, p.__data);
}

template <class T>
bool optParamIn(CheckpointIn &cp, const std::string &name, T &param);

template <typename DataType, typename BitUnion>
bool optParamIn(CheckpointIn &cp, const std::string &name,
                BitfieldBackend::BitUnionOperators<DataType, BitUnion> &p)
{
    return optParamIn(cp, name, p.__data);
}

template <class T>
void arrayParamOut(CheckpointOut &cp, const std::string &name,
                   const T *param, unsigned size);

template <class T>
void arrayParamOut(CheckpointOut &cp, const std::string &name,
                   const std::vector<T> &param);

template <class T>
void arrayParamOut(CheckpointOut &cp, const std::string &name,
                   const std::list<T> &param);

template <class T>
void arrayParamIn(CheckpointIn &cp, const std::string &name,
                  T *param, unsigned size);

template <class T>
void arrayParamIn(CheckpointIn &cp, const std::string &name,
                  std::vector<T> &param);

template <class T>
void arrayParamIn(CheckpointIn &cp, const std::string &name,
                  std::list<T> &param);

void
objParamIn(CheckpointIn &cp, const std::string &name, SimObject * &param);

template <typename T>
void fromInt(T &t, int i)
{
    t = (T)i;
}

template <typename T>
void fromSimObject(T &t, SimObject *s)
{
    t = dynamic_cast<T>(s);
}

//
// These macros are streamlined to use in serialize/unserialize
// functions.  It's assumed that serialize() has a parameter 'os' for
// the ostream, and unserialize() has parameters 'cp' and 'section'.
#define SERIALIZE_SCALAR(scalar)        paramOut(cp, #scalar, scalar)

#define UNSERIALIZE_SCALAR(scalar)      paramIn(cp, #scalar, scalar)
#define UNSERIALIZE_OPT_SCALAR(scalar)      optParamIn(cp, #scalar, scalar)

// ENUMs are like SCALARs, but we cast them to ints on the way out
#define SERIALIZE_ENUM(scalar)          paramOut(cp, #scalar, (int)scalar)

#define UNSERIALIZE_ENUM(scalar)                \
 do {                                           \
    int tmp;                                    \
    paramIn(cp, #scalar, tmp);                  \
    fromInt(scalar, tmp);                       \
  } while (0)

#define SERIALIZE_ARRAY(member, size)           \
        arrayParamOut(cp, #member, member, size)

#define UNSERIALIZE_ARRAY(member, size)         \
        arrayParamIn(cp, #member, member, size)

#define SERIALIZE_CONTAINER(member)             \
        arrayParamOut(cp, #member, member)

#define UNSERIALIZE_CONTAINER(member)           \
        arrayParamIn(cp, #member, member)

#define SERIALIZE_EVENT(event) event.serializeSection(cp, #event);

#define UNSERIALIZE_EVENT(event)                        \
    do {                                                \
        event.unserializeSection(cp, #event);           \
        eventQueue()->checkpointReschedule(&event);     \
    } while(0)

#define SERIALIZE_OBJ(obj) obj.serializeSection(cp, #obj)
#define UNSERIALIZE_OBJ(obj) obj.unserializeSection(cp, #obj)

#define SERIALIZE_OBJPTR(objptr)        paramOut(cp, #objptr, (objptr)->name())

#define UNSERIALIZE_OBJPTR(objptr)                      \
  do {                                                  \
    SimObject *sptr;                                    \
    objParamIn(cp, #objptr, sptr);                      \
    fromSimObject(objptr, sptr);                        \
  } while (0)

/**
 * Basic support for object serialization.
 *
 * Objects that support serialization should derive from this
 * class. Such objects can largely be divided into two categories: 1)
 * True SimObjects (deriving from SimObject), and 2) child objects
 * (non-SimObjects).
 *
 * SimObjects are serialized automatically into their own sections
 * automatically by the SimObject base class (see
 * SimObject::serializeAll().
 *
 * SimObjects can contain other serializable objects that are not
 * SimObjects. Much like normal serialized members are not serialized
 * automatically, these objects will not be serialized automatically
 * and it is expected that the objects owning such serializable
 * objects call the required serialization/unserialization methods on
 * child objects. The preferred method to serialize a child object is
 * to call serializeSection() on the child, which serializes the
 * object into a new subsection in the current section. Another option
 * is to call serialize() directly, which serializes the object into
 * the current section. The latter is not recommended as it can lead
 * to naming clashes between objects.
 *
 * @note Many objects that support serialization need to be put in a
 * consistent state when serialization takes place. We refer to the
 * action of forcing an object into a consistent state as
 * 'draining'. Objects that need draining inherit from Drainable. See
 * Drainable for more information.
 */
class Serializable
{
  protected:
    /**
     * Scoped checkpoint section helper class
     *
     * This helper class creates a section within a checkpoint without
     * the need for a separate serializeable object. It is mainly used
     * within the Serializable class when serializing or unserializing
     * section (see serializeSection() and unserializeSection()). It
     * can also be used to maintain backwards compatibility in
     * existing code that serializes structs that are not inheriting
     * from Serializable into subsections.
     *
     * When the class is instantiated, it appends a name to the active
     * path in a checkpoint. The old path is later restored when the
     * instance is destroyed. For example, serializeSection() could be
     * implemented by instantiating a ScopedCheckpointSection and then
     * calling serialize() on an object.
     */
    class ScopedCheckpointSection {
      public:
        template<class CP>
        ScopedCheckpointSection(CP &cp, const char *name) {
            pushName(name);
            nameOut(cp);
        }

        template<class CP>
        ScopedCheckpointSection(CP &cp, const std::string &name) {
            pushName(name.c_str());
            nameOut(cp);
        }

        ~ScopedCheckpointSection();

        ScopedCheckpointSection() = delete;
        ScopedCheckpointSection(const ScopedCheckpointSection &) = delete;
        ScopedCheckpointSection &operator=(
            const ScopedCheckpointSection &) = delete;
        ScopedCheckpointSection &operator=(
            ScopedCheckpointSection &&) = delete;

      private:
        void pushName(const char *name);
        void nameOut(CheckpointOut &cp);
        void nameOut(CheckpointIn &cp) {};
    };

  public:
    Serializable();
    virtual ~Serializable();

    /**
     * Serialize an object
     *
     * Output an object's state into the current checkpoint section.
     *
     * @param cp Checkpoint state
     */
    virtual void serialize(CheckpointOut &cp) const = 0;

    /**
     * Unserialize an object
     *
     * Read an object's state from the current checkpoint section.
     *
     * @param cp Checkpoint state
     */
    virtual void unserialize(CheckpointIn &cp) = 0;

    /**
     * Serialize an object into a new section
     *
     * This method creates a new section in a checkpoint and calls
     * serialize() to serialize the current object into that
     * section. The name of the section is appended to the current
     * checkpoint path.
     *
     * @param cp Checkpoint state
     * @param name Name to append to the active path
     */
    void serializeSection(CheckpointOut &cp, const char *name) const;

    void serializeSection(CheckpointOut &cp, const std::string &name) const {
        serializeSection(cp, name.c_str());
    }

    /**
     * Unserialize an a child object
     *
     * This method loads a child object from a checkpoint. The object
     * name is appended to the active path to form a fully qualified
     * section name and unserialize() is called.
     *
     * @param cp Checkpoint state
     * @param name Name to append to the active path
     */
    void unserializeSection(CheckpointIn &cp, const char *name);

    void unserializeSection(CheckpointIn &cp, const std::string &name) {
        unserializeSection(cp, name.c_str());
    }

    /**
     * @{
     * @name Legacy interface
     *
     * Interface for objects that insist on changing their state when
     * serializing. Such state change should be done in drain(),
     * memWriteback(), or memInvalidate() and not in the serialization
     * method. In general, if state changes occur in serialize, it
     * complicates testing since it breaks assumptions about draining
     * and serialization. It potentially also makes components more
     * fragile since they there are no ordering guarantees when
     * serializing SimObjects.
     *
     * @warn This interface is considered deprecated and should never
     * be used.
     */

    virtual void serializeOld(CheckpointOut &cp) {
        serialize(cp);
    }
    void serializeSectionOld(CheckpointOut &cp, const char *name);
    void serializeSectionOld(CheckpointOut &cp, const std::string &name) {
        serializeSectionOld(cp, name.c_str());
    }
    /** @} */

    /** Get the fully-qualified name of the active section */
    static const std::string &currentSection();

    static Serializable *create(CheckpointIn &cp, const std::string &section);

    static int ckptCount;
    static int ckptMaxCount;
    static int ckptPrevCount;
    static void serializeAll(const std::string &cpt_dir);
    static void unserializeGlobals(CheckpointIn &cp);

  private:
    static std::stack<std::string> path;
};

void debug_serialize(const std::string &cpt_dir);

//
// A SerializableBuilder serves as an evaluation context for a set of
// parameters that describe a specific instance of a Serializable.  This
// evaluation context corresponds to a section in the .ini file (as
// with the base ParamContext) plus an optional node in the
// configuration hierarchy (the configNode member) for resolving
// Serializable references.  SerializableBuilder is an abstract superclass;
// derived classes specialize the class for particular subclasses of
// Serializable (e.g., BaseCache).
//
// For typical usage, see the definition of
// SerializableClass::createObject().
//
class SerializableBuilder
{
  public:

    SerializableBuilder() {}

    virtual ~SerializableBuilder() {}

    // Create the actual Serializable corresponding to the parameter
    // values in this context.  This function is overridden in derived
    // classes to call a specific constructor for a particular
    // subclass of Serializable.
    virtual Serializable *create() = 0;
};

//
// An instance of SerializableClass corresponds to a class derived from
// Serializable.  The SerializableClass instance serves to bind the string
// name (found in the config file) to a function that creates an
// instance of the appropriate derived class.
//
// This would be much cleaner in Smalltalk or Objective-C, where types
// are first-class objects themselves.
//
class SerializableClass
{
  public:

    // Type CreateFunc is a pointer to a function that creates a new
    // simulation object builder based on a .ini-file parameter
    // section (specified by the first string argument), a unique name
    // for the object (specified by the second string argument), and
    // an optional config hierarchy node (specified by the third
    // argument).  A pointer to the new SerializableBuilder is returned.
    typedef Serializable *(*CreateFunc)(CheckpointIn &cp,
                                        const std::string &section);

    static std::map<std::string,CreateFunc> *classMap;

    // Constructor.  For example:
    //
    // SerializableClass baseCacheSerializableClass("BaseCacheSerializable",
    //                         newBaseCacheSerializableBuilder);
    //
    SerializableClass(const std::string &className, CreateFunc createFunc);

    // create Serializable given name of class and pointer to
    // configuration hierarchy node
    static Serializable *createObject(CheckpointIn &cp,
                                      const std::string &section);
};

//
// Macros to encapsulate the magic of declaring & defining
// SerializableBuilder and SerializableClass objects
//

#define REGISTER_SERIALIZEABLE(CLASS_NAME, OBJ_CLASS)                      \
SerializableClass the##OBJ_CLASS##Class(CLASS_NAME,                        \
                                         OBJ_CLASS::createForUnserialize);

// Base class to wrap object resolving functionality.  This can be
// provided to Checkpoint to allow it to map object names onto
// object C++ objects in which to unserialize
class SimObjectResolver
{
  public:
    virtual ~SimObjectResolver() { }

    // Find a SimObject given a full path name
    virtual SimObject *resolveSimObject(const std::string &name) = 0;
};

class CheckpointIn
{
  private:

    IniFile *db;

    SimObjectResolver &objNameResolver;

  public:
    CheckpointIn(const std::string &cpt_dir, SimObjectResolver &resolver);
    ~CheckpointIn();

    const std::string cptDir;

    bool find(const std::string &section, const std::string &entry,
              std::string &value);

    bool findObj(const std::string &section, const std::string &entry,
                 SimObject *&value);

    bool sectionExists(const std::string &section);

    // The following static functions have to do with checkpoint
    // creation rather than restoration.  This class makes a handy
    // namespace for them though.  Currently no Checkpoint object is
    // created on serialization (only unserialization) so we track the
    // directory name as a global.  It would be nice to change this
    // someday

  private:
    // current directory we're serializing into.
    static std::string currentDirectory;

  public:
    // Set the current directory.  This function takes care of
    // inserting curTick() if there's a '%d' in the argument, and
    // appends a '/' if necessary.  The final name is returned.
    static std::string setDir(const std::string &base_name);

    // Export current checkpoint directory name so other objects can
    // derive filenames from it (e.g., memory).  The return value is
    // guaranteed to end in '/' so filenames can be directly appended.
    // This function is only valid while a checkpoint is being created.
    static std::string dir();

    // Filename for base checkpoint file within directory.
    static const char *baseFilename;
};

#endif // __SERIALIZE_HH__
