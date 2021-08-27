/*
 * Copyright (c) 2015, 2018, 2020 ARM Limited
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
 */

/* @file
 * Serialization Interface Declarations
 */

#ifndef __SERIALIZE_HH__
#define __SERIALIZE_HH__


#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stack>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "base/inifile.hh"
#include "base/logging.hh"
#include "sim/serialize_handlers.hh"

namespace gem5
{

typedef std::ostream CheckpointOut;

class CheckpointIn
{
  private:
    IniFile db;

    const std::string _cptDir;

  public:
    CheckpointIn(const std::string &cpt_dir);
    ~CheckpointIn() = default;

    /**
     * @return Returns the current directory being used for creating
     * checkpoints or restoring checkpoints.
     * @ingroup api_serialize
     * @{
     */
    const std::string getCptDir() { return _cptDir; }

    bool find(const std::string &section, const std::string &entry,
              std::string &value);

    bool entryExists(const std::string &section, const std::string &entry);
    bool sectionExists(const std::string &section);
    void visitSection(const std::string &section,
        IniFile::VisitSectionCallback cb);
    /** @}*/ //end of api_checkout group

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
    /**
     * Set the current directory
     *
     * This function takes care of inserting curTick() if there's a '%d' in the
     * argument, and appends a '/' if necessary. The final name is returned.
     *
     * @ingroup api_serialize
     */
    static std::string setDir(const std::string &base_name);

    /**
     * Get the current checkout directory name
     *
     * This function exports the current checkout point directory name so other
     * objects can derive filenames from it (e.g., memory). The return value is
     * guaranteed to end in '/' so filenames can be directly appended. This
     * function is only valid while a checkpoint is being created.
     *
     * @ingroup api_serialize
     */
    static std::string dir();

    // Filename for base checkpoint file within directory.
    static const char *baseFilename;
};

/**
 * Basic support for object serialization.
 *
 * The Serailizable interface is used to create checkpoints. Any
 * object that implements this interface can be included in
 * gem5's checkpointing system.
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
  public:
    class ScopedCheckpointSection
    {
      public:
        /**
         * This is the constructor for Scoped checkpoint section helper
         * class.
         *
         * Scoped checkpoint helper class creates a section within a
         * checkpoint without the need for a separate serializeable
         * object. It is mainly used within the Serializable class
         * when serializing or unserializing section (see
         * serializeSection() and unserializeSection()). It
         * can also be used to maintain backwards compatibility in
         * existing code that serializes structs that are not inheriting
         * from Serializable into subsections.
         *
         * When the class is instantiated, it appends a name to the active
         * path in a checkpoint. The old path is later restored when the
         * instance is destroyed. For example, serializeSection() could be
         * implemented by instantiating a ScopedCheckpointSection and then
         * calling serialize() on an object.
         *
         * @ingroup api_serialize
         * @{
         */
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
        /** @}*/ //end of api_serialize group

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

    /**
     * @ingroup api_serialize
     */
    Serializable();
    virtual ~Serializable();

    /**
     * Serialize an object
     *
     * Output an object's state into the current checkpoint section.
     *
     * @param cp Checkpoint state
     *
     * @ingroup api_serialize
     */
    virtual void serialize(CheckpointOut &cp) const = 0;

    /**
     * Unserialize an object
     *
     * Read an object's state from the current checkpoint section.
     *
     * @param cp Checkpoint state
     *
     * @ingroup api_serialize
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
     *
     * @ingroup api_serialize
     */
    void serializeSection(CheckpointOut &cp, const char *name) const;

    /**
     * @ingroup api_serialize
     */
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
     *
     * @ingroup api_serialize
     */
    void unserializeSection(CheckpointIn &cp, const char *name);

    /**
     * @ingroup api_serialize
     */
    void unserializeSection(CheckpointIn &cp, const std::string &name) {
        unserializeSection(cp, name.c_str());
    }

    /**
     * Gets the fully-qualified name of the active section
     *
     * @ingroup api_serialize
     */
    static const std::string &currentSection();

    /**
     * Generate a checkpoint file so that the serialization can be routed to
     * it.
     *
     * @param cpt_dir The dir at which the cpt file will be created.
     * @param outstream The cpt file.
     * @ingroup api_serialize
     */
    static void generateCheckpointOut(const std::string &cpt_dir,
        std::ofstream &outstream);

  private:
    static std::stack<std::string> path;
};

/**
 * This function is used for writing parameters to a checkpoint.
 * @param os The checkpoint to be written to.
 * @param name Name of the parameter to be set.
 * @param param Value of the parameter to be written.
 * @ingroup api_serialize
 */
template <class T>
void
paramOut(CheckpointOut &os, const std::string &name, const T &param)
{
    os << name << "=";
    ShowParam<T>::show(os, param);
    os << "\n";
}

template <class T>
bool
paramInImpl(CheckpointIn &cp, const std::string &name, T &param)
{
    const std::string &section(Serializable::currentSection());
    std::string str;
    return cp.find(section, name, str) && ParseParam<T>::parse(str, param);
}

/**
 * This function is used for restoring optional parameters from the
 * checkpoint.
 * @param cp The checkpoint to be read from.
 * @param name Name of the parameter to be read.
 * @param param Value of the parameter to be read.
 * @param do_warn If the warn is set to true then the function prints the
 * warning message.
 * @return Returns if the parameter existed in the checkpoint.
 *
 * @ingroup api_serialize
 */
template <class T>
bool
optParamIn(CheckpointIn &cp, const std::string &name, T &param,
           bool do_warn=true)
{
    if (paramInImpl(cp, name, param))
        return true;

    warn_if(do_warn, "optional parameter %s:%s not present",
            Serializable::currentSection(), name);
    return false;
}

/**
 * This function is used for restoring parameters from a checkpoint.
 * @param os The checkpoint to be restored from.
 * @param name Name of the parameter to be set.
 * @param param Value of the parameter to be restored.
 * @ingroup api_serialize
 */
template <class T>
void
paramIn(CheckpointIn &cp, const std::string &name, T &param)
{
    fatal_if(!paramInImpl(cp, name, param),
        "Can't unserialize '%s:%s'", Serializable::currentSection(), name);
}

/**
 * @ingroup api_serialize
 */
template <class InputIterator>
void
arrayParamOut(CheckpointOut &os, const std::string &name,
              InputIterator start, InputIterator end)
{
    os << name << "=";
    auto it = start;
    using Elem = std::remove_cv_t<std::remove_reference_t<decltype(*it)>>;
    if (it != end)
        ShowParam<Elem>::show(os, *it++);
    while (it != end) {
        os << " ";
        ShowParam<Elem>::show(os, *it++);
    }
    os << "\n";
}

/**
 * @ingroup api_serialize
 */
template <class T>
decltype(std::begin(std::declval<const T&>()),
         std::end(std::declval<const T&>()), void())
arrayParamOut(CheckpointOut &os, const std::string &name,
              const T &param)
{
    arrayParamOut(os, name, std::begin(param), std::end(param));
}


/**
 * @ingroup api_serialize
 */
template <class T>
void
arrayParamOut(CheckpointOut &os, const std::string &name,
              const T *param, unsigned size)
{
    arrayParamOut(os, name, param, param + size);
}

/**
 * Extract values stored in the checkpoint, and assign them to the provided
 * array container.
 *
 * @param cp The checkpoint to be parsed.
 * @param name Name of the container.
 * @param param The array container.
 * @param size The expected number of entries to be extracted.
 *
 * @ingroup api_serialize
 */

template <class T, class InsertIterator>
void
arrayParamIn(CheckpointIn &cp, const std::string &name,
             InsertIterator inserter, ssize_t fixed_size=-1)
{
    const std::string &section = Serializable::currentSection();
    std::string str;
    fatal_if(!cp.find(section, name, str),
        "Can't unserialize '%s:%s'.", section, name);

    std::vector<std::string> tokens;
    tokenize(tokens, str, ' ');

    fatal_if(fixed_size >= 0 && tokens.size() != fixed_size,
             "Array size mismatch on %s:%s (Got %u, expected %u)'\n",
             section, name, tokens.size(), fixed_size);

    for (const auto &token: tokens) {
        T value;
        fatal_if(!ParseParam<T>::parse(token, value),
                 "Could not parse \"%s\".", str);
        *inserter = value;
    }
}

/**
 * @ingroup api_serialize
 */
template <class T>
decltype(std::declval<T>().insert(std::declval<typename T::value_type>()),
         void())
arrayParamIn(CheckpointIn &cp, const std::string &name, T &param)
{
    param.clear();
    arrayParamIn<typename T::value_type>(
            cp, name, std::inserter(param, param.begin()));
}

/**
 * @ingroup api_serialize
 */
template <class T>
decltype(std::declval<T>().push_back(std::declval<typename T::value_type>()),
         void())
arrayParamIn(CheckpointIn &cp, const std::string &name, T &param)
{
    param.clear();
    arrayParamIn<typename T::value_type>(cp, name, std::back_inserter(param));
}

/**
 * @ingroup api_serialize
 */
template <class T>
void
arrayParamIn(CheckpointIn &cp, const std::string &name,
             T *param, unsigned size)
{
    struct ArrayInserter
    {
        T *data;
        T &operator *() { return *data++; }
    } insert_it{param};

    arrayParamIn<T>(cp, name, insert_it, size);
}

/**
 * Serialize a mapping represented as two arrays: one containing names
 * and the other containing values.
 *
 * @param names array of keys
 * @param param array of values
 * @param size size of the names and param arrays
 */
template <class T>
void
mappingParamOut(CheckpointOut &os, const char* sectionName,
    const char* const names[], const T *param, unsigned size)
{
    Serializable::ScopedCheckpointSection sec(os, sectionName);
    for (unsigned i = 0; i < size; ++i) {
        paramOut(os, names[i], param[i]);
    }
}

/**
 * Restore mappingParamOut. Keys missing from the checkpoint are ignored.
 */
template <class T>
void
mappingParamIn(CheckpointIn &cp, const char* sectionName,
    const char* const names[], T *param, unsigned size)
{
    Serializable::ScopedCheckpointSection sec(cp, sectionName);
    std::unordered_map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < size; i++) {
        name_to_index[names[i]] = i;
    }
    for (size_t i = 0; i < size; i++) {
        auto& key = names[i];
        T value;
        if (optParamIn(cp, key, value)) {
            param[name_to_index[key]] = value;
        }
    }
    cp.visitSection(
        Serializable::currentSection(),
        [name_to_index](const std::string& key, const std::string& val)
        {
            if (!name_to_index.count(key)) {
                warn("unknown entry found in checkpoint: %s %s %s\n",
                    Serializable::currentSection(), key, val);
            }
        }
    );
}

//
// These macros are streamlined to use in serialize/unserialize
// functions.  It's assumed that serialize() has a parameter 'os' for
// the ostream, and unserialize() has parameters 'cp' and 'section'.


/**
 * \def SERIALIZE_SCALER(scaler)
 *
 * @ingroup api_serialize
 */
#define SERIALIZE_SCALAR(scalar)        paramOut(cp, #scalar, scalar)

/**
 * \def UNSERIALIZE_SCALER(scalar)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_SCALAR(scalar)      paramIn(cp, #scalar, scalar)

/**
 * \def UNSERIALIZE_OPT_SCALAR(scalar)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_OPT_SCALAR(scalar)      optParamIn(cp, #scalar, scalar)

// ENUMs are like SCALARs, but we cast them to ints on the way out

/**
 * \def SERIALIZE_ENUM(scalar)
 *
 * @ingroup api_serialize
 */
#define SERIALIZE_ENUM(scalar)          paramOut(cp, #scalar, (int)scalar)

/**
 * \def UNSERIALIZE_ENUM(scaler)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_ENUM(scalar)                        \
    do {                                                \
        int tmp;                                        \
        ::gem5::paramIn(cp, #scalar, tmp);              \
        scalar = static_cast<decltype(scalar)>(tmp);    \
    } while (0)

/**
 * \def SERIALIZE_ARRAY(member, size)
 *
 * @ingroup api_serialize
 */
#define SERIALIZE_ARRAY(member, size)           \
        ::gem5::arrayParamOut(cp, #member, member, size)

/**
 * \def UNSERIALIZE_ARRAY(member, size)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_ARRAY(member, size)         \
        ::gem5::arrayParamIn(cp, #member, member, size)

/**
 * \def SERIALIZE_CONTAINER(member)
 *
 * @ingroup api_serialize
 */
#define SERIALIZE_CONTAINER(member)             \
        ::gem5::arrayParamOut(cp, #member, member)

/**
 * \def UNSERIALIZE_CONTAINER(member)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_CONTAINER(member)           \
        ::gem5::arrayParamIn(cp, #member, member)

/**
 * \def SERIALIZE_OBJ(obj)
 *
 * This macro serializes an object into its own section. The object must
 * inherit from Serializable, but NOT from SimObject (i.e, it is an object
 * in the strict sense of "object oriented programing"). Objects that
 * derive from SimObject are automatically serialized elsewhere
 * (@see Serializable, SimObject::serializeAll()).
 *
 * @ingroup api_serialize
 */
#define SERIALIZE_OBJ(obj) obj.serializeSection(cp, #obj)

/**
 * \def UNSERIALIZE_OBJ(obj)
 *
 * @ingroup api_serialize
 */
#define UNSERIALIZE_OBJ(obj) obj.unserializeSection(cp, #obj)

/**
 * \def SERIALIZE_MAPPING(member, names, size)
 */
#define SERIALIZE_MAPPING(member, names, size) \
        ::gem5::mappingParamOut(cp, #member, names, member, size)

/**
 * \def UNSERIALIZE_MAPPING(member, names, size)
 */
#define UNSERIALIZE_MAPPING(member, names, size) \
        ::gem5::mappingParamIn(cp, #member, names, member, size)

} // namespace gem5

#endif // __SERIALIZE_HH__
