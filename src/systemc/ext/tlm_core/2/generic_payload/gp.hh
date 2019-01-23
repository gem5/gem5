/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

#ifndef __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_GP_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_GP_HH__

#include <typeinfo> // std::type_info

#include "../../../utils/sc_report_handler.hh" // sc_assert
#include "array.hh"

namespace tlm
{

class tlm_generic_payload;

class tlm_mm_interface
{
  public:
    virtual void free(tlm_generic_payload *) = 0;
    virtual ~tlm_mm_interface() {}
};

//---------------------------------------------------------------------------
// Classes and helpers for the extension mechanism
//---------------------------------------------------------------------------
// Helper function:
unsigned int max_num_extensions();

// This class can be used for storing pointers to the extension classes, used
// in tlm_generic_payload:
class tlm_extension_base
{
  public:
    virtual tlm_extension_base *clone() const = 0;
    virtual void free() { delete this; }
    virtual void copy_from(tlm_extension_base const &) = 0;
  protected:
    virtual ~tlm_extension_base() {}
    static unsigned int register_extension(const std::type_info &);
};

// Base class for all extension classes, derive your extension class in
// the following way:
// class my_extension : public tlm_extension<my_extension> { ...
// This triggers proper extension registration during C++ static
// contruction time. my_extension::ID will hold the unique index in the
// tlm_generic_payload::m_extensions array.
template <typename T>
class tlm_extension : public tlm_extension_base
{
  public:
    virtual tlm_extension_base *clone() const = 0;
    virtual void copy_from(tlm_extension_base const &ext) = 0;
    virtual ~tlm_extension() {}
    const static unsigned int ID;
};

template <typename T>
const unsigned int tlm_extension<T>::ID =
    tlm_extension_base::register_extension(typeid(T));

//---------------------------------------------------------------------------
// enumeration types
//---------------------------------------------------------------------------
enum tlm_command
{
    TLM_READ_COMMAND,
    TLM_WRITE_COMMAND,
    TLM_IGNORE_COMMAND
};

enum tlm_response_status
{
    TLM_OK_RESPONSE = 1,
    TLM_INCOMPLETE_RESPONSE = 0,
    TLM_GENERIC_ERROR_RESPONSE = -1,
    TLM_ADDRESS_ERROR_RESPONSE = -2,
    TLM_COMMAND_ERROR_RESPONSE = -3,
    TLM_BURST_ERROR_RESPONSE = -4,
    TLM_BYTE_ENABLE_ERROR_RESPONSE = -5
};

enum tlm_gp_option
{
    TLM_MIN_PAYLOAD,
    TLM_FULL_PAYLOAD,
    TLM_FULL_PAYLOAD_ACCEPTED
};

#define TLM_BYTE_DISABLED 0x0
#define TLM_BYTE_ENABLED 0xff

//---------------------------------------------------------------------------
// The generic payload class:
//---------------------------------------------------------------------------

extern template class tlm_array<tlm_extension_base *>;

class tlm_generic_payload
{
  public:
    tlm_generic_payload();
    explicit tlm_generic_payload(tlm_mm_interface *mm);

    void
    acquire()
    {
        sc_assert(m_mm != 0);
        m_ref_count++;
    }

    void
    release()
    {
        sc_assert(m_mm != 0 && m_ref_count > 0);
        if (--m_ref_count == 0)
            m_mm->free(this);
    }

    int get_ref_count() const { return m_ref_count; }

    void set_mm(tlm_mm_interface *mm) { m_mm = mm; }
    bool has_mm() const { return m_mm != 0; }

    void reset();

  private:
    // Disabled copy ctor and assignment operator.
    tlm_generic_payload(const tlm_generic_payload &x);
    tlm_generic_payload &operator = (const tlm_generic_payload &x);

  public:
    // Non-virtual deep-copying of the object.
    void deep_copy_from(const tlm_generic_payload &other);

    // To update the state of the original generic payload from a deep copy.
    // Assumes that "other" was created from the original by calling
    // deep_copy_from Argument use_byte_enable_on_read determines whether to
    // use or ignores byte enables when copying back the data array on a read
    // command.

    void update_original_from(const tlm_generic_payload &other,
                              bool use_byte_enable_on_read=true);

    void update_extensions_from(const tlm_generic_payload &other);

    // Free all extensions. Useful when reusing a cloned transaction that
    // doesn't have memory manager. Normal and sticky extensions are freed and
    // extension array cleared.
    void free_all_extensions();

    virtual ~tlm_generic_payload();

    //----------------
    // API (including setters & getters).
    //---------------

    // Command related method.
    bool is_read() const { return (m_command == TLM_READ_COMMAND); }
    void set_read() { m_command = TLM_READ_COMMAND; }
    bool is_write() const { return (m_command == TLM_WRITE_COMMAND); }
    void set_write() { m_command = TLM_WRITE_COMMAND; }
    tlm_command get_command() const { return m_command; }
    void set_command(const tlm_command command) { m_command = command; }

    // Address related methods.
    sc_dt::uint64 get_address() const { return m_address; }
    void set_address(const sc_dt::uint64 address) { m_address = address; }

    // Data related methods.
    unsigned char *get_data_ptr() const { return m_data; }
    void set_data_ptr(unsigned char *data) { m_data = data; }

    // Transaction length (in bytes) related methods.
    unsigned int get_data_length() const { return m_length; }
    void set_data_length(const unsigned int length) { m_length = length; }

    // Response status related methods.
    bool is_response_ok() const { return (m_response_status > 0); }
    bool is_response_error() const { return (m_response_status <= 0); }
    tlm_response_status
    get_response_status() const
    {
        return m_response_status;
    }
    void
    set_response_status(const tlm_response_status response_status)
    {
        m_response_status = response_status;
    }
    std::string get_response_string() const;

    // Streaming related methods.
    unsigned int get_streaming_width() const { return m_streaming_width; }
    void
    set_streaming_width(const unsigned int streaming_width)
    {
        m_streaming_width = streaming_width;
    }

    // Byte enable related methods.
    unsigned char *get_byte_enable_ptr() const { return m_byte_enable; }
    void
    set_byte_enable_ptr(unsigned char *byte_enable)
    {
        m_byte_enable = byte_enable;
    }
    unsigned int
    get_byte_enable_length() const
    {
        return m_byte_enable_length;
    }
    void
    set_byte_enable_length(const unsigned int byte_enable_length)
    {
        m_byte_enable_length = byte_enable_length;
    }

    // This is the "DMI-hint" a slave can set this to true if it
    // wants to indicate that a DMI request would be supported:
    void
    set_dmi_allowed(bool dmi_allowed)
    {
        m_dmi = dmi_allowed;
    }
    bool
    is_dmi_allowed() const
    {
        return m_dmi;
    }

    // Use full set of attributes in DMI/debug?
    tlm_gp_option get_gp_option() const { return m_gp_option; }
    void set_gp_option(const tlm_gp_option gp_opt) { m_gp_option = gp_opt; }

  private:
    /* --------------------------------------------------------------------- */
    /* Generic Payload attributes:                                           */
    /* --------------------------------------------------------------------- */
    /* - m_command         : Type of transaction. Three values supported:    */
    /*                       - TLM_WRITE_COMMAND                             */
    /*                       - TLM_READ_COMMAND                              */
    /*                       - TLM_IGNORE_COMMAND                            */
    /* - m_address         : Transaction base address (byte-addressing).     */
    /* - m_data            : When m_command = TLM_WRITE_COMMAND contains a   */
    /*                       pointer to the data to be written in the target.*/
    /*                       When m_command = TLM_READ_COMMAND contains a    */
    /*                       pointer where to copy the data read from the    */
    /*                       target.                                         */
    /* - m_length          : Total number of bytes of the transaction.       */
    /* - m_response_status : This attribute indicates whether an error has   */
    /*                       occurred during the transaction.                */
    /*                       Values supported are:                           */
    /*                       - TLM_OK_RESP                                   */
    /*                       - TLM_INCOMPLETE_RESP                           */
    /*                       - TLM_GENERIC_ERROR_RESP                        */
    /*                       - TLM_ADDRESS_ERROR_RESP                        */
    /*                       - TLM_COMMAND_ERROR_RESP                        */
    /*                       - TLM_BURST_ERROR_RESP                          */
    /*                       - TLM_BYTE_ENABLE_ERROR_RESP                    */
    /*                                                                       */
    /* - m_byte_enable     : It can be used to create burst transfers where  */
    /*                    the address increment between each beat is greater */
    /*                    than the word length of each beat, or to place     */
    /*                    words in selected byte lanes of a bus.             */
    /* - m_byte_enable_length : For a read or a write command, the target    */
    /*                    interpret the byte enable length attribute as the  */
    /*                    number of elements in the bytes enable array.      */
    /* - m_streaming_width  :                                                */
    /* --------------------------------------------------------------------- */

    sc_dt::uint64 m_address;
    tlm_command m_command;
    unsigned char *m_data;
    unsigned int m_length;
    tlm_response_status m_response_status;
    bool m_dmi;
    unsigned char *m_byte_enable;
    unsigned int m_byte_enable_length;
    unsigned int m_streaming_width;
    tlm_gp_option m_gp_option;

  public:
    /* --------------------------------------------------------------------- */
    /* Dynamic extension mechanism:                                          */
    /* --------------------------------------------------------------------- */
    /* The extension mechanism is intended to enable initiator modules to    */
    /* optionally and transparently add data fields to the                   */
    /* tlm_generic_payload. Target modules are free to check for extensions  */
    /* and may or may not react to the data in the extension fields. The     */
    /* definition of the extensions' semantics is solely in the              */
    /* responsibility of the user.                                           */
    /*                                                                       */
    /* The following rules apply:                                            */
    /*                                                                       */
    /* - Every extension class must be derived from tlm_extension, e.g.:     */
    /*     class my_extension : public tlm_extension<my_extension> { ... }   */
    /*                                                                       */
    /* - A tlm_generic_payload object should be constructed after C++        */
    /*   static initialization time. This way it is guaranteed that the      */
    /*   extension array is of sufficient size to hold all possible          */
    /*   extensions. Alternatively, the initiator module can enforce a valid */
    /*   extension array size by calling the resize_extensions() method      */
    /*   once before the first transaction with the payload object is        */
    /*   initiated.                                                          */
    /*                                                                       */
    /* - Initiators should use the the set_extension(e) or clear_extension(e)*/
    /*   methods for manipulating the extension array. The type of the       */
    /*   argument must be a pointer to the specific registered extension     */
    /*   type (my_extension in the above example) and is used to             */
    /*   automatically locate the appropriate index in the array.            */
    /*                                                                       */
    /* - Targets can check for a specific extension by calling               */
    /*   get_extension(e). e will point to zero if the extension is not      */
    /*   present.                                                            */
    /*                                                                       */
    /* --------------------------------------------------------------------- */

    // Stick the pointer to an extension into the vector, return the
    // previous value:
    template <typename T>
    T *
    set_extension(T *ext)
    {
        return static_cast<T *>(set_extension(T::ID, ext));
    }

    // Non-templatized version with manual index:
    tlm_extension_base *set_extension(
            unsigned int index, tlm_extension_base *ext);

    // Stick the pointer to an extension into the vector, return the
    // previous value and schedule its release.
    template <typename T>
    T *
    set_auto_extension(T *ext)
    {
        return static_cast<T *>(set_auto_extension(T::ID, ext));
    }

    // Non-templatized version with manual index:
    tlm_extension_base *set_auto_extension(
            unsigned int index, tlm_extension_base *ext);

    // Check for an extension, ext will point to 0 if not present.
    template <typename T>
    void get_extension(T *& ext) const { ext = get_extension<T>(); }
    template <typename T>
    T *
    get_extension() const
    {
        return static_cast<T*>(get_extension(T::ID));
    }
    // Non-templatized version with manual index:
    tlm_extension_base *get_extension(unsigned int index) const;

    // This call just removes the extension from the txn but does not
    // call free() or tells the MM to do so it return false if there was
    // active MM so you are now in an unsafe situation recommended use:
    // when 100% sure there is no MM.
    template <typename T>
    void clear_extension(const T *ext) { clear_extension<T>(); }

    // This call just removes the extension from the txn but does not
    // call free() or tells the MM to do so it return false if there was
    // active MM so you are now in an unsafe situation recommended use: when
    // 100% sure there is no MM.
    template <typename T>
    void clear_extension() { clear_extension(T::ID); }

    // This call removes the extension from the txn and does call free() or
    // tells the MM to do so when the txn is finally done recommended use:
    // when not sure there is no MM.
    template <typename T>
    void release_extension(T *ext)
    {
        release_extension<T>();
    }

    // This call removes the extension from the txn and does call free() or
    // tells the MM to do so when the txn is finally done recommended use:
    // when not sure there is no MM
    template <typename T>
    void release_extension()
    {
        release_extension(T::ID);
    }

  private:
    // Non-templatized version with manual index
    void clear_extension(unsigned int index);
    // Non-templatized version with manual index
    void release_extension(unsigned int index);

  public:
    // Make sure the extension array is large enough. Can be called once by
    // an initiator module (before issuing the first transaction) to make
    // sure that the extension array is of correct size. This is only needed
    // if the initiator cannot guarantee that the generic payload object is
    // allocated after C++ static construction time.
    void resize_extensions();

  private:
    tlm_array<tlm_extension_base *> m_extensions;
    tlm_mm_interface *m_mm;
    unsigned int m_ref_count;
};

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_GP_HH__ */
