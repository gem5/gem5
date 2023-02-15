/*
 * Copyright 2023 Google, Inc.
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
 * Extensible Object Base Class Declaration
 */

#ifndef __BASE_EXTENSIBLE_HH__
#define __BASE_EXTENSIBLE_HH__

#include <list>
#include <memory>
#include <utility>

namespace gem5
{

/**
 * This is base of every extension.
 */
class ExtensionBase
{
  public:
    explicit ExtensionBase(const unsigned int id)
        : extID(id) {}

    virtual ~ExtensionBase() = default;

    virtual std::unique_ptr<ExtensionBase> clone() const = 0;

    static unsigned int
    maxNumExtensions()
    {
        static unsigned int max_num = 0;
        return ++max_num;
    }

    unsigned int getExtensionID() const { return extID; }

  private:
    const unsigned int extID;
};

/**
 * This is the extension for carrying additional information.
 * Each type of extension will have a unique extensionID.
 * This extensionID will assign to base class for comparsion.
 */
template <typename Target, typename T>
class Extension : public ExtensionBase
{
  public:
    Extension() : ExtensionBase(extensionID) {}

    const static unsigned int extensionID;
};

template <typename Target, typename T>
const unsigned int Extension<Target, T>::extensionID =
        ExtensionBase::maxNumExtensions() - 1;

template <typename Target>
class Extensible
{
  public:
     Extensible() = default;
     Extensible(const Extensible& other)
     {
        // Clone every extension from other.
        for (auto& ext : other.extensions) {
            extensions.emplace_back(ext->clone());
        }
     }
     virtual ~Extensible() = default;

    /**
     * Set a new extension to the packet and replace the old one, if there
     * already exists the same type of extension in this packet. This new
     * extension will be deleted automatically with the shared_ptr<>.
     *
     * @param ext Extension to set
     */
    template <typename T>
    void
    setExtension(std::shared_ptr<T> ext)
    {
        static_assert(std::is_base_of<ExtensionBase, T>::value,
                      "Extension should inherit from ExtensionBase.");
        assert(ext.get() != nullptr);

        auto it = findExtension<T>();

        if (it != extensions.end()) {
            // There exists the same type of extension in the list.
            // Replace it to the new one.
            *it = std::move(ext);
        } else {
            // Add ext into the linked list.
            extensions.emplace_back(std::move(ext));
        }
    }

    /**
     * Remove the extension based on its type.
     *
     * @param ext Extension to remove
     */
    template <typename T>
    void
    removeExtension(void)
    {
        static_assert(std::is_base_of<ExtensionBase, T>::value,
                      "Extension should inherit from ExtensionBase.");

        auto it = findExtension<T>();
        if (it != extensions.end())
            extensions.erase(it);
    }

    /**
     * Get the extension pointer by linear search with the extensionID.
     */
    template <typename T>
    std::shared_ptr<T>
    getExtension()
    {
        static_assert(std::is_base_of<ExtensionBase, T>::value,
                      "Extension should inherit from ExtensionBase.");
        auto it = findExtension<T>();
        if (it == extensions.end())
            return nullptr;
        return std::static_pointer_cast<T>(*it);
    }

  protected:

    /**
     * Go through the extension list and return the iterator to the instance of
     * the type of extension. If there is no such an extension, return the end
     * iterator of the list.
     *
     *  @return The iterator to the extension type T if there exists.
     */
    template <typename T>
    std::list<std::shared_ptr<ExtensionBase>>::iterator
    findExtension()
    {
        auto it = extensions.begin();
        while (it != extensions.end()) {
            if ((*it)->getExtensionID() == T::extensionID)
                break;
            it++;
        }
        return it;
    }

    // Linked list of extensions.
    std::list<std::shared_ptr<ExtensionBase>> extensions;
};

} // namespace gem5

#endif //__BASE_EXTENSIBLE_HH__
