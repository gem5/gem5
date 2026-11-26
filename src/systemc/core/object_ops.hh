/*
 * Copyright 2026 Google LLC
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

#ifndef __SYSTEMC_CORE_OBJECT_OPS_HH__
#define __SYSTEMC_CORE_OBJECT_OPS_HH__

/*
 * Free-function interface to sc_gem5::Object and the object tree, used by
 * core/sc_object.cc, sc_simcontext.cc, sc_module.cc and utils/sc_vector.cc.
 * It needs only forward declarations, so its callers do not have to see
 * the kernel's object.hh.
 */

#include <iosfwd>
#include <string>
#include <vector>

namespace sc_core
{

class sc_attr_base;
class sc_attr_cltn;
class sc_event;
class sc_object;
class sc_simcontext;

} // namespace sc_core

namespace sc_gem5
{

class Object;

Object *newObject(sc_core::sc_object *sc_obj);
Object *newObject(sc_core::sc_object *sc_obj, const char *name);
Object *copyObject(sc_core::sc_object *sc_obj, const Object *other);
void destroyObject(Object *obj);
void assignObject(Object *obj, const Object *other);

const char *getObjectName(const Object *obj);
const char *getObjectBasename(const Object *obj);
void printObject(const Object *obj, std::ostream &out);
void dumpObject(const Object *obj, std::ostream &out);

const std::vector<sc_core::sc_object *> &getChildObjects(const Object *obj);
const std::vector<sc_core::sc_event *> &getChildEvents(const Object *obj);
sc_core::sc_object *getParentObject(const Object *obj);

bool addObjectAttribute(Object *obj, sc_core::sc_attr_base &attr);
sc_core::sc_attr_base *getObjectAttribute(Object *obj,
                                          const std::string &name);
sc_core::sc_attr_base *removeObjectAttribute(Object *obj,
                                             const std::string &name);
void removeAllObjectAttributes(Object *obj);
int getNumObjectAttributes(const Object *obj);
sc_core::sc_attr_cltn &getObjectAttrCltn(Object *obj);
const sc_core::sc_attr_cltn &getObjectAttrCltn(const Object *obj);

sc_core::sc_simcontext *getObjectSimcontext(const Object *obj);

const std::vector<sc_core::sc_object *> &getTopLevelObjects();
const std::vector<sc_core::sc_object *> &getAllObjects();

sc_core::sc_object *findObject(const char *name);

void pushParentObj(sc_core::sc_object *obj);
void popParentObj();

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_OBJECT_OPS_HH__
