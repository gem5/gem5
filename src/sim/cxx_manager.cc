/*
 * Copyright (c) 2014 ARM Limited
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
 * Authors: Andrew Bardsley
 */

#include "sim/cxx_manager.hh"

#include <cstdlib>
#include <sstream>

#include "base/str.hh"
#include "base/trace.hh"
#include "debug/CxxConfig.hh"
#include "sim/serialize.hh"
#include "sim/sim_object.hh"

CxxConfigManager::CxxConfigManager(CxxConfigFileBase &configFile_) :
    configFile(configFile_), flags(configFile_.getFlags()),
    simObjectResolver(*this)
{
}

const CxxConfigDirectoryEntry &
CxxConfigManager::findObjectType(const std::string &object_name,
    std::string &object_type)
{
    if (!configFile.objectExists(object_name))
        throw Exception(object_name, "Can't find sim object");

    if (!configFile.getParam(object_name, "type", object_type))
        throw Exception(object_name, "Sim object has no 'type' field");

    if (cxx_config_directory.find(object_type) ==
        cxx_config_directory.end())
    {
        throw Exception(object_name, csprintf(
            "No sim object type %s is available", object_type));
    }

    const CxxConfigDirectoryEntry *entry = cxx_config_directory[object_type];

    return *entry;
}

std::string
CxxConfigManager::rename(const std::string &from_name)
{
    for (auto i = renamings.begin(); i != renamings.end(); ++ i) {
        const Renaming &renaming = *i;

        if (from_name.find(renaming.fromPrefix) == 0) {
            return renaming.toPrefix +
                from_name.substr(renaming.fromPrefix.length());
        }
    }

    return from_name;
}

std::string
CxxConfigManager::unRename(const std::string &to_name)
{
    for (auto i = renamings.begin(); i != renamings.end(); ++ i) {
        const Renaming &renaming = *i;

        if (to_name.find(renaming.toPrefix) == 0) {
            return renaming.fromPrefix +
                to_name.substr(renaming.toPrefix.length());
        }
    }

    return to_name;
}

static
std::string formatParamList(const std::vector<std::string> &param_values)
{
    std::ostringstream params;

    auto i = param_values.begin();
    auto end_i = param_values.end();

    params << '[';
    while (i != end_i) {
        params << (*i);
        ++i;

        if (i != end_i)
            params << ", ";
    }
    params << ']';

    return params.str();
}

SimObject *
CxxConfigManager::findObject(const std::string &object_name,
    bool visit_children)
{
    std::string instance_name = rename(object_name);

    if (object_name == "Null")
        return NULL;

    /* Already constructed */
    if (objectsByName.find(instance_name) != objectsByName.end())
        return objectsByName[instance_name];

    if (inVisit.find(instance_name) != inVisit.end())
        throw Exception(instance_name, "Cycle in configuration");

    std::string object_type;
    const CxxConfigDirectoryEntry &entry =
        findObjectType(object_name, object_type);

    SimObject *object = NULL;

    CxxConfigParams *object_params = findObjectParams(object_name);

    try {
        DPRINTF(CxxConfig, "Configuring sim object references for: %s"
            " (%s from object %s)\n", instance_name, object_type,
            object_name);

        /* Remember the path back to the top of the recursion to detect
         *  cycles */
        inVisit.insert(instance_name);

        /* Resolve pointed-to SimObjects by recursing into them */
        for (auto i = entry.parameters.begin();
            i != entry.parameters.end(); ++i)
        {
            const CxxConfigDirectoryEntry::ParamDesc *param = (*i).second;

            if (param->isSimObject) {
                if (param->isVector) {
                    std::vector<std::string> sub_object_names;

                    if (!configFile.getParamVector(object_name, param->name,
                        sub_object_names))
                    {
                        throw Exception(object_name, csprintf(
                            "Element not found: %s", param->name));
                    }

                    std::vector<SimObject *> sub_objects;

                    for (auto n = sub_object_names.begin();
                        n != sub_object_names.end(); ++n)
                    {
                        SimObject *sub_object = findObject(*n,
                            visit_children);

                        if (sub_object)
                            sub_objects.push_back(sub_object);
                    }

                    if (!object_params->setSimObjectVector(param->name,
                        sub_objects))
                    {
                        throw Exception(object_name, csprintf(
                            "Can't assign sim object element %s from \"%s\"",
                            param->name, formatParamList(sub_object_names)));
                    }

                    DPRINTF(CxxConfig, "Setting sim object(s): %s.%s=%s\n",
                        object_name, param->name,
                        formatParamList(sub_object_names));
                } else {
                    std::string sub_object_name;

                    if (!configFile.getParam(object_name, param->name,
                        sub_object_name))
                    {
                        throw Exception(object_name, csprintf(
                            "Element not found: %s", param->name));
                    }

                    SimObject *sub_object = findObject(sub_object_name,
                        visit_children);

                    if (sub_object) {
                        if (!object_params->setSimObject(param->name,
                            sub_object))
                        {
                            throw Exception(object_name, csprintf(
                                "Can't assign sim object element %s from"
                                " \"%s\"", param->name, sub_object_name));
                        }
                    }

                    DPRINTF(CxxConfig, "Setting sim object(s):"
                        " %s.%s=%s\n", object_name, param->name,
                        sub_object_name);
                }
            }
        }

        DPRINTF(CxxConfig, "Creating SimObject: %s\n", instance_name);
        object = object_params->simObjectCreate();

        if (!object) {
            throw Exception(object_name, csprintf("Couldn't create object of"
                " type: %s", object_type));
        }

        objectsByName[instance_name] = object;
        objectParamsByName[instance_name] = object_params;

        if (visit_children) {
            std::vector<std::string> children;
            configFile.getObjectChildren(object_name, children, true);

            /* Visit all your children */
            for (auto i = children.begin(); i != children.end(); ++i)
                findObject(*i, visit_children);
        }
    } catch (Exception &) {
        delete object_params;
        throw;
    }

    /* Mark that we've exited object
     *  construction and so 'find'ing this object again won't be a
     *  configuration loop */
    inVisit.erase(object_name);
    return object;
}

CxxConfigParams *
CxxConfigManager::findObjectParams(const std::string &object_name)
{
    std::string instance_name = rename(object_name);

    /* Already constructed */
    if (objectParamsByName.find(instance_name) != objectParamsByName.end())
        return objectParamsByName[instance_name];

    std::string object_type;
    const CxxConfigDirectoryEntry &entry =
        findObjectType(object_name, object_type);

    DPRINTF(CxxConfig, "Configuring parameters of object: %s (%s)\n",
        instance_name, object_type);

    CxxConfigParams *object_params = entry.makeParamsObject();

    try {
        /* Fill in the implicit parameters that don't necessarily
         *  appear in config files */
        object_params->setName(instance_name);

        /* Fill in parameters */
        for (auto i = entry.parameters.begin();
            i != entry.parameters.end(); ++i)
        {
            const CxxConfigDirectoryEntry::ParamDesc *param = (*i).second;

            if (!param->isSimObject) {
                /* Only handle non-SimObject parameters here (see below) */

                if (param->isVector) {
                    std::vector<std::string> param_values;

                    if (!configFile.getParamVector(object_name, param->name,
                        param_values))
                    {
                        throw Exception(object_name, csprintf(
                            "Element not found for parameter: %s",
                            param->name));
                    }

                    if (!object_params->setParamVector(param->name,
                        param_values, flags))
                    {
                        throw Exception(instance_name, csprintf(
                            "Bad parameter value: .%s=X=\"%s\"",
                            param->name, formatParamList(param_values)));
                    }

                    DPRINTF(CxxConfig, "Setting parameter"
                        " %s.%s=%s\n", instance_name, param->name,
                        formatParamList(param_values));
                } else {
                    std::string param_value;

                    if (!configFile.getParam(object_name, param->name,
                        param_value))
                    {
                        throw Exception(object_name, csprintf(
                            "Element not found for parameter: %s",
                            param->name));
                    }

                    if (!object_params->setParam(param->name, param_value,
                        flags))
                    {
                        throw Exception(instance_name, csprintf(
                            "Bad parameter value: .%s=X=\"%s\"",
                            param->name, param_value));
                    }

                    DPRINTF(CxxConfig, "Setting parameter %s.%s=%s\n",
                        instance_name, param->name, param_value);
                }
            }
        }

        /* Find the number of ports that will need binding and set the
         *  appropriate port_..._connection_count parameters */
        for (auto i = entry.ports.begin(); i != entry.ports.end(); ++i) {
            const CxxConfigDirectoryEntry::PortDesc *port = (*i).second;
            std::vector<std::string> peers;

            if (!configFile.getPortPeers(object_name, port->name, peers)) {
                DPRINTF(CxxConfig, "Port not found: %s.%s,"
                    " assuming there are no connections\n",
                    instance_name, port->name);
            }

            unsigned int peer_count = peers.size();

            /* It would be more efficient to split the peer list and
             *  save the values for peer binding later but that would
             *  require another annoying intermediate structure to
             *  hold for little performance increase */

            if (!object_params->setPortConnectionCount(port->name,
                peer_count))
            {
                throw Exception(instance_name, csprintf(
                    "Unconnected port: %s", port->name));
            }

            DPRINTF(CxxConfig, "Setting port connection count"
                " for: %s.%s to %d\n",
                instance_name, port->name, peer_count);
        }

        /* Set pointed-to SimObjects to NULL */
        for (auto i = entry.parameters.begin();
            i != entry.parameters.end(); ++i)
        {
            const CxxConfigDirectoryEntry::ParamDesc *param = (*i).second;

            if (param->isSimObject) {
                bool ret;

                DPRINTF(CxxConfig, "Nulling sim object reference: %s.%s\n",
                    instance_name, param->name);

                if (param->isVector) {
                    /* Clear the reference list. */
                    std::vector<SimObject *> empty;
                    ret = object_params->setSimObjectVector(param->name,
                        empty);
                } else {
                    ret = object_params->setSimObject(param->name, NULL);
                }

                if (!ret) {
                    throw Exception(instance_name, csprintf(
                        "Error nulling sim object reference(s): %s",
                        param->name));
                }
            }
        }
    } catch (Exception &) {
        delete object_params;
        throw;
    }

    objectParamsByName[instance_name] = object_params;

    return object_params;
}

void
CxxConfigManager::findAllObjects()
{
    std::vector<std::string> objects;
    configFile.getAllObjectNames(objects);

    /* Set the traversal order for further iterators */
    objectsInOrder.clear();
    findTraversalOrder("root");
}

void
CxxConfigManager::findTraversalOrder(const std::string &object_name)
{
    SimObject *object = findObject(object_name);

    if (object) {
        objectsInOrder.push_back(object);

        std::vector<std::string> children;
        configFile.getObjectChildren(object_name, children, true);

        /* Visit all your children */
        for (auto i = children.begin(); i != children.end(); ++i)
            findTraversalOrder(*i);
    }
}

void
CxxConfigManager::bindAllPorts()
{
    for (auto i = objectsInOrder.begin(); i != objectsInOrder.end(); ++i)
        bindObjectPorts(*i);
}

void
CxxConfigManager::bindPort(
    SimObject *master_object, const std::string &master_port_name,
    PortID master_port_index,
    SimObject *slave_object, const std::string &slave_port_name,
    PortID slave_port_index)
{
    /* FIXME, check slave_port_index against connection_count
     *  defined for port, need getPortConnectionCount and a
     *  getCxxConfigDirectoryEntry for each object. */

    /* It would be nice to be able to catch the errors from these calls. */
    Port &master_port = master_object->getPort(
        master_port_name, master_port_index);
    Port &slave_port = slave_object->getPort(
        slave_port_name, slave_port_index);

    if (master_port.isConnected()) {
        throw Exception(master_object->name(), csprintf(
            "Master port: %s[%d] is already connected\n", master_port_name,
            master_port_index));
    }

    if (slave_port.isConnected()) {
        throw Exception(slave_object->name(), csprintf(
            "Slave port: %s[%d] is already connected\n", slave_port_name,
            slave_port_index));
    }

    DPRINTF(CxxConfig, "Binding port %s.%s[%d]"
        " to %s:%s[%d]\n",
        master_object->name(), master_port_name, master_port_index,
        slave_object->name(), slave_port_name, slave_port_index);

    master_port.bind(slave_port);
}

void
CxxConfigManager::bindMasterPort(SimObject *object,
    const CxxConfigDirectoryEntry::PortDesc &port,
    const std::vector<std::string> &peers)
{
    unsigned int master_port_index = 0;

    for (auto peer_i = peers.begin(); peer_i != peers.end();
        ++peer_i)
    {
        const std::string &peer = *peer_i;
        std::string slave_object_name;
        std::string slave_port_name;
        unsigned int slave_port_index;

        parsePort(peer, slave_object_name, slave_port_name,
            slave_port_index);

        std::string slave_instance_name = rename(slave_object_name);

        if (objectsByName.find(slave_instance_name) == objectsByName.end()) {
            throw Exception(object->name(), csprintf(
                "Can't find slave port object: %s", slave_instance_name));
        }

        SimObject *slave_object = objectsByName[slave_instance_name];

        bindPort(object, port.name, master_port_index,
            slave_object, slave_port_name, slave_port_index);

        master_port_index++;
    }
}

void
CxxConfigManager::bindObjectPorts(SimObject *object)
{
    /* We may want to separate object->name() from the name in configuration
     *  later to allow (for example) repetition of fragments of configs */
    const std::string &instance_name = object->name();

    std::string object_name = unRename(instance_name);

    std::string object_type;
    const CxxConfigDirectoryEntry &entry =
        findObjectType(object_name, object_type);

    DPRINTF(CxxConfig, "Binding ports of object: %s (%s)\n",
        instance_name, object_type);

    for (auto i = entry.ports.begin(); i != entry.ports.end(); ++i) {
        const CxxConfigDirectoryEntry::PortDesc *port = (*i).second;

        DPRINTF(CxxConfig, "Binding port: %s.%s\n", instance_name,
            port->name);

        std::vector<std::string> peers;
        configFile.getPortPeers(object_name, port->name, peers);

        /* Only handle master ports as binding only needs to happen once
         *  for each observed pair of ports */
        if (port->isMaster) {
            if (!port->isVector && peers.size() > 1) {
                throw Exception(instance_name, csprintf(
                    "Too many connections to non-vector port %s (%d)\n",
                    port->name, peers.size()));
            }

            bindMasterPort(object, *port, peers);
        }
    }
}

void
CxxConfigManager::parsePort(const std::string &inp,
    std::string &path, std::string &port, unsigned int &index)
{
    std::size_t dot_i = inp.rfind('.');
    std::size_t open_square_i = inp.rfind('[');

    if (dot_i == std::string::npos) {
        DPRINTF(CxxConfig, "Bad port string: %s\n", inp);
        path = "";
        port = "";
        index = 0;
    } else {
        path = std::string(inp, 0, dot_i);

        if (open_square_i == std::string::npos) {
            /* Singleton port */
            port = std::string(inp, dot_i + 1, inp.length() - dot_i);
            index = 0;
        } else {
            /* Vectored port elemnt */
            port = std::string(inp, dot_i + 1, (open_square_i - 1) - dot_i);
            index = std::atoi(inp.c_str() + open_square_i + 1);
        }
    }
}

void
CxxConfigManager::forEachObject(void (SimObject::*mem_func)())
{
    for (auto i = objectsInOrder.begin(); i != objectsInOrder.end(); ++i)
        ((*i)->*mem_func)();
}

void
CxxConfigManager::instantiate(bool build_all)
{
    if (build_all) {
        findAllObjects();
        bindAllPorts();
    }

    DPRINTF(CxxConfig, "Initialising all objects\n");
    forEachObject(&SimObject::init);

    DPRINTF(CxxConfig, "Registering stats\n");
    forEachObject(&SimObject::regStats);

    DPRINTF(CxxConfig, "Registering probe points\n");
    forEachObject(&SimObject::regProbePoints);

    DPRINTF(CxxConfig, "Connecting probe listeners\n");
    forEachObject(&SimObject::regProbeListeners);
}

void
CxxConfigManager::initState()
{
    DPRINTF(CxxConfig, "Calling initState on all objects\n");
    forEachObject(&SimObject::initState);
}

void
CxxConfigManager::startup()
{
    DPRINTF(CxxConfig, "Starting up all objects\n");
    forEachObject(&SimObject::startup);
}

unsigned int
CxxConfigManager::drain()
{
    return DrainManager::instance().tryDrain() ? 0 : 1;
}

void
CxxConfigManager::drainResume()
{
    DrainManager::instance().resume();
}

void
CxxConfigManager::serialize(std::ostream &os)
{
    for (auto i = objectsInOrder.begin(); i != objectsInOrder.end(); ++ i) {
        // (*i)->nameOut(os); FIXME, change access spec. for nameOut
        os << '[' << (*i)->name() << "]\n";
        (*i)->serialize(os);
    }
}

void
CxxConfigManager::loadState(CheckpointIn &checkpoint)
{
    for (auto i = objectsInOrder.begin(); i != objectsInOrder.end(); ++ i)
        (*i)->loadState(checkpoint);
}

void
CxxConfigManager::deleteObjects()
{
    for (auto i = objectsInOrder.rbegin(); i != objectsInOrder.rend(); ++i) {
        DPRINTF(CxxConfig, "Freeing sim object: %s\n", (*i)->name());
        delete *i;
    }

    for (auto i = objectParamsByName.rbegin();
        i != objectParamsByName.rend(); ++i)
    {
        CxxConfigParams *params = (*i).second;

        DPRINTF(CxxConfig, "Freeing sim object params: %s\n",
            params->getName());
        delete params;
    }

    objectsInOrder.clear();
    objectsByName.clear();
}

void
CxxConfigManager::setParam(const std::string &object_name,
    const std::string &param_name, const std::string &param_value)
{
    CxxConfigParams *params = findObjectParams(object_name);

    if (!params->setParam(param_name, param_value, flags)) {
        throw Exception(object_name, csprintf("Bad parameter value:"
            " .%s=X=\"%s\"", param_name, param_value));
    } else {
        std::string instance_name = rename(object_name);

        DPRINTF(CxxConfig, "Setting parameter %s.%s=%s\n",
            instance_name, param_name, param_value);
    }
}

void
CxxConfigManager::setParamVector(const std::string &object_name,
    const std::string &param_name,
    const std::vector<std::string> &param_values)
{
    CxxConfigParams *params = findObjectParams(object_name);

    if (!params->setParamVector(param_name, param_values, flags)) {
        throw Exception(object_name, csprintf("Bad vector parameter value:"
            " .%s=X=\"%s\"", param_name, formatParamList(param_values)));
    } else {
        std::string instance_name = rename(object_name);

        DPRINTF(CxxConfig, "Setting parameter %s.%s=\"%s\"\n",
            instance_name, param_name, formatParamList(param_values));
    }
}

void CxxConfigManager::addRenaming(const Renaming &renaming)
{
    renamings.push_back(renaming);
}
