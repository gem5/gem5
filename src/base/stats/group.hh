/*
 * Copyright (c) 2019, 2020 Arm Limited
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
 */

#ifndef __BASE_STATS_GROUP_HH__
#define __BASE_STATS_GROUP_HH__

#include <map>
#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/stats/units.hh"

namespace gem5
{

/**
 * Convenience macro to add a stat to a statistics group.
 *
 * This macro is used to add a stat to a statistics::Group in the
 * initilization list in the Group's constructor. The macro
 * automatically assigns the stat to the current group and gives it
 * the same name as in the class. For example:
 *
 * \code
 * struct MyStats : public statistics::Group
 * {
 *     statistics::Scalar scalar0;
 *     statistics::Scalar scalar1;
 *
 *     Group()
 *         : ADD_STAT(scalar0, "Description of scalar0"),
 *           scalar1(this, "scalar1", statistics::units::Unspecified::get(),
 *                   "Description of scalar1")
 *     {
 *     }
 * };
 * \endcode
 */

#define ADD_STAT(n, ...) n(this, #n, __VA_ARGS__)

namespace statistics
{

class Info;

/**
 * Statistics container.
 *
 * A stat group is a hierarchical structure that contain statistics
 * and other groups. Groups are used by the stat system to reflect
 * gem5's SimObject hierarchy and to expose internal hierarchy within
 * an object. They can also be used to conveniently group stats into
 * their own class/struct and then be merged into the parent group
 * (typically a SimObject).
 */
class Group
{
  public:
    /**
     * @ingroup api_stats
     * @{
     */
    Group() = delete;
    Group(const Group &) = delete;
    Group &operator=(const Group &) = delete;
    /** @}*/ //end of api_stats

    /**
     * Construct a new statistics group.
     *
     * The constructor takes two parameters, a parent and a name. The
     * parent group should typically be specified. However, there are
     * special cases where the parent group may be null. One such
     * special case is SimObjects where the Python code performs late
     * binding of the group parent.
     *
     * If the name parameter is NULL, the group gets merged into the
     * parent group instead of creating a sub-group. Stats belonging
     * to a merged group behave as if they have been added directly to
     * the parent group.
     *
     * @param parent Parent group to associate this object to.
     * @param name Name of this group, can be NULL to merge this group
     * with the parent group.
     *
     * @ingroup api_stats
     */
    Group(Group *parent, const char *name = nullptr);

    virtual ~Group();

    /**
     * Callback to set stat parameters.
     *
     * This callback is typically used for complex stats (e.g.,
     * distributions) that need parameters in addition to a name and a
     * description. Stat names and descriptions should typically be
     * set from the constructor usingo from the constructor using the
     * ADD_STAT macro.
     *
     * @ingroup api_stats
     */
    virtual void regStats();

    /**
     * Callback to reset stats.
     *
     * @ingroup api_stats
     */
    virtual void resetStats();

    /**
     * Callback before stats are dumped. This can be overridden by
     * objects that need to perform calculations in addition to the
     * capabiltiies implemented in the stat framework.
     *
     * @ingroup api_stats
     */
    virtual void preDumpStats();

    /**
     * Register a stat with this group. This method is normally called
     * automatically when a stat is instantiated.
     *
     * @ingroup api_stats
     */
    void addStat(statistics::Info *info);

    /**
     * Get all child groups associated with this object.
     *
     * @ingroup api_stats
     */
    const std::map<std::string, Group *> &getStatGroups() const;

    /**
     * Get all stats associated with this object.
     *
     * @ingroup api_stats
     */
    const std::vector<Info *> &getStats() const;

     /**
     * Add a stat block as a child of this block
     *
     * This method may only be called from a Group constructor or from
     * regStats. It's typically only called explicitly from Python
     * when setting up the SimObject hierarchy.
     *
     * @ingroup api_stats
     */
    void addStatGroup(const char *name, Group *block);

    /**
     * Resolve a stat by its name within this group.
     *
     * This method goes through the stats in this group and sub-groups
     * and returns a pointer to the the stat that matches the provided
     * name. The input name has to be relative to the name of this
     * group. For example, if this group is the SimObject
     * system.bigCluster.cpus and we want the stat
     * system.bigCluster.cpus.ipc, the input param should be the
     * string "ipc".
     *
     * @param name Name of the desired stat
     * @return Pointer to the stat with the provided name
     */
    const Info * resolveStat(std::string name) const;

    /**
     * Merge the contents (stats & children) of a block to this block.
     *
     * This is called on a parent group by the child when it is being
     * merged into the parent.
     */
    void mergeStatGroup(Group *block);

  private:
    /** Parent pointer if merged into parent */
    Group *mergedParent;

    std::map<std::string, Group *> statGroups;
    std::vector<Group *> mergedStatGroups;
    std::vector<Info *> stats;
};

} // namespace statistics
} // namespace gem5

#endif // __BASE_STATS_GROUP_HH__
