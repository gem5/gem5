/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * All rights reserved
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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "base/stats/group.hh"
#include "base/stats/info.hh"
#include "base/stats/output.hh"

using namespace gem5;

/** Test that the constructor without a parent doesn't do anything. */
TEST(StatsGroupTest, ConstructNoParent)
{
    Stats::Group root(nullptr);
    ASSERT_EQ(root.getStatGroups().size(), 0);
}

/** Test adding a single stat group to a root node. */
TEST(StatsGroupTest, AddGetSingleStatGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    root.addStatGroup("Node1", &node1);

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 1);
    ASSERT_NE(root_map.find("Node1"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 0);
}

/** Test that group names are unique within a node's stat group. */
TEST(StatsGroupDeathTest, AddUniqueNameStatGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node2(nullptr);
    root.addStatGroup("Node1", &node1);
    ASSERT_ANY_THROW(root.addStatGroup("Node1", &node2));
}

/** Test that group names are not unique among two nodes' stat groups. */
TEST(StatsGroupTest, AddNotUniqueNameAmongGroups)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node2(nullptr);
    Stats::Group node1_1(nullptr);
    root.addStatGroup("Node1", &node1);
    node1.addStatGroup("Node1_1", &node1_1);
    ASSERT_NO_THROW(node1.addStatGroup("Node1", &node2));
}

/** Test that a group cannot add a non-existent group. */
TEST(StatsGroupDeathTest, AddNull)
{
    Stats::Group root(nullptr);
    ASSERT_ANY_THROW(root.addStatGroup("Node1", nullptr));
}

/** Test that a group cannot add itself. */
TEST(StatsGroupDeathTest, AddItself)
{
    Stats::Group root(nullptr);
    ASSERT_ANY_THROW(root.addStatGroup("Node1", &root));
}

/** @todo Test that a group cannot be added in a cycle. */
TEST(StatsGroupDeathTest, DISABLED_AddCycle)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node1_1(nullptr);
    root.addStatGroup("Node1", &node1);
    node1.addStatGroup("Node1_1", &node1_1);
    ASSERT_ANY_THROW(node1_1.addStatGroup("Root", &root));
}

/** Test adding multiple stat groups to a root node. */
TEST(StatsGroupTest, AddGetMultipleStatGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node2(nullptr);
    root.addStatGroup("Node1", &node1);
    root.addStatGroup("Node2", &node2);

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 2);
    ASSERT_NE(root_map.find("Node1"), root_map.end());
    ASSERT_NE(root_map.find("Node2"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 0);
    ASSERT_EQ(node2.getStatGroups().size(), 0);
}

/** Make sure that the groups are correctly assigned in the map. */
TEST(StatsGroupTest, ConstructCorrectlyAssigned)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node1_1(nullptr);
    Stats::Group node1_1_1(nullptr);
    root.addStatGroup("Node1", &node1);
    node1.addStatGroup("Node1_1", &node1_1);
    node1_1.addStatGroup("Node1_1_1", &node1_1_1);

    ASSERT_EQ(node1.getStatGroups().find("Node1_1")->second->getStatGroups(),
        node1_1.getStatGroups());
}

/**
 * Test that the constructor, when provided both the parent and the nodes'
 * name, creates the following tree:
 *
 * root
 *   |
 * node1
 */
TEST(StatsGroupTest, ConstructOneLevelLinear)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 1);
    ASSERT_NE(root_map.find("Node1"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 0);
}

/**
 * Test that the constructor, when provided both the parent and the nodes'
 * name, creates the following tree:
 *
 *    root
 *   /    \
 * node1 node2
 */
TEST(StatsGroupTest, ConstructOneLevelOfTwoNodes)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");
    Stats::Group node2(&root, "Node2");

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 2);
    ASSERT_NE(root_map.find("Node1"), root_map.end());
    ASSERT_NE(root_map.find("Node2"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 0);
    ASSERT_EQ(node2.getStatGroups().size(), 0);
}

/**
 * Test that the constructor, when provided both the parent and the nodes'
 * name, creates the following tree:
 *
 * root
 *   |
 * node1
 *   |
 * node1_1
 */
TEST(StatsGroupTest, ConstructTwoLevelsLinear)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");
    Stats::Group node1_1(&node1, "Node1_1");

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 1);
    ASSERT_NE(root_map.find("Node1"), root_map.end());
    ASSERT_EQ(root_map.find("Node1_1"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 1);
    ASSERT_NE(node1.getStatGroups().find("Node1_1"),
        node1.getStatGroups().end());

    ASSERT_EQ(node1_1.getStatGroups().size(), 0);
}

/**
 * Test that the constructor, when provided both the parent and the nodes'
 * name, creates the following tree:
 *
 *        root
 *       /     \
 *  node1       node2
 *    |        /     \
 * node1_1  node2_1 node2_2
 */
TEST(StatsGroupTest, ConstructTwoLevelsUnbalancedTree)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");
    Stats::Group node2(&root, "Node2");
    Stats::Group node1_1(&node1, "Node1_1");
    Stats::Group node2_1(&node2, "Node2_1");
    Stats::Group node2_2(&node2, "Node2_2");

    const auto root_map = root.getStatGroups();
    ASSERT_EQ(root_map.size(), 2);
    ASSERT_NE(root_map.find("Node1"), root_map.end());
    ASSERT_NE(root_map.find("Node2"), root_map.end());
    ASSERT_EQ(root_map.find("Node1_1"), root_map.end());
    ASSERT_EQ(root_map.find("Node2_1"), root_map.end());
    ASSERT_EQ(root_map.find("Node2_2"), root_map.end());

    ASSERT_EQ(node1.getStatGroups().size(), 1);
    ASSERT_NE(node1.getStatGroups().find("Node1_1"),
        node1.getStatGroups().end());
    ASSERT_EQ(node1.getStatGroups().find("Node2_1"),
        node1.getStatGroups().end());
    ASSERT_EQ(node1.getStatGroups().find("Node2_2"),
        node1.getStatGroups().end());

    ASSERT_EQ(node2.getStatGroups().size(), 2);
    ASSERT_EQ(node2.getStatGroups().find("Node1_1"),
        node2.getStatGroups().end());
    ASSERT_NE(node2.getStatGroups().find("Node2_1"),
        node2.getStatGroups().end());
    ASSERT_NE(node2.getStatGroups().find("Node2_2"),
        node2.getStatGroups().end());

    ASSERT_EQ(node1_1.getStatGroups().size(), 0);
    ASSERT_EQ(node2_1.getStatGroups().size(), 0);
    ASSERT_EQ(node2_2.getStatGroups().size(), 0);
}

class DummyInfo : public Stats::Info
{
  public:
    using Stats::Info::Info;

    int value = 0;

    bool check() const override { return true; }
    void prepare() override {}
    void reset() override { value = 0; }
    bool zero() const override { return false; }
    void visit(Stats::Output &visitor) override {}
};

/** Test adding stats to a group. */
TEST(StatsGroupTest, AddGetStat)
{
    Stats::Group root(nullptr);
    auto info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 0);

    DummyInfo info;
    info.setName("InfoAddGetStat");
    root.addStat(&info);
    info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 1);
    ASSERT_EQ(info_vec[0]->name, "InfoAddGetStat");

    DummyInfo info2;
    info2.setName("InfoAddGetStat2");
    root.addStat(&info2);
    info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 2);
    ASSERT_EQ(info_vec[0]->name, "InfoAddGetStat");
    ASSERT_EQ(info_vec[1]->name, "InfoAddGetStat2");
}

/** Test that a group cannot merge if another group is not provided. */
TEST(StatsGroupDeathTest, MergeStatGroupNoGroup)
{
    Stats::Group root(nullptr);
    ASSERT_ANY_THROW(root.mergeStatGroup(nullptr));
}

/** Test that a group cannot merge with itself. */
TEST(StatsGroupDeathTest, MergeStatGroupItself)
{
    Stats::Group root(nullptr);
    ASSERT_ANY_THROW(root.mergeStatGroup(&root));
}

/** Test merging groups. */
TEST(StatsGroupTest, MergeStatGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);

    DummyInfo info;
    info.setName("InfoMergeStatGroup");
    node1.addStat(&info);
    DummyInfo info2;
    info2.setName("InfoMergeStatGroup2");
    node1.addStat(&info2);

    root.mergeStatGroup(&node1);
    auto info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 2);
    ASSERT_EQ(info_vec[0]->name, "InfoMergeStatGroup");
    ASSERT_EQ(info_vec[1]->name, "InfoMergeStatGroup2");
}

/** Test that a group that has already been merged cannot be merged again. */
TEST(StatsGroupDeathTest, MergeStatGroupMergedParent)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node2(nullptr);
    root.mergeStatGroup(&node2);
    ASSERT_ANY_THROW(node1.mergeStatGroup(&node2));
}

/**
 * Test that after a group has been merged, adding stats to it will add
 * stats to the group it was merged to too.
 */
TEST(StatsGroupTest, AddStatMergedParent)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node1_1(nullptr);

    root.mergeStatGroup(&node1);
    root.mergeStatGroup(&node1_1);

    DummyInfo info;
    info.setName("AddStatMergedParent");
    node1_1.addStat(&info);

    auto info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 1);
    ASSERT_EQ(info_vec[0]->name, "AddStatMergedParent");
    info_vec = node1.getStats();
    ASSERT_EQ(info_vec.size(), 0);
    info_vec = node1_1.getStats();
    ASSERT_EQ(info_vec.size(), 1);
    ASSERT_EQ(info_vec[0]->name, "AddStatMergedParent");
}

/**
 * Test that after a group has been merged, adding stats to the "main" group
 * does not add stats to the group it was merged to.
 */
TEST(StatsGroupTest, AddStatMergedParentMain)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);

    root.mergeStatGroup(&node1);

    DummyInfo info;
    info.setName("AddStatMergedParentMain");
    root.addStat(&info);

    auto info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 1);
    ASSERT_EQ(info_vec[0]->name, "AddStatMergedParentMain");
    info_vec = node1.getStats();
    ASSERT_EQ(info_vec.size(), 0);
}

/**
 * Test that calling the constructor with a parent, but no name merges the
 * groups.
 */
TEST(StatsGroupTest, ConstructNoName)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root);

    DummyInfo info;
    info.setName("InfoConstructNoName");
    node1.addStat(&info);

    auto info_vec = root.getStats();
    ASSERT_EQ(info_vec.size(), 1);
    ASSERT_EQ(info_vec[0]->name, "InfoConstructNoName");
}

/**
 * Test that calling regStats calls the respective function of all sub-groups
 * and merged groups.
 */
TEST(StatsGroupTest, RegStats)
{
    class TestGroup : public Stats::Group
    {
      public:
        using Stats::Group::Group;

        int value = 0;

        void
        regStats() override
        {
            value++;
            Stats::Group::regStats();
        }
    };

    TestGroup root(nullptr);
    root.value = 1;
    TestGroup node1(&root, "Node1");
    node1.value = 2;
    TestGroup node1_1(&node1, "Node1_1");
    node1_1.value = 3;
    TestGroup node1_2(&node1_1);
    node1_2.value = 4;

    node1.regStats();
    ASSERT_EQ(root.value, 1);
    ASSERT_EQ(node1.value, 3);
    ASSERT_EQ(node1_1.value, 4);
    ASSERT_EQ(node1_2.value, 5);
}

/**
 * Test that resetting a stat from a specific node reset the stats of all its
 * sub-groups and merged groups, and that it does not reset the stats of its
 * parents.
 */
TEST(StatsGroupTest, ResetStats)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");
    Stats::Group node1_1(&node1, "Node1_1");
    Stats::Group node1_2(&node1_1);

    DummyInfo info;
    info.setName("InfoResetStats");
    info.value = 1;
    root.addStat(&info);

    DummyInfo info2;
    info2.setName("InfoResetStats2");
    info2.value = 2;
    node1.addStat(&info2);

    DummyInfo info3;
    info3.setName("InfoResetStats3");
    info3.value = 3;
    node1_1.addStat(&info3);

    DummyInfo info4;
    info4.setName("InfoResetStats4");
    info4.value = 4;
    node1_1.addStat(&info4);

    DummyInfo info5;
    info5.setName("InfoResetStats5");
    info5.value = 5;
    node1_2.addStat(&info5);

    node1.resetStats();
    ASSERT_EQ(info.value, 1);
    ASSERT_EQ(info2.value, 0);
    ASSERT_EQ(info3.value, 0);
    ASSERT_EQ(info4.value, 0);
    ASSERT_EQ(info5.value, 0);
}

/**
 * Test that calling preDumpStats calls the respective function of all sub-
 * groups and merged groups.
 */
TEST(StatsGroupTest, PreDumpStats)
{
    class TestGroup : public Stats::Group
    {
      public:
        using Stats::Group::Group;

        int value = 0;

        void
        preDumpStats() override
        {
            value++;
            Stats::Group::preDumpStats();
        }
    };

    TestGroup root(nullptr);
    root.value = 1;
    TestGroup node1(&root, "Node1");
    node1.value = 2;
    TestGroup node1_1(&node1, "Node1_1");
    node1_1.value = 3;
    TestGroup node1_2(&node1_1);
    node1_2.value = 4;

    node1.preDumpStats();
    ASSERT_EQ(root.value, 1);
    ASSERT_EQ(node1.value, 3);
    ASSERT_EQ(node1_1.value, 4);
    ASSERT_EQ(node1_2.value, 5);
}

/** Test that resolving a non-existent stat returns a nullptr. */
TEST(StatsGroupTest, ResolveStatNone)
{
    Stats::Group root(nullptr);

    DummyInfo info;
    info.setName("InfoResolveStatNone");
    root.addStat(&info);

    auto info_found = root.resolveStat("InfoResolveStatAny");
    ASSERT_EQ(info_found, nullptr);
}

/** Test resolving a stat belonging to the caller group. */
TEST(StatsGroupTest, ResolveStatSelf)
{
    Stats::Group root(nullptr);

    DummyInfo info;
    info.setName("InfoResolveStatSelf");
    root.addStat(&info);

    DummyInfo info2;
    info2.setName("InfoResolveStatSelf2");
    root.addStat(&info2);

    DummyInfo info3;
    info3.setName("InfoResolveStatSelf3");
    root.addStat(&info3);

    auto info_found = root.resolveStat("InfoResolveStatSelf");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatSelf");

    info_found = root.resolveStat("InfoResolveStatSelf2");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatSelf2");

    info_found = root.resolveStat("InfoResolveStatSelf3");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatSelf3");
}

/** Test that resolving stats from sub-groups is possible. */
TEST(StatsGroupTest, ResolveSubGroupStatFromParent)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");
    Stats::Group node1_1(&node1, "Node1_1");
    Stats::Group node1_1_1(&node1_1, "Node1_1_1");

    DummyInfo info;
    info.setName("InfoResolveSubGroupStatFromParent");
    node1.addStat(&info);

    DummyInfo info2;
    info2.setName("InfoResolveSubGroupStatFromParent2");
    node1_1.addStat(&info2);

    DummyInfo info3;
    info3.setName("InfoResolveSubGroupStatFromParent3");
    node1_1_1.addStat(&info3);

    auto info_found =
        root.resolveStat("Node1.InfoResolveSubGroupStatFromParent");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveSubGroupStatFromParent");

    info_found = root.resolveStat(
        "Node1.Node1_1.InfoResolveSubGroupStatFromParent2");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveSubGroupStatFromParent2");

    info_found = root.resolveStat(
        "Node1.Node1_1.Node1_1_1.InfoResolveSubGroupStatFromParent3");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveSubGroupStatFromParent3");
}

/** Test that resolving a stat from the parent is not possible. */
TEST(StatsGroupTest, ResolveStatSubGroupOnSubGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(&root, "Node1");

    DummyInfo info;
    info.setName("InfoResolveStatSubGroupOnSubGroup");
    root.addStat(&info);

    auto info_found = node1.resolveStat("InfoResolveStatSubGroupOnSubGroup");
    ASSERT_EQ(info_found, nullptr);
}

/** Test that resolving a merged stat is possible. */
TEST(StatsGroupTest, ResolveStatMerged)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);

    DummyInfo info;
    info.setName("InfoResolveStatMerged");
    node1.addStat(&info);
    DummyInfo info2;
    info2.setName("InfoResolveStatMerged2");
    node1.addStat(&info2);

    root.mergeStatGroup(&node1);

    auto info_found = root.resolveStat("InfoResolveStatMerged");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatMerged");

    info_found = root.resolveStat("InfoResolveStatMerged2");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatMerged2");
}

/** Test that resolving a stat belonging to a merged sub-group is possible. */
TEST(StatsGroupTest, ResolveStatMergedSubGroup)
{
    Stats::Group root(nullptr);
    Stats::Group node1(nullptr);
    Stats::Group node2(nullptr);

    DummyInfo info;
    info.setName("InfoResolveStatMergedSubGroup");
    node2.addStat(&info);

    root.addStatGroup("Node1", &node1);
    node1.mergeStatGroup(&node2);

    auto info_found = root.resolveStat("Node1.InfoResolveStatMergedSubGroup");
    ASSERT_NE(info_found, nullptr);
    ASSERT_EQ(info_found->name, "InfoResolveStatMergedSubGroup");
}
