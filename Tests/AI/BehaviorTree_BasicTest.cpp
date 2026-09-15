#include <gtest/gtest.h>

#include "AI/BehaviorTree/BehaviorTree.h"
#include "AI/BehaviorTree/Blackboard.h"

#include <string>

using Runtime::AI::BehaviorTree::BehaviorTree;
using Runtime::AI::BehaviorTree::Blackboard;

TEST(BlackboardTest, StoresFindsAndRemovesTypedValues) {
    Blackboard blackboard;
    blackboard.SetValue("health", int64_t{100});

    EXPECT_TRUE(blackboard.HasKey("health"));
    EXPECT_EQ(blackboard.GetValue<int64_t>("health"), 100);
    EXPECT_FALSE(blackboard.GetValue<std::string>("health").has_value());
    EXPECT_FALSE(blackboard.GetValue<int64_t>("missing").has_value());

    blackboard.RemoveKey("health");
    EXPECT_FALSE(blackboard.HasKey("health"));
}

TEST(BehaviorTreeTest, ExposesItsBlackboard) {
    BehaviorTree tree;
    tree.Data().SetValue("ready", true);

    const BehaviorTree& constTree = tree;
    EXPECT_EQ(constTree.Data().GetValue<bool>("ready"), true);
}