#include <gtest/gtest.h>

#include "Common/Handle.h"
#include "Gameplay/GameplayTags.h"
#include "Shared/Config/BuildConfig.h"

#include <functional>

TEST(BuildConfigTest, ExposesEngineIdentity) {
    EXPECT_STREQ(Shared::Config::BuildConfig::EngineName(), "MyGameEngine");
    EXPECT_STREQ(Shared::Config::BuildConfig::EngineVersion(), "0.1.0");
}

TEST(HandleTest, SupportsValidityValueAndComparison) {
    constexpr Runtime::Handle invalid;
    constexpr Runtime::Handle first{42};
    constexpr Runtime::Handle same{42};
    constexpr Runtime::Handle later{84};

    static_assert(!invalid.IsValid());
    static_assert(first.IsValid());
    static_assert(first.Value() == 42);
    static_assert(first == same);
    static_assert(first != later);
    static_assert(first < later);

    EXPECT_EQ(std::hash<Runtime::Handle>{}(first), std::hash<Runtime::Handle>{}(same));
}

TEST(GameplayTagTest, RegistersAndFindsTags) {
    auto& registry = Runtime::Gameplay::GameplayTagRegistry::Instance();

    const auto playerTag = registry.RegisterTag("Test.CoreTypes.Player");
    const auto sameTag = registry.RegisterTag("Test.CoreTypes.Player");

    EXPECT_TRUE(playerTag.IsValid());
    EXPECT_EQ(playerTag, sameTag);
    EXPECT_EQ(registry.FindTag("Test.CoreTypes.Player"), playerTag);
    EXPECT_EQ(registry.FindName(playerTag), "Test.CoreTypes.Player");
    EXPECT_FALSE(registry.FindTag("Test.CoreTypes.Missing").IsValid());
}