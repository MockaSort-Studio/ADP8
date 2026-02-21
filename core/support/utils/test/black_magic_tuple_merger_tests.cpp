#include <tuple>
#include <type_traits>

#include <gtest/gtest.h>

#include "core/support/utils/black_magic_tuple_merger.hpp"
namespace core::utils {

TEST(BlackMagicMergerTest, MergeUniqueTuples)
{
    // Given
    using TupleA = std::tuple<int, float>;
    using TupleB = std::tuple<double, char>;

    // When
    using Result = typename BlackMagicTupleMerger<TupleA, TupleB>::type;

    // Then
    using Expected = std::tuple<int, float, double, char>;
    ASSERT_TRUE((std::is_same_v<Expected, Result>));
}

TEST(BlackMagicMergerTest, MergeWithDuplicatesAcrossTuples)
{
    // Given
    using TupleA = std::tuple<int, float, double>;
    using TupleB = std::tuple<float, char, int>;

    // When
    using Result = typename BlackMagicTupleMerger<TupleA, TupleB>::type;

    // Then
    using Expected = std::tuple<int, float, double, char>;
    ASSERT_TRUE((std::is_same_v<Expected, Result>));
}

TEST(BlackMagicMergerTest, MergeMultipleTuples)
{
    // Given
    using T1 = std::tuple<int>;
    using T2 = std::tuple<int, float>;
    using T3 = std::tuple<float, double>;
    using T4 = std::tuple<char>;

    // When
    using Result = typename BlackMagicTupleMerger<T1, T2, T3, T4>::type;

    // Then
    using Expected = std::tuple<int, float, double, char>;
    ASSERT_TRUE((std::is_same_v<Expected, Result>));
}

TEST(BlackMagicMergerTest, HandleEmptyTuples)
{
    // Given
    using T1 = std::tuple<>;
    using T2 = std::tuple<int, float>;
    using T3 = std::tuple<>;

    // When
    using Result = typename BlackMagicTupleMerger<T1, T2, T3>::type;

    // Then
    using Expected = std::tuple<int, float>;
    ASSERT_TRUE((std::is_same_v<Expected, Result>));
}

}  // namespace core::utils