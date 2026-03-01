
#include <gtest/gtest.h>

#include "core/lifecycle/parameters_provider.hpp"
#include "core/support/utils/lookup_table.hpp"

namespace core::lifecycle {
struct kMocka {};
struct kSort {};

using Params = utils::LookupTable<
    utils::TableItem<kMocka, bool>,       
    utils::TableItem<kSort, float, float>>;

struct ParamsDefaults {
    static constexpr auto values = utils::TableDefaults<Params>(
        utils::Init{kMocka{}, true}, 
        utils::Init{kSort{}, 1.5f, 0.5f}          
    );
};

TEST(ParametersTest, VerifyGetFunctional) {

    ParametersProvider<Params, ParamsDefaults> params;

    auto single_value = params.GetParameterValue<kMocka>();
    
    EXPECT_TRUE(single_value);

    auto array_value = params.GetParameterValue<kSort>();
    
    EXPECT_FLOAT_EQ(array_value[0], 1.5f);
    EXPECT_FLOAT_EQ(array_value[1], 0.5f);
}


}  // namespace core::lifecycle