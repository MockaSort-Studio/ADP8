#ifndef SUPPORT_VISIT_TEST_TEST_VISITORS
#define SUPPORT_VISIT_TEST_TEST_VISITORS

#include <iostream>
#include <vector>

#include "support/visit/visitor_base.hpp"

namespace game::test
{

struct SimpleVisitor final : public game::support::VisitorBase<SimpleVisitor>
{
    template <typename T>
    void visit(std::string_view name, const T& value)
    {
        std::cout << name << std::endl;
    }

    void visit(std::string_view name, const int& value)
    {
        c_result.first = name;
        c_result.second = value;
    }

    void visit(std::string_view name, const double& value)
    {
        d_result.first = name;
        d_result.second = value;
    }

    void visit(std::string_view name, const std::string& value)
    {
        s_result.first = name;
        s_result.second = value;
    }

    std::pair<std::string_view, int> c_result {};
    std::pair<std::string_view, double> d_result {};
    std::pair<std::string_view, std::string> s_result {};
};

struct NestedVisitor final : public game::support::VisitorBase<NestedVisitor>
{

    template <typename T>
    void visit(std::string_view name, const T& value)
    {
        std::cout << name << std::endl;
    }

    void visit_nested(std::string_view name)
    {
        enter_names.push_back(name);
        in_nested = true;
    }
    void exit_nested(std::string_view name)
    {
        exit_names.push_back(name);
        in_nested = false;
    }

    void visit(std::string_view name, const int& value)
    {
        if (not in_nested)
        {
            return;
        }
        c_result.first = name;
        c_result.second = value;
    }

    void visit(std::string_view name, const double& value)
    {
        if (not in_nested)
        {
            return;
        }
        d_result.first = name;
        d_result.second = value;
    }

    void visit(std::string_view name, const std::string& value)
    {
        if (not in_nested)
        {
            return;
        }
        s_result.first = name;
        s_result.second = value;
    }

    std::vector<std::string_view> enter_names {};
    std::vector<std::string_view> exit_names {};
    std::pair<std::string_view, int> c_result {};
    std::pair<std::string_view, double> d_result {};
    std::pair<std::string_view, std::string> s_result {};

  private:
    bool in_nested {false};
};

}  // namespace game::test

#endif // SUPPORT_VISIT_TEST_TEST_VISITORS
