// Catch2
#include <catch2/catch_test_macros.hpp>

// UFO
#include <ufo/container/tree/predicate.hpp>
#include <ufo/utility/type_traits.hpp>


using namespace ufo;

TEST_CASE("Predicate Operator||")
{
    SECTION("Pred || Pred")
    {
        auto p1 = pred::Leaf();
        auto p2 = pred::Depth();
        
        auto p12 = p1 || p2;
        auto a = pred::Leaf() || pred::Depth();
        auto b = p1 || pred::Depth();
        auto c = pred::Leaf() || p2;
    }

    SECTION("Or (lvalue) || Or (lvalue)")
    {
        auto a = pred::Leaf() || pred::Depth();
        auto b = pred::Leaf() || pred::Modified();
        auto c = a || b;
    }

    SECTION("Or (rvalue) || Or (rvalue)")
    {
        auto c = (pred::Leaf() || pred::Depth()) ||
                 (pred::Leaf() || pred::Modified());
    }

    SECTION("Or (lvalue) || Pred")
    {
        auto a = pred::Leaf() || pred::Depth();
        auto b = pred::Modified();

        auto c = a || pred::Modified();
        auto d = a || b;
    }

    SECTION("Pred || Or (lvalue)")
    {
        auto b = pred::Leaf() || pred::Modified();
        auto l = pred::Modified();

        auto c = l || b;
        auto d = pred::Depth() || b;
    }
}

TEST_CASE("Predicate Operator&&")
{
    SECTION("Pred && Pred")
    {
        auto p1 = pred::Leaf();
        auto p2 = pred::Depth();
        
        auto p12 = p1 && p2;
        auto a = pred::Leaf() && pred::Depth();
        auto b = p1 && pred::Depth();
        auto c = pred::Leaf() && p2;
    }

    SECTION("And (lvalue) && And (lvalue)")
    {
        auto a = pred::Leaf() && pred::Depth();
        auto b = pred::Leaf() && pred::Modified();

        auto c = a && b;
    }

    SECTION("And (rvalue) && And (rvalue)")
    {
        auto c = (pred::Leaf() && pred::Depth()) &&
                 (pred::Leaf() && pred::Modified());
    }

    SECTION("And (lvalue) && Pred")
    {
        auto a = pred::Leaf() && pred::Depth();
        auto b = pred::Modified();

        auto c = a && pred::Modified();
        auto d = a && b;
    }

    SECTION("Pred && And (lvalue)")
    {
        auto b = pred::Leaf() && pred::Modified();
        auto l = pred::Modified();

        auto c = l && b;
        auto d = pred::Depth() && b;
    }
}
