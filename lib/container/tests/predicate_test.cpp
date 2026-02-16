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

		[[maybe_unused]] auto p12 = p1 || p2;
		[[maybe_unused]] auto a   = pred::Leaf() || pred::Depth();
		[[maybe_unused]] auto b   = p1 || pred::Depth();
		[[maybe_unused]] auto c   = pred::Leaf() || p2;
	}

	SECTION("Or (lvalue) || Or (lvalue)")
	{
		auto                  a = pred::Leaf() || pred::Depth();
		auto                  b = pred::Leaf() || pred::Modified();
		[[maybe_unused]] auto c = a || b;
	}

	SECTION("Or (rvalue) || Or (rvalue)")
	{
		[[maybe_unused]] auto c =
		    (pred::Leaf() || pred::Depth()) || (pred::Leaf() || pred::Modified());
	}

	SECTION("Or (lvalue) || Pred")
	{
		auto a = pred::Leaf() || pred::Depth();
		auto b = pred::Modified();

		[[maybe_unused]] auto c = a || pred::Modified();
		[[maybe_unused]] auto d = a || b;
	}

	SECTION("Pred || Or (lvalue)")
	{
		auto b = pred::Leaf() || pred::Modified();
		auto l = pred::Modified();

		[[maybe_unused]] auto c = l || b;
		[[maybe_unused]] auto d = pred::Depth() || b;
	}
}

TEST_CASE("Predicate Operator&&")
{
	SECTION("Pred && Pred")
	{
		auto p1 = pred::Leaf();
		auto p2 = pred::Depth();

		[[maybe_unused]] auto p12 = p1 && p2;
		[[maybe_unused]] auto a   = pred::Leaf() && pred::Depth();
		[[maybe_unused]] auto b   = p1 && pred::Depth();
		[[maybe_unused]] auto c   = pred::Leaf() && p2;
	}

	SECTION("And (lvalue) && And (lvalue)")
	{
		auto a = pred::Leaf() && pred::Depth();
		auto b = pred::Leaf() && pred::Modified();

		[[maybe_unused]] auto c = a && b;
	}

	SECTION("And (rvalue) && And (rvalue)")
	{
		[[maybe_unused]] auto c =
		    (pred::Leaf() && pred::Depth()) && (pred::Leaf() && pred::Modified());
	}

	SECTION("And (lvalue) && Pred")
	{
		auto a = pred::Leaf() && pred::Depth();
		auto b = pred::Modified();

		[[maybe_unused]] auto c = a && pred::Modified();
		[[maybe_unused]] auto d = a && b;
	}

	SECTION("Pred && And (lvalue)")
	{
		auto b = pred::Leaf() && pred::Modified();
		auto l = pred::Modified();

		[[maybe_unused]] auto c = l && b;
		[[maybe_unused]] auto d = pred::Depth() && b;
	}
}
