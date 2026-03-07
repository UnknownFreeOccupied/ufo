// UFO
#include <ufo/geometry/distance.hpp>
#include <ufo/geometry/intersects.hpp>
#include <ufo/geometry/line.hpp>

// STL
#include <iostream>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Line 2D]")
{
	ufo::Vec2f a(-1, -1);
	ufo::Vec2f b(1, 1);

	ufo::Line<2> l(a, b);

	// Direction should be normalized (1, 1) / sqrt(2)
	REQUIRE(l.origin == a);
	REQUIRE(l.direction.x() == Catch::Approx(std::sqrt(2) / 2.0f));
	REQUIRE(l.direction.y() == Catch::Approx(std::sqrt(2) / 2.0f));
}

TEST_CASE("[Line 2D] distance")
{
	ufo::Vec2f a(-1, -1);
	ufo::Vec2f b(1, 1);

	ufo::Line<2> l(a, b);

	auto d1 = ufo::distance(l, a);
	REQUIRE(d1 == Catch::Approx(0));

	auto d2 = ufo::distance(l, ufo::Vec2f(0, 1));
	// Closest point on line y=x to (0,1) is (0.5, 0.5)
	// Distance is sqrt(0.5^2 + 0.5^2) = sqrt(0.5) = 1/sqrt(2) = sqrt(2)/2
	REQUIRE(d2 == Catch::Approx(std::sqrt(2) / 2.0f));
}

TEST_CASE("[Line 2D] intersects")
{
	SECTION("origin")
	{
		ufo::Vec2f   a1(-1, -1);
		ufo::Vec2f   b1(1, 1);
		ufo::Line<2> l1(a1, b1);

		ufo::Vec2f   a2(-1, 1);
		ufo::Vec2f   b2(1, -1);
		ufo::Line<2> l2(a2, b2);

		REQUIRE(ufo::intersects(l1, l2));
		auto pt = ufo::intersectionPoint(l1, l2);
		REQUIRE(pt.x() == Catch::Approx(0));
		REQUIRE(pt.y() == Catch::Approx(0));
	}

	SECTION("not origin")
	{
		ufo::Vec2f   a1(0, -1);
		ufo::Vec2f   b1(1, 0);  // y = x - 1
		ufo::Line<2> l1(a1, b1);

		ufo::Vec2f   a2(2, 2);
		ufo::Vec2f   b2(2, 0);  // x = 2
		ufo::Line<2> l2(a2, b2);

		REQUIRE(ufo::intersects(l1, l2));
		auto pt = ufo::intersectionPoint(l1, l2);
		REQUIRE(pt.x() == Catch::Approx(2));
		REQUIRE(pt.y() == Catch::Approx(1));
	}
}
