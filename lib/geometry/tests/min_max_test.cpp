// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/geometry/capsule.hpp>
#include <ufo/geometry/cylinder.hpp>
#include <ufo/geometry/fun.hpp>
#include <ufo/geometry/line.hpp>
#include <ufo/geometry/plane.hpp>
#include <ufo/geometry/ray.hpp>
#include <ufo/geometry/sphere.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ufo;

TEST_CASE("Geometry min/max")
{
	SECTION("Cylinder")
	{
		Cylinder3f c(Vec3f(0, 0, 0), Vec3f(0, 0, 1), 0.5f);
		auto       mi = min(c);
		auto       ma = max(c);

		CHECK(mi.x() == -0.5f);
		CHECK(mi.y() == -0.5f);
		CHECK(mi.z() == 0.0f);

		CHECK(ma.x() == 0.5f);
		CHECK(ma.y() == 0.5f);
		CHECK(ma.z() == 1.0f);
	}

	SECTION("Ray")
	{
		Ray3f r(Vec3f(1, 2, 3), Vec3f(1, 0, -1));
		auto  mi = min(r);
		auto  ma = max(r);

		CHECK(mi.x() == 1.0f);
		CHECK(mi.y() == 2.0f);
		CHECK(mi.z() == -std::numeric_limits<float>::infinity());

		CHECK(ma.x() == std::numeric_limits<float>::infinity());
		CHECK(ma.y() == 2.0f);
		CHECK(ma.z() == 3.0f);
	}

	SECTION("Line")
	{
		Line2f l(Vec2f(5, 0), Vec2f(0, 1));  // Origin (5,0), direction (0,1) -> x=5
		auto   mi = min(l);
		auto   ma = max(l);

		CHECK(mi.x() == -std::numeric_limits<float>::infinity());
		CHECK(mi.y() == -std::numeric_limits<float>::infinity());
		CHECK(ma.x() == std::numeric_limits<float>::infinity());
		CHECK(ma.y() == std::numeric_limits<float>::infinity());
	}

	SECTION("Plane")
	{
		Plane3f p(Vec3f(0, 0, 1), 10.0f);  // z = 10
		auto    mi = min(p);
		auto    ma = max(p);

		CHECK(mi.x() == -std::numeric_limits<float>::infinity());
		CHECK(ma.z() == std::numeric_limits<float>::infinity());
	}
}
