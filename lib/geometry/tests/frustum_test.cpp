// UFO
#include <ufo/geometry/frustum.hpp>
#include <ufo/geometry/fun.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Frustum 2D", "[Frustum][2D]")
{
	SECTION("Constructor from corners")
	{
		ufo::Vec2f a(2, 2);
		ufo::Vec2f b(-2, 2);
		ufo::Vec2f c(-1, 1);
		ufo::Vec2f d(1, 1);

		ufo::Frustum<2> f(a, b, c, d);

		auto cs = ufo::corners(f);

		REQUIRE(cs[0].x() == Catch::Approx(a.x()));
		REQUIRE(cs[0].y() == Catch::Approx(a.y()));

		REQUIRE(cs[1].x() == Catch::Approx(b.x()));
		REQUIRE(cs[1].y() == Catch::Approx(b.y()));

		REQUIRE(cs[2].x() == Catch::Approx(c.x()));
		REQUIRE(cs[2].y() == Catch::Approx(c.y()));

		REQUIRE(cs[3].x() == Catch::Approx(d.x()));
		REQUIRE(cs[3].y() == Catch::Approx(d.y()));
	}

	SECTION("min/max")
	{
		ufo::Vec2f a(2, 2);
		ufo::Vec2f b(-2, 2);
		ufo::Vec2f c(-1, 1);
		ufo::Vec2f d(1, 1);

		ufo::Frustum<2> f(a, b, c, d);

		auto min_result = ufo::min(f);
		auto max_result = ufo::max(f);

		ufo::Vec2f min_expected(-2, 1);
		ufo::Vec2f max_expected(2, 2);

		REQUIRE(min_result.x() == Catch::Approx(min_expected.x()));
		REQUIRE(min_result.y() == Catch::Approx(min_expected.y()));

		REQUIRE(max_result.x() == Catch::Approx(max_expected.x()));
		REQUIRE(max_result.y() == Catch::Approx(max_expected.y()));
	}

	SECTION("Position and target from origin")
	{
		ufo::Vec2f     pos(0, 0);
		ufo::Vec2f     target(0, 1);
		float          fov       = 1.57079632679f;  // 90 degrees
		float          near_dist = 1.0f;
		float          far_dist  = 3.0f;
		ufo::Frustum2f f(pos, target, fov, near_dist, far_dist);

		auto cs = ufo::corners(f);

		std::array<ufo::Vec2f, 4> expected{ufo::Vec2f(3, 3), ufo::Vec2f(-3, 3),
		                                   ufo::Vec2f(-1, 1), ufo::Vec2f(1, 1)};

		REQUIRE(cs[0].x() == Catch::Approx(expected[0].x()));
		REQUIRE(cs[0].y() == Catch::Approx(expected[0].y()));
		REQUIRE(cs[1].x() == Catch::Approx(expected[1].x()));
		REQUIRE(cs[1].y() == Catch::Approx(expected[1].y()));
		REQUIRE(cs[2].x() == Catch::Approx(expected[2].x()));
		REQUIRE(cs[2].y() == Catch::Approx(expected[2].y()));
		REQUIRE(cs[3].x() == Catch::Approx(expected[3].x()));
		REQUIRE(cs[3].y() == Catch::Approx(expected[3].y()));
	}

	SECTION("Position and target rotated")
	{
		ufo::Vec2f     eye(0.5, 0.5);
		ufo::Vec2f     target(1.5, 1.5);
		float          fov       = 1.57079632679f;  // 90 degrees
		float          near_dist = 1.0f;
		float          far_dist  = 3.0f;
		ufo::Frustum2f f(eye, target, fov, near_dist, far_dist);

		auto cs = ufo::corners(f);

		std::array<ufo::Vec2f, 4> expected{
		    ufo::Vec2f(4.74264f, 0.5f), ufo::Vec2f(0.5f, 4.74264f),
		    ufo::Vec2f(0.5f, 1.91421f), ufo::Vec2f(1.91421f, 0.5f)};

		REQUIRE(cs[0].x() == Catch::Approx(expected[0].x()));
		REQUIRE(cs[0].y() == Catch::Approx(expected[0].y()));
		REQUIRE(cs[1].x() == Catch::Approx(expected[1].x()));
		REQUIRE(cs[1].y() == Catch::Approx(expected[1].y()));
		REQUIRE(cs[2].x() == Catch::Approx(expected[2].x()));
		REQUIRE(cs[2].y() == Catch::Approx(expected[2].y()));
		REQUIRE(cs[3].x() == Catch::Approx(expected[3].x()));
		REQUIRE(cs[3].y() == Catch::Approx(expected[3].y()));
	}
}

TEST_CASE("Frustum 3D", "[Frustum][3D]")
{
	SECTION("Constructor from corners")
	{
		ufo::Vec3f near_top_right(1, 1, 1);
		ufo::Vec3f near_top_left(-1, 1, 1);
		ufo::Vec3f near_bottom_left(-1, -1, 1);
		ufo::Vec3f near_bottom_right(1, -1, 1);
		ufo::Vec3f far_top_right(2, 2, 2);
		ufo::Vec3f far_top_left(-2, 2, 2);
		ufo::Vec3f far_bottom_left(-2, -2, 2);
		ufo::Vec3f far_bottom_right(2, -2, 2);

		// Signature: far_top_right, far_top_left, far_bottom_left, far_bottom_right,
		// near_top_right, near_top_left, near_bottom_left, near_bottom_right
		ufo::Frustum3f f(far_top_right, far_top_left, far_bottom_left, far_bottom_right,
		                 near_top_right, near_top_left, near_bottom_left, near_bottom_right);

		auto cs = ufo::corners(f);

		REQUIRE(cs[0].x() == Catch::Approx(far_top_right.x()));
		REQUIRE(cs[0].y() == Catch::Approx(far_top_right.y()));
		REQUIRE(cs[0].z() == Catch::Approx(far_top_right.z()));

		REQUIRE(cs[1].x() == Catch::Approx(far_top_left.x()));
		REQUIRE(cs[1].y() == Catch::Approx(far_top_left.y()));
		REQUIRE(cs[1].z() == Catch::Approx(far_top_left.z()));

		REQUIRE(cs[2].x() == Catch::Approx(far_bottom_left.x()));
		REQUIRE(cs[2].y() == Catch::Approx(far_bottom_left.y()));
		REQUIRE(cs[2].z() == Catch::Approx(far_bottom_left.z()));

		REQUIRE(cs[3].x() == Catch::Approx(far_bottom_right.x()));
		REQUIRE(cs[3].y() == Catch::Approx(far_bottom_right.y()));
		REQUIRE(cs[3].z() == Catch::Approx(far_bottom_right.z()));

		REQUIRE(cs[4].x() == Catch::Approx(near_top_right.x()));
		REQUIRE(cs[4].y() == Catch::Approx(near_top_right.y()));
		REQUIRE(cs[4].z() == Catch::Approx(near_top_right.z()));

		REQUIRE(cs[5].x() == Catch::Approx(near_top_left.x()));
		REQUIRE(cs[5].y() == Catch::Approx(near_top_left.y()));
		REQUIRE(cs[5].z() == Catch::Approx(near_top_left.z()));

		REQUIRE(cs[6].x() == Catch::Approx(near_bottom_left.x()));
		REQUIRE(cs[6].y() == Catch::Approx(near_bottom_left.y()));
		REQUIRE(cs[6].z() == Catch::Approx(near_bottom_left.z()));

		REQUIRE(cs[7].x() == Catch::Approx(near_bottom_right.x()));
		REQUIRE(cs[7].y() == Catch::Approx(near_bottom_right.y()));
		REQUIRE(cs[7].z() == Catch::Approx(near_bottom_right.z()));
	}

	SECTION("min/max")
	{
		ufo::Vec3f near_top_right(1, 1, 1);
		ufo::Vec3f near_top_left(-1, 1, 1);
		ufo::Vec3f near_bottom_left(-1, -1, 1);
		ufo::Vec3f near_bottom_right(1, -1, 1);
		ufo::Vec3f far_top_right(2, 2, 2);
		ufo::Vec3f far_top_left(-2, 2, 2);
		ufo::Vec3f far_bottom_left(-2, -2, 2);
		ufo::Vec3f far_bottom_right(2, -2, 2);

		ufo::Frustum3f f(far_top_right, far_top_left, far_bottom_left, far_bottom_right,
		                 near_top_right, near_top_left, near_bottom_left, near_bottom_right);

		auto min_result = ufo::min(f);
		auto max_result = ufo::max(f);

		ufo::Vec3f min_expected(-2, -2, 1);
		ufo::Vec3f max_expected(2, 2, 2);

		REQUIRE(min_result.x() == Catch::Approx(min_expected.x()));
		REQUIRE(min_result.y() == Catch::Approx(min_expected.y()));
		REQUIRE(min_result.z() == Catch::Approx(min_expected.z()));

		REQUIRE(max_result.x() == Catch::Approx(max_expected.x()));
		REQUIRE(max_result.y() == Catch::Approx(max_expected.y()));
		REQUIRE(max_result.z() == Catch::Approx(max_expected.z()));
	}
}
