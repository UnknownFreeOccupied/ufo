// UFO
#include <ufo/core/normal.hpp>

// STL
#include <sstream>
#include <type_traits>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [normal] Type aliases")
{
	STATIC_REQUIRE(std::is_same_v<ufo::Normal2f, ufo::Normal<2, float>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Normal3f, ufo::Normal<3, float>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Normal4f, ufo::Normal<4, float>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Normal2d, ufo::Normal<2, double>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Normal3d, ufo::Normal<3, double>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Normal4d, ufo::Normal<4, double>>);
}

TEST_CASE("[core] [normal] Is-a Vec")
{
	STATIC_REQUIRE(std::is_base_of_v<ufo::Vec<2, float>, ufo::Normal2f>);
	STATIC_REQUIRE(std::is_base_of_v<ufo::Vec<3, float>, ufo::Normal3f>);
	STATIC_REQUIRE(std::is_base_of_v<ufo::Vec<4, float>, ufo::Normal4f>);
	STATIC_REQUIRE(std::is_base_of_v<ufo::Vec<3, double>, ufo::Normal3d>);
}

TEST_CASE("[core] [normal] Default construction")
{
	SECTION("3D float")
	{
		ufo::Normal3f n{};
		REQUIRE(n[0] == Catch::Approx(0.0f));
		REQUIRE(n[1] == Catch::Approx(0.0f));
		REQUIRE(n[2] == Catch::Approx(0.0f));
	}

	SECTION("2D float")
	{
		ufo::Normal2f n{};
		REQUIRE(n[0] == Catch::Approx(0.0f));
		REQUIRE(n[1] == Catch::Approx(0.0f));
	}

	SECTION("3D double")
	{
		ufo::Normal3d n{};
		REQUIRE(n[0] == Catch::Approx(0.0));
		REQUIRE(n[1] == Catch::Approx(0.0));
		REQUIRE(n[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[core] [normal] Component construction")
{
	SECTION("3D float — axis-aligned")
	{
		ufo::Normal3f nx{1.0f, 0.0f, 0.0f};
		REQUIRE(nx[0] == Catch::Approx(1.0f));
		REQUIRE(nx[1] == Catch::Approx(0.0f));
		REQUIRE(nx[2] == Catch::Approx(0.0f));

		ufo::Normal3f ny{0.0f, 1.0f, 0.0f};
		REQUIRE(ny[0] == Catch::Approx(0.0f));
		REQUIRE(ny[1] == Catch::Approx(1.0f));
		REQUIRE(ny[2] == Catch::Approx(0.0f));

		ufo::Normal3f nz{0.0f, 0.0f, 1.0f};
		REQUIRE(nz[0] == Catch::Approx(0.0f));
		REQUIRE(nz[1] == Catch::Approx(0.0f));
		REQUIRE(nz[2] == Catch::Approx(1.0f));
	}

	SECTION("3D float — diagonal (not normalized, caller's responsibility)")
	{
		ufo::Normal3f n{1.0f, 1.0f, 0.0f};
		REQUIRE(n[0] == Catch::Approx(1.0f));
		REQUIRE(n[1] == Catch::Approx(1.0f));
		REQUIRE(n[2] == Catch::Approx(0.0f));
	}

	SECTION("2D float")
	{
		ufo::Normal2f n{0.0f, 1.0f};
		REQUIRE(n[0] == Catch::Approx(0.0f));
		REQUIRE(n[1] == Catch::Approx(1.0f));
	}

	SECTION("3D double")
	{
		ufo::Normal3d n{0.0, 0.0, 1.0};
		REQUIRE(n[0] == Catch::Approx(0.0));
		REQUIRE(n[1] == Catch::Approx(0.0));
		REQUIRE(n[2] == Catch::Approx(1.0));
	}

	SECTION("4D float")
	{
		ufo::Normal4f n{0.0f, 0.0f, 1.0f, 0.0f};
		REQUIRE(n[0] == Catch::Approx(0.0f));
		REQUIRE(n[1] == Catch::Approx(0.0f));
		REQUIRE(n[2] == Catch::Approx(1.0f));
		REQUIRE(n[3] == Catch::Approx(0.0f));
	}
}

TEST_CASE("[core] [normal] Construction from Vec")
{
	SECTION("Normal3f from Vec3f")
	{
		ufo::Vec3f    v{0.0f, 1.0f, 0.0f};
		ufo::Normal3f n{v};
		REQUIRE(n[0] == Catch::Approx(0.0f));
		REQUIRE(n[1] == Catch::Approx(1.0f));
		REQUIRE(n[2] == Catch::Approx(0.0f));
	}

	SECTION("Normal2f from Vec2f")
	{
		ufo::Vec2f    v{1.0f, 0.0f};
		ufo::Normal2f n{v};
		REQUIRE(n[0] == Catch::Approx(1.0f));
		REQUIRE(n[1] == Catch::Approx(0.0f));
	}

	SECTION("Normal3d from Vec3d")
	{
		ufo::Vec3d    v{0.0, 0.0, 1.0};
		ufo::Normal3d n{v};
		REQUIRE(n[2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[core] [normal] Equality")
{
	ufo::Normal3f a{1.0f, 0.0f, 0.0f};
	ufo::Normal3f b{1.0f, 0.0f, 0.0f};
	ufo::Normal3f c{0.0f, 1.0f, 0.0f};

	REQUIRE(a == b);
	REQUIRE_FALSE(a == c);
	REQUIRE(a != c);
	REQUIRE_FALSE(a != b);
}

TEST_CASE("[core] [normal] Inherits Vec arithmetic")
{
	SECTION("Addition")
	{
		ufo::Normal3f a{1.0f, 0.0f, 0.0f};
		ufo::Normal3f b{0.0f, 1.0f, 0.0f};
		auto          c = a + b;
		REQUIRE(c[0] == Catch::Approx(1.0f));
		REQUIRE(c[1] == Catch::Approx(1.0f));
		REQUIRE(c[2] == Catch::Approx(0.0f));
	}

	SECTION("Scalar multiplication")
	{
		ufo::Normal3f n{1.0f, 2.0f, 3.0f};
		auto          scaled = n * 2.0f;
		REQUIRE(scaled[0] == Catch::Approx(2.0f));
		REQUIRE(scaled[1] == Catch::Approx(4.0f));
		REQUIRE(scaled[2] == Catch::Approx(6.0f));
	}

	SECTION("Negation")
	{
		ufo::Normal3f n{0.0f, 0.0f, 1.0f};
		auto          neg = -n;
		REQUIRE(neg[0] == Catch::Approx(0.0f));
		REQUIRE(neg[1] == Catch::Approx(0.0f));
		REQUIRE(neg[2] == Catch::Approx(-1.0f));
	}
}

TEST_CASE("[core] [normal] std::format delegates to Vec formatter (Normal4f)")
{
	// Vec's formatter iterates over the four named components x, y, z, w.
	// We use Normal4f here because it has exactly 4 components, matching
	// the formatter's internal names array.
	SECTION("Axis-aligned normals")
	{
		REQUIRE(std::format("{}", ufo::Normal4f{1.0f, 0.0f, 0.0f, 0.0f}) ==
		        "x: 1 y: 0 z: 0 w: 0");
		REQUIRE(std::format("{}", ufo::Normal4f{0.0f, 1.0f, 0.0f, 0.0f}) ==
		        "x: 0 y: 1 z: 0 w: 0");
		REQUIRE(std::format("{}", ufo::Normal4f{0.0f, 0.0f, 0.0f, 1.0f}) ==
		        "x: 0 y: 0 z: 0 w: 1");
	}

	SECTION("Mixed values")
	{
		REQUIRE(std::format("{}", ufo::Normal4f{0.5f, -0.5f, 0.5f, -0.5f}) ==
		        "x: 0.5 y: -0.5 z: 0.5 w: -0.5");
	}
}

TEST_CASE("[core] [normal] operator<< delegates to Vec formatter (Normal4f)")
{
	SECTION("Axis-aligned normal")
	{
		std::ostringstream oss;
		oss << ufo::Normal4f{0.0f, 0.0f, 1.0f, 0.0f};
		REQUIRE(oss.str() == "x: 0 y: 0 z: 1 w: 0");
	}
}
