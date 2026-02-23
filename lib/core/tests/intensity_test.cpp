// UFO
#include <ufo/core/intensity.hpp>

// STL
#include <cmath>
#include <limits>
#include <sstream>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [intensity] Intensity struct")
{
	SECTION("Default construction")
	{
		ufo::Intensity i_default;
		REQUIRE(static_cast<float>(i_default) == Catch::Approx(0.0f));
	}

	SECTION("Value construction and conversion")
	{
		ufo::Intensity i_val(0.42f);
		REQUIRE(static_cast<float>(i_val) == Catch::Approx(0.42f));
		float f = i_val;
		REQUIRE(f == Catch::Approx(0.42f));
	}

	SECTION("Comparison operators")
	{
		ufo::Intensity i1(0.5f), i2(0.5f), i3(0.7f);
		REQUIRE(i1 == i2);
		REQUIRE(i1 != i3);
		REQUIRE((i1 < i3));
		REQUIRE((i3 > i2));
		REQUIRE((i1 <= i2));
		REQUIRE((i3 >= i1));
	}

	SECTION("Ostream streaming")
	{
		ufo::Intensity     i(0.123f);
		std::ostringstream oss;
		oss << i;
		REQUIRE(oss.str() == "0.123");
	}

	SECTION("std::format formatting")
	{
		ufo::Intensity i(0.123456f);
		auto           s = std::format("{:.4f}", i);
		REQUIRE(s == "0.1235");
	}

	SECTION("Boundary values and precision")
	{
		ufo::Intensity i_max(std::numeric_limits<float>::max());
		ufo::Intensity i_min(std::numeric_limits<float>::min());
		ufo::Intensity i_eps(std::numeric_limits<float>::epsilon());
		REQUIRE(static_cast<float>(i_max) ==
		        Catch::Approx(std::numeric_limits<float>::max()));
		REQUIRE(static_cast<float>(i_min) ==
		        Catch::Approx(std::numeric_limits<float>::min()));
		REQUIRE(static_cast<float>(i_eps) ==
		        Catch::Approx(std::numeric_limits<float>::epsilon()));
	}

	SECTION("Property-based round-trip conversion")
	{
		for (float v = 0.0f; v <= 1.0f; v += 0.01f) {
			ufo::Intensity i(v);
			REQUIRE(static_cast<float>(i) == Catch::Approx(v).epsilon(1e-6));
		}
	}
}
