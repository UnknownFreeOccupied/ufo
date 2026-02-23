// UFO
#include <ufo/core/confidence.hpp>

// STL
#include <cmath>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [confidence] Confidence struct")
{
	SECTION("Default construction")
	{
		ufo::Confidence c_default;
		REQUIRE(static_cast<float>(c_default) == Catch::Approx(0.0f));
	}

	SECTION("Value construction and conversion")
	{
		ufo::Confidence c_val(0.42f);
		REQUIRE(static_cast<float>(c_val) == Catch::Approx(0.42f));
		float f = c_val;
		REQUIRE(f == Catch::Approx(0.42f));
	}

	SECTION("Comparison operators")
	{
		ufo::Confidence c1(0.5f);
		ufo::Confidence c2(0.5f);
		ufo::Confidence c3(0.7f);
		REQUIRE(c1 == c2);
		REQUIRE(c1 != c3);
		REQUIRE((c1 < c3));
		REQUIRE((c3 > c2));
		REQUIRE((c1 <= c2));
		REQUIRE((c3 >= c1));
	}

	SECTION("Ostream streaming")
	{
		ufo::Confidence    c(0.123f);
		std::ostringstream oss;
		oss << c;
		REQUIRE(oss.str() == "0.123");
	}

	SECTION("std::format formatting")
	{
		ufo::Confidence c(0.123456f);
		auto            s = std::format("{:.4f}", c);
		REQUIRE(s == "0.1235");
	}

	SECTION("Boundary values and precision")
	{
		ufo::Confidence c_max(std::numeric_limits<float>::max());
		ufo::Confidence c_min(std::numeric_limits<float>::min());
		ufo::Confidence c_eps(std::numeric_limits<float>::epsilon());
		REQUIRE(static_cast<float>(c_max) ==
		        Catch::Approx(std::numeric_limits<float>::max()));
		REQUIRE(static_cast<float>(c_min) ==
		        Catch::Approx(std::numeric_limits<float>::min()));
		REQUIRE(static_cast<float>(c_eps) ==
		        Catch::Approx(std::numeric_limits<float>::epsilon()));
	}

	SECTION("Property-based round-trip conversion")
	{
		for (float v = 0.0f; v <= 1.0f; v += 0.01f) {
			ufo::Confidence c(v);
			REQUIRE(static_cast<float>(c) == Catch::Approx(v).epsilon(1e-6));
		}
	}

	SECTION("Randomized input stress test")
	{
		for (int i = 0; i < 100; ++i) {
			float           v = static_cast<float>(rand()) / RAND_MAX;
			ufo::Confidence c(v);
			REQUIRE(static_cast<float>(c) == Catch::Approx(v).epsilon(1e-6));
		}
	}
}