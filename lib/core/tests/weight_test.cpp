// UFO
#include <ufo/core/weight.hpp>

// STL
#include <cmath>
#include <limits>
#include <sstream>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [weight] Weight struct")
{
	SECTION("Default construction")
	{
		ufo::Weight w_default;
		REQUIRE(static_cast<float>(w_default) == Catch::Approx(0.0f));
	}

	SECTION("Value construction and conversion")
	{
		ufo::Weight w_val(0.42f);
		REQUIRE(static_cast<float>(w_val) == Catch::Approx(0.42f));
		float f = w_val;
		REQUIRE(f == Catch::Approx(0.42f));
	}

	SECTION("Comparison operators")
	{
		ufo::Weight w1(0.5f), w2(0.5f), w3(0.7f);
		REQUIRE(w1 == w2);
		REQUIRE(w1 != w3);
		REQUIRE((w1 < w3));
		REQUIRE((w3 > w2));
		REQUIRE((w1 <= w2));
		REQUIRE((w3 >= w1));
	}

	SECTION("Ostream streaming")
	{
		ufo::Weight        w(0.123f);
		std::ostringstream oss;
		oss << w;
		REQUIRE(oss.str() == "0.123");
	}

	SECTION("std::format formatting")
	{
		ufo::Weight w(0.123456f);
		auto        s = std::format("{:.4f}", w);
		REQUIRE(s == "0.1235");
	}

	SECTION("Boundary values and precision")
	{
		ufo::Weight w_max(std::numeric_limits<float>::max());
		ufo::Weight w_min(std::numeric_limits<float>::min());
		ufo::Weight w_eps(std::numeric_limits<float>::epsilon());
		REQUIRE(static_cast<float>(w_max) ==
		        Catch::Approx(std::numeric_limits<float>::max()));
		REQUIRE(static_cast<float>(w_min) ==
		        Catch::Approx(std::numeric_limits<float>::min()));
		REQUIRE(static_cast<float>(w_eps) ==
		        Catch::Approx(std::numeric_limits<float>::epsilon()));
	}

	SECTION("Property-based round-trip conversion")
	{
		for (float v = 0.0f; v <= 1.0f; v += 0.01f) {
			ufo::Weight w(v);
			REQUIRE(static_cast<float>(w) == Catch::Approx(v).epsilon(1e-6));
		}
	}

	SECTION("Randomized input stress test")
	{
		for (int i = 0; i < 100; ++i) {
			float       v = static_cast<float>(rand()) / RAND_MAX;
			ufo::Weight w_val(v);
			REQUIRE(static_cast<float>(w_val) == Catch::Approx(v).epsilon(1e-6));
		}
	}
}
