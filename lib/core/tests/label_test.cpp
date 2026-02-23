// UFO
#include <ufo/core/label.hpp>

// STL
#include <cstdint>
#include <limits>
#include <sstream>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [label] Label struct")
{
	SECTION("Default construction")
	{
		ufo::Label l_default;
		REQUIRE(static_cast<std::uint32_t>(l_default) == 0u);
	}

	SECTION("Value construction and conversion")
	{
		ufo::Label l_val(42u);
		REQUIRE(static_cast<std::uint32_t>(l_val) == 42u);
		std::uint32_t v = l_val;
		REQUIRE(v == 42u);
	}

	SECTION("Comparison operators")
	{
		ufo::Label l1(5u), l2(5u), l3(7u);
		REQUIRE(l1 == l2);
		REQUIRE(l1 != l3);
		REQUIRE((l1 < l3));
		REQUIRE((l3 > l2));
		REQUIRE((l1 <= l2));
		REQUIRE((l3 >= l1));
	}

	SECTION("Ostream streaming")
	{
		ufo::Label         l(123u);
		std::ostringstream oss;
		oss << l;
		REQUIRE(oss.str() == "123");
	}

	SECTION("std::format formatting")
	{
		ufo::Label l(123u);
		auto       s = std::format("{:05d}", l);
		REQUIRE(s == "00123");
	}

	SECTION("Boundary values and precision")
	{
		ufo::Label l_max(std::numeric_limits<std::uint32_t>::max());
		ufo::Label l_min(std::numeric_limits<std::uint32_t>::min());
		REQUIRE(static_cast<std::uint32_t>(l_max) ==
		        std::numeric_limits<std::uint32_t>::max());
		REQUIRE(static_cast<std::uint32_t>(l_min) ==
		        std::numeric_limits<std::uint32_t>::min());
	}

	SECTION("Property-based round-trip conversion")
	{
		for (std::uint32_t v = 0u; v < 100u; ++v) {
			ufo::Label l(v);
			REQUIRE(static_cast<std::uint32_t>(l) == v);
		}
	}

	SECTION("Randomized input stress test")
	{
		for (int i = 0; i < 100; ++i) {
			std::uint32_t v = static_cast<std::uint32_t>(rand());
			ufo::Label    l(v);
			REQUIRE(static_cast<std::uint32_t>(l) == v);
		}
	}
}
