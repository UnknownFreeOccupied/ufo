// UFO
#include <ufo/core/semantic.hpp>

// STL
#include <limits>
#include <sstream>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [semantic] Semantic struct")
{
	SECTION("Default construction")
	{
		ufo::Semantic s_default;
		REQUIRE(static_cast<std::uint32_t>(s_default.label) == 0u);
		REQUIRE(static_cast<float>(s_default.confidence) == Catch::Approx(0.0f));
	}

	SECTION("Value construction and conversion")
	{
		ufo::Semantic s_val(ufo::Label(42u), ufo::Confidence(0.42f));
		REQUIRE(static_cast<std::uint32_t>(s_val.label) == 42u);
		REQUIRE(static_cast<float>(s_val.confidence) == Catch::Approx(0.42f));
	}

	SECTION("Comparison operators")
	{
		ufo::Semantic s1(ufo::Label(5u), ufo::Confidence(0.5f));
		ufo::Semantic s2(ufo::Label(5u), ufo::Confidence(0.5f));
		ufo::Semantic s3(ufo::Label(7u), ufo::Confidence(0.7f));
		REQUIRE(s1 == s2);
		REQUIRE(s1 != s3);
		REQUIRE((s1 < s3));
		REQUIRE((s3 > s2));
		REQUIRE((s1 <= s2));
		REQUIRE((s3 >= s1));
	}

	SECTION("Ostream streaming")
	{
		ufo::Semantic      s(ufo::Label(123u), ufo::Confidence(0.123f));
		std::ostringstream oss;
		oss << s;
		REQUIRE(oss.str() == "123: 0.123");
	}

	SECTION("std::format formatting")
	{
		ufo::Semantic s(ufo::Label(123u), ufo::Confidence(0.123456f));
		auto          str = std::format("{}", s);
		REQUIRE(str == "123: 0.123456");
		auto str2 = std::format("{:05d}: {:.4f}", s.label, s.confidence);
		REQUIRE(str2 == "00123: 0.1235");
	}

	SECTION("Boundary values and precision")
	{
		ufo::Semantic s_max(ufo::Label{std::numeric_limits<std::uint32_t>::max()},
		                    ufo::Confidence{std::numeric_limits<float>::max()});
		ufo::Semantic s_min(ufo::Label{std::numeric_limits<std::uint32_t>::min()},
		                    ufo::Confidence{std::numeric_limits<float>::min()});
		REQUIRE(static_cast<std::uint32_t>(s_max.label) ==
		        std::numeric_limits<std::uint32_t>::max());
		REQUIRE(static_cast<float>(s_max.confidence) ==
		        Catch::Approx(std::numeric_limits<float>::max()));
		REQUIRE(static_cast<std::uint32_t>(s_min.label) ==
		        std::numeric_limits<std::uint32_t>::min());
		REQUIRE(static_cast<float>(s_min.confidence) ==
		        Catch::Approx(std::numeric_limits<float>::min()));
	}

	SECTION("Property-based round-trip construction")
	{
		for (std::uint32_t l = 0u; l < 100u; ++l) {
			float         c = static_cast<float>(l) / 100.0f;
			ufo::Semantic s(ufo::Label{l}, ufo::Confidence{c});
			REQUIRE(static_cast<std::uint32_t>(s.label) == l);
			REQUIRE(static_cast<float>(s.confidence) == Catch::Approx(c).epsilon(1e-6));
		}
	}

	SECTION("Randomized input stress test")
	{
		for (int i = 0; i < 100; ++i) {
			std::uint32_t l = static_cast<std::uint32_t>(rand());
			float         c = static_cast<float>(rand()) / RAND_MAX;
			ufo::Semantic s(ufo::Label{l}, ufo::Confidence{c});
			REQUIRE(static_cast<std::uint32_t>(s.label) == l);
			REQUIRE(static_cast<float>(s.confidence) == Catch::Approx(c).epsilon(1e-6));
		}
	}
}
