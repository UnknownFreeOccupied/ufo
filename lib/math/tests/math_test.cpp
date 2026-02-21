// UFO
#include <ufo/math/math.hpp>

// STL
#include <cmath>
#include <numbers>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[math] [sign] Sign function")
{
	SECTION("Positive integer") { REQUIRE(ufo::sign(5) == 1); }

	SECTION("Negative integer") { REQUIRE(ufo::sign(-5) == -1); }

	SECTION("Zero integer") { REQUIRE(ufo::sign(0) == 0); }

	SECTION("Positive unsigned") { REQUIRE(ufo::sign<unsigned>(5u) == 1); }

	SECTION("Zero unsigned") { REQUIRE(ufo::sign<unsigned>(0u) == 0); }

	SECTION("Positive double") { REQUIRE(ufo::sign(3.14) == 1); }

	SECTION("Negative double") { REQUIRE(ufo::sign(-3.14) == -1); }

	SECTION("Zero double") { REQUIRE(ufo::sign(0.0) == 0); }
}

TEST_CASE("[math] [radians] Degrees to radians conversion")
{
	SECTION("180 degrees equals pi")
	{
		REQUIRE(ufo::radians(180.0) == Catch::Approx(std::numbers::pi));
	}

	SECTION("90 degrees equals pi/2")
	{
		REQUIRE(ufo::radians(90.0) == Catch::Approx(std::numbers::pi / 2.0));
	}

	SECTION("360 degrees equals 2*pi")
	{
		REQUIRE(ufo::radians(360.0) == Catch::Approx(2.0 * std::numbers::pi));
	}

	SECTION("0 degrees equals 0") { REQUIRE(ufo::radians(0.0) == Catch::Approx(0.0)); }

	SECTION("Negative degrees")
	{
		REQUIRE(ufo::radians(-90.0) == Catch::Approx(-std::numbers::pi / 2.0));
	}
}

TEST_CASE("[math] [degrees] Radians to degrees conversion")
{
	SECTION("pi radians equals 180 degrees")
	{
		REQUIRE(ufo::degrees(std::numbers::pi) == Catch::Approx(180.0));
	}

	SECTION("pi/2 radians equals 90 degrees")
	{
		REQUIRE(ufo::degrees(std::numbers::pi / 2.0) == Catch::Approx(90.0));
	}

	SECTION("2*pi radians equals 360 degrees")
	{
		REQUIRE(ufo::degrees(2.0 * std::numbers::pi) == Catch::Approx(360.0));
	}

	SECTION("0 radians equals 0 degrees")
	{
		REQUIRE(ufo::degrees(0.0) == Catch::Approx(0.0));
	}

	SECTION("Negative radians")
	{
		REQUIRE(ufo::degrees(-std::numbers::pi) == Catch::Approx(-180.0));
	}

	SECTION("radians and degrees are inverse operations")
	{
		REQUIRE(ufo::degrees(ufo::radians(45.0)) == Catch::Approx(45.0));
		REQUIRE(ufo::radians(ufo::degrees(1.0)) == Catch::Approx(1.0));
	}
}

TEST_CASE("[math] [ipow] Integer power function")
{
	SECTION("2^3 = 8") { REQUIRE(ufo::ipow(2, 3) == 8); }

	SECTION("2^0 = 1") { REQUIRE(ufo::ipow(2, 0) == 1); }

	SECTION("0^0 = 1") { REQUIRE(ufo::ipow(0, 0) == 1); }

	SECTION("2.0^(-3) = 0.125") { REQUIRE(ufo::ipow(2.0, -3) == Catch::Approx(0.125)); }

	SECTION("(-2)^3 = -8") { REQUIRE(ufo::ipow(-2, 3) == -8); }

	SECTION("(-2)^2 = 4") { REQUIRE(ufo::ipow(-2, 2) == 4); }

	SECTION("5^2 = 25") { REQUIRE(ufo::ipow(5, 2) == 25); }

	SECTION("10.0^(-1) = 0.1") { REQUIRE(ufo::ipow(10.0, -1) == Catch::Approx(0.1)); }
}

TEST_CASE("[math] [probabilityToLogit] Probability to logit conversion")
{
	SECTION("p=0.5 maps to logit 0")
	{
		REQUIRE(ufo::probabilityToLogit(0.5) == Catch::Approx(0.0));
	}

	SECTION("p=0.0 maps to negative infinity")
	{
		auto result = ufo::probabilityToLogit(0.0);
		REQUIRE(std::isinf(result));
		REQUIRE(result < 0.0);
	}

	SECTION("p=1.0 maps to positive infinity")
	{
		auto result = ufo::probabilityToLogit(1.0);
		REQUIRE(std::isinf(result));
		REQUIRE(result > 0.0);
	}

	SECTION("p > 0.5 maps to positive logit")
	{
		auto result = ufo::probabilityToLogit(0.9);
		REQUIRE(!std::isinf(result));
		REQUIRE(result > 0.0);
	}

	SECTION("p < 0.5 maps to negative logit")
	{
		auto result = ufo::probabilityToLogit(0.1);
		REQUIRE(!std::isinf(result));
		REQUIRE(result < 0.0);
	}

	SECTION("Known value: logit(0.731) = 1.0 approximately")
	{
		double p = 1.0 / (1.0 + std::exp(-1.0));
		REQUIRE(ufo::probabilityToLogit(p) == Catch::Approx(1.0));
	}
}

TEST_CASE("[math] [logitToProbability] Logit to probability conversion")
{
	SECTION("logit 0 maps to probability 0.5")
	{
		REQUIRE(ufo::logitToProbability(0.0) == Catch::Approx(0.5));
	}

	SECTION("Large positive logit maps to probability near 1")
	{
		REQUIRE(ufo::logitToProbability(100.0) == Catch::Approx(1.0).epsilon(1e-6));
	}

	SECTION("Large negative logit maps to probability near 0")
	{
		REQUIRE(ufo::logitToProbability(-100.0) == Catch::Approx(0.0).margin(1e-6));
	}

	SECTION("Positive logit maps to probability > 0.5")
	{
		REQUIRE(ufo::logitToProbability(1.0) > 0.5);
	}

	SECTION("Negative logit maps to probability < 0.5")
	{
		REQUIRE(ufo::logitToProbability(-1.0) < 0.5);
	}

	SECTION("Round-trip: logitToProbability(probabilityToLogit(p)) == p")
	{
		for (double p = 0.05; p < 1.0; p += 0.05) {
			REQUIRE(ufo::logitToProbability(ufo::probabilityToLogit(p)) ==
			        Catch::Approx(p).epsilon(1e-10));
		}
	}
}
