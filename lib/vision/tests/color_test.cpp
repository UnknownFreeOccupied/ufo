// UFO
#include <ufo/vision/color.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Color")
{
	SECTION("Basic Properties")
	{
		FineRgb rgb(1.0f, 0.5f, 0.25f);
		CHECK(rgb.red == 1.0f);
		CHECK(rgb.green == 0.5f);
		CHECK(rgb.blue == 0.25f);
	}

	SECTION("Conversion")
	{
		FineRgb  rgb(1.0f, 1.0f, 1.0f);
		FineGray gray = convert<FineGray>(rgb);
		CHECK(gray.gray == Catch::Approx(1.0f));
	}

	SECTION("Arithmetic")
	{
		FineRgb a(0.1f, 0.2f, 0.3f);
		FineRgb b(0.4f, 0.5f, 0.6f);
		FineRgb c = a + b;
		CHECK(c.red == Catch::Approx(0.5f));
		CHECK(c.green == Catch::Approx(0.7f));
		CHECK(c.blue == Catch::Approx(0.9f));
	}
}