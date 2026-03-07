// UFO
#include <ufo/morton/morton.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

template <std::size_t Dim>
void testMorton()
{
	using M = ufo::Morton<Dim>;

	ufo::Vec<Dim, std::uint32_t> v;
	for (std::uint32_t i = 0; i < Dim; ++i) {
		v[i] = i * 10 + 1;
	}

	SECTION(std::format("Dim {} - 32-bit Encode/Decode", Dim))
	{
		auto code = M::encode32(v);
		auto dec  = M::decode32(code);
		REQUIRE(v == dec);
	}

	SECTION(std::format("Dim {} - 64-bit Encode/Decode", Dim))
	{
		auto code = M::encode64(v);
		auto dec  = M::decode64(code);
		REQUIRE(v == dec);
	}

	SECTION(std::format("Dim {} - Spread/Compact 32", Dim))
	{
		for (std::uint32_t i = 0; i < Dim; ++i) {
			auto s = M::spread32(v[i]);
			auto c = M::compact32(s);
			REQUIRE(v[i] == c);
		}
	}

	SECTION(std::format("Dim {} - Spread/Compact 64", Dim))
	{
		for (std::uint32_t i = 0; i < Dim; ++i) {
			auto s = M::spread64(v[i]);
			auto c = M::compact64(s);
			REQUIRE(v[i] == c);
		}
	}
}

TEST_CASE("Morton - Generic Implementation")
{
	SECTION("Dim 1") { testMorton<1>(); }
	SECTION("Dim 2") { testMorton<2>(); }
	SECTION("Dim 3") { testMorton<3>(); }
	SECTION("Dim 4") { testMorton<4>(); }

	SECTION("Dim 3 - Specific Exhaustive (Small Range)")
	{
		for (std::uint32_t x = 0; x < 32; ++x) {
			for (std::uint32_t y = 0; y < 32; ++y) {
				for (std::uint32_t z = 0; z < 32; ++z) {
					ufo::Vec3u v(x, y, z);
					auto       code = ufo::Morton<3>::encode64(v);
					auto       dec  = ufo::Morton<3>::decode64(code);
					REQUIRE(v == dec);
				}
			}
		}
	}
}