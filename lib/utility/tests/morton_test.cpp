// UFO
#include <ufo/utility/morton.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <limits>

TEST_CASE("Morton")
{
	using namespace ufo;

	REQUIRE(std::uint32_t(23132) == mortonCompact2(mortonSpread2(std::uint32_t(23132))));
	REQUIRE(std::uint32_t(23132) == mortonCompact3(mortonSpread3(std::uint32_t(23132))));
	REQUIRE(std::uint16_t(23132) == mortonCompact4(mortonSpread4(std::uint16_t(23132))));

	// for (std::uint64_t v{}; std::numeric_limits<std::uint32_t>::max() >= v; ++v) {
	// 	REQUIRE(v == mortonCompact2(mortonSpread2(v)));
	// }

	for (std::uint32_t v{}; 0x1FFFFF >= v; ++v) {
		REQUIRE(v == mortonCompact3(mortonSpread3(v)));
	}

	for (std::uint32_t v{}; std::numeric_limits<std::uint16_t>::max() >= v; ++v) {
		REQUIRE(v == mortonCompact4(mortonSpread4(v)));
	}
}